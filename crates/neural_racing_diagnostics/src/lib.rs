//! Best-effort local evidence. No disk I/O or blocking sends in event producers.
//! Initialize once in the executable; library callers are otherwise silent.
pub use serde_json::{json, Value};
use std::{
    fs::{self, File, OpenOptions},
    io::{self, Write},
    path::{Path, PathBuf},
    sync::{
        atomic::{AtomicBool, AtomicU64, Ordering::Relaxed},
        mpsc::{self, SyncSender},
        Arc, OnceLock,
    },
    time::{Duration, Instant, SystemTime, UNIX_EPOCH},
};
const SEGMENT_BYTES: u64 = 2 * 1024 * 1024;
const RUN_SEGMENTS: u64 = 32;
const STORE_BYTES: u64 = 256 * 1024 * 1024;
const RETENTION: Duration = Duration::from_secs(7 * 86400);
const MAX_EVENT_BYTES: usize = 8192;
static GLOBAL: OnceLock<Recorder> = OnceLock::new();
static IDS: AtomicU64 = AtomicU64::new(1);
thread_local! { static CONTEXT: std::cell::Cell<u64> = const { std::cell::Cell::new(0) }; }
/// A comparison key for non-secret model/geometry bytes; not an integrity or security hash.
pub fn fingerprint(bytes: &[u8]) -> String {
    let hash = bytes.iter().fold(0xcbf29ce484222325u64, |h, b| {
        (h ^ u64::from(*b)).wrapping_mul(0x100000001b3)
    });
    format!("{hash:016x}")
}
pub fn operation() -> u64 {
    IDS.fetch_add(1, Relaxed)
}
pub fn current_operation() -> u64 {
    CONTEXT.with(|c| c.get())
}
pub struct Context(u64);
pub fn context(id: u64) -> Context {
    Context(CONTEXT.with(|c| c.replace(id)))
}
impl Drop for Context {
    fn drop(&mut self) {
        CONTEXT.with(|c| c.set(self.0));
    }
}
pub fn enabled() -> bool {
    GLOBAL.get().is_some()
}
pub fn now_ms() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_millis() as u64
}

#[derive(Default)]
struct Health {
    dropped: AtomicU64,
    io_errors: AtomicU64,
    written: AtomicU64,
    warned: AtomicBool,
}
enum Message {
    Event(Value),
    Flush(mpsc::Sender<()>),
    Stop(mpsc::Sender<()>),
}
struct Recorder {
    sender: SyncSender<Message>,
    health: Arc<Health>,
    sequence: AtomicU64,
    start: Instant,
    run: String,
}
impl Recorder {
    fn emit(&self, name: &'static str, operation: u64, fields: Value) {
        let sequence = self.sequence.fetch_add(1, Relaxed) + 1;
        let event = json!({"schema":1,"run":self.run,"seq":sequence,"ts_ms":now_ms(),
            "elapsed_ms":self.start.elapsed().as_millis() as u64,"event":name,"operation":operation,
            "dropped_total":self.health.dropped.load(Relaxed),"io_errors_total":self.health.io_errors.load(Relaxed),"data":fields});
        if self.sender.try_send(Message::Event(event)).is_err() {
            self.health.dropped.fetch_add(1, Relaxed);
        }
    }
}
pub fn event(name: &'static str, operation: u64, fields: Value) {
    if let Some(r) = GLOBAL.get() {
        r.emit(name, operation, fields);
    }
}
/// Avoid even constructing JSON when diagnostics have not been initialized.
#[macro_export]
macro_rules! emit {
    ($name:expr, $op:expr, $fields:expr) => {
        if $crate::enabled() {
            $crate::event($name, $op, $fields);
        }
    };
}

pub fn default_directory() -> PathBuf {
    std::env::var_os("NEURAL_RACING_DIAGNOSTICS_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|| {
            std::env::var_os("HOME")
                .map(PathBuf::from)
                .unwrap_or_else(std::env::temp_dir)
                .join(".local/state/neural-racing/diagnostics")
        })
}

/// On normal return (or unwind), wait at most two seconds for accepted events.
/// SIGKILL, abort, power failure and std::process::exit bypass this guard.
pub struct Guard;
pub fn init(application: &'static str) -> Guard {
    if std::env::var_os("NEURAL_RACING_DIAGNOSTICS").is_some_and(|v| v == "off") {
        return Guard;
    }
    GLOBAL.get_or_init(|| {
        let directory = default_directory();
        let run = format!("{}-{}-{}", now_ms(), std::process::id(), operation());
        let (sender, receiver) = mpsc::sync_channel(256);
        let health = Arc::new(Health::default());
        let worker_health = health.clone();
        let metadata = json!({"schema":1,"event":"segment_start","run":run,"ts_ms":now_ms(),
            "application":application,"revision":env!("DIAG_REVISION"),"source_fingerprint":env!("DIAG_SOURCE"),
            "version":env!("CARGO_PKG_VERSION"),"profile":env!("DIAG_PROFILE"),
            "os":std::env::consts::OS,"arch":std::env::consts::ARCH,
            "math_backend":if cfg!(target_os="macos") {"accelerate"} else {"rust"},
            "segment_bytes":SEGMENT_BYTES,"run_segments":RUN_SEGMENTS,"store_bytes_soft_limit":STORE_BYTES,
            "retention_days":7,"queue_capacity":256,"event_bytes_limit":MAX_EVENT_BYTES,
            "attempt_sampling":"first 4 then every 1000; totals in progress events",
            "update_sampling":"first 3 then every 100; rollback totals in progress events"});
        let writer_run = run.clone();
        if std::thread::Builder::new().name("diagnostic-writer".into()).spawn(move || {
            writer_loop(directory, writer_run, metadata, receiver, worker_health);
        }).is_err() { warn(&health); }
        let previous = std::panic::take_hook();
        std::panic::set_hook(Box::new(move |info| {
            // Payloads and arbitrary thread names can contain secrets. Keep only location.
            event("panic", current_operation(), json!({"file":info.location().and_then(|l| Path::new(l.file()).file_name()).and_then(|s| s.to_str()),
                "line":info.location().map(|l| l.line()),"payload":"redacted"}));
            flush(Duration::from_millis(300));
            previous(info);
        }));
        let recorder = Recorder { sender, health, sequence: AtomicU64::new(0), start: Instant::now(), run };
        recorder.emit("process_start", 0, json!({}));
        recorder
    });
    Guard
}
/// Returns false if disabled, the writer is unavailable, busy, or exceeds timeout.
pub fn flush(timeout: Duration) -> bool {
    GLOBAL.get().is_some_and(|r| barrier(r, false, timeout))
}

// Control messages may wait within the caller's deadline; normal events never do.
// A full event queue must not prevent the very flush intended to drain it.
fn barrier(recorder: &Recorder, stop: bool, timeout: Duration) -> bool {
    let deadline = Instant::now() + timeout;
    let (tx, rx) = mpsc::channel();
    let mut message = if stop {
        Message::Stop(tx)
    } else {
        Message::Flush(tx)
    };
    loop {
        match recorder.sender.try_send(message) {
            Ok(()) => break,
            Err(mpsc::TrySendError::Disconnected(_)) => return false,
            Err(mpsc::TrySendError::Full(pending)) => {
                if Instant::now() >= deadline {
                    return false;
                }
                message = pending;
                std::thread::sleep(Duration::from_millis(1));
            }
        }
    }
    rx.recv_timeout(deadline.saturating_duration_since(Instant::now()))
        .is_ok()
}
fn shutdown(recorder: &Recorder) {
    barrier(recorder, false, Duration::from_secs(1));
    recorder.emit(
        "process_end",
        0,
        json!({"unwinding":std::thread::panicking(),
        "written_before_end":recorder.health.written.load(Relaxed)}),
    );
    barrier(recorder, true, Duration::from_secs(1));
}
impl Drop for Guard {
    fn drop(&mut self) {
        if let Some(recorder) = GLOBAL.get() {
            shutdown(recorder);
        }
    }
}

fn warn(health: &Health) {
    health.io_errors.fetch_add(1, Relaxed);
    if !health.warned.swap(true, Relaxed) {
        eprintln!("neural-racing: diagnostic evidence is incomplete; local telemetry I/O failed");
    }
}
struct Store {
    dir: PathBuf,
    run: String,
    metadata: Value,
    segment: u64,
    file: Option<File>,
    bytes: u64,
    last_cleanup: Instant,
}
impl Store {
    fn new(dir: PathBuf, run: String, metadata: Value) -> Self {
        Self {
            dir,
            run,
            metadata,
            segment: 0,
            file: None,
            bytes: 0,
            last_cleanup: Instant::now() - Duration::from_secs(60),
        }
    }
    fn open(&mut self) -> io::Result<()> {
        use std::fs::DirBuilder;
        let mut builder = DirBuilder::new();
        builder.recursive(true);
        #[cfg(unix)]
        {
            use std::os::unix::fs::DirBuilderExt;
            builder.mode(0o700);
        }
        builder.create(&self.dir)?;
        let mut options = OpenOptions::new();
        options.write(true).create_new(true);
        #[cfg(unix)]
        {
            use std::os::unix::fs::OpenOptionsExt;
            options.mode(0o600);
        }
        let path = self
            .dir
            .join(format!("nr-{}-{:06}.jsonl", self.run, self.segment));
        let mut file = options.open(path)?;
        file.try_lock().map_err(io::Error::other)?;
        let mut header = self.metadata.clone();
        header["segment"] = json!(self.segment);
        header["ts_ms"] = json!(now_ms());
        let line = serde_json::to_vec(&header)?;
        file.write_all(&line)?;
        file.write_all(b"\n")?;
        self.bytes = line.len() as u64 + 1;
        self.file = Some(file);
        self.cleanup();
        Ok(())
    }
    fn append(&mut self, line: &[u8]) -> io::Result<()> {
        if self.file.is_none() {
            self.open()?;
        }
        if self.bytes + line.len() as u64 + 1 > SEGMENT_BYTES {
            self.sync()?;
            self.file.take();
            self.segment += 1;
            self.open()?;
        }
        if self.last_cleanup.elapsed() >= Duration::from_secs(60) {
            self.cleanup();
        }
        let file = self.file.as_mut().unwrap();
        file.write_all(line)?;
        file.write_all(b"\n")?;
        self.bytes += line.len() as u64 + 1;
        Ok(())
    }
    fn sync(&mut self) -> io::Result<()> {
        if let Some(f) = &self.file {
            f.sync_data()?;
        }
        Ok(())
    }
    fn cleanup(&mut self) {
        self.last_cleanup = Instant::now();
        let Ok(entries) = fs::read_dir(&self.dir) else {
            return;
        };
        let mut files: Vec<_> = entries
            .flatten()
            .filter_map(|e| {
                let name = e.file_name().to_str()?.to_owned();
                if !name.starts_with("nr-")
                    || !name.ends_with(".jsonl")
                    || !e.file_type().ok()?.is_file()
                {
                    return None;
                }
                let meta = e.metadata().ok()?;
                Some((meta.modified().ok()?, meta.len(), name, e.path()))
            })
            .collect();
        files.sort_by_key(|f| f.0);
        let mut total: u64 = files.iter().map(|f| f.1).sum();
        for (modified, size, name, path) in files {
            let own_old = name
                .strip_prefix(&format!("nr-{}-", self.run))
                .and_then(|s| s.strip_suffix(".jsonl"))
                .and_then(|s| s.parse::<u64>().ok())
                .is_some_and(|n| n + RUN_SEGMENTS <= self.segment);
            if own_old || total > STORE_BYTES || modified.elapsed().unwrap_or_default() > RETENTION
            {
                if let Ok(file) = OpenOptions::new().write(true).open(&path) {
                    // Active writers hold an exclusive lock; never unlink their evidence.
                    if file.try_lock().is_ok() && fs::remove_file(path).is_ok() {
                        total = total.saturating_sub(size);
                    }
                }
            }
        }
    }
}
fn writer_loop(
    dir: PathBuf,
    run: String,
    metadata: Value,
    receiver: mpsc::Receiver<Message>,
    health: Arc<Health>,
) {
    let mut store = Store::new(dir, run, metadata);
    let mut last_sync = Instant::now();
    let mut retry = Instant::now();
    let mut failed = false;
    loop {
        match receiver.recv_timeout(Duration::from_millis(200)) {
            Ok(Message::Event(mut event)) => {
                if failed && retry.elapsed() < Duration::from_secs(1) {
                    health.dropped.fetch_add(1, Relaxed);
                    continue;
                }
                event["io_errors_total"] = json!(health.io_errors.load(Relaxed));
                event["dropped_total"] = json!(health.dropped.load(Relaxed));
                let bytes = serde_json::to_vec(&event).unwrap_or_default();
                if bytes.len() > MAX_EVENT_BYTES {
                    health.dropped.fetch_add(1, Relaxed);
                    continue;
                }
                match store.append(&bytes) {
                    Ok(()) => {
                        failed = false;
                        health.written.fetch_add(1, Relaxed);
                    }
                    Err(_) => {
                        warn(&health);
                        health.dropped.fetch_add(1, Relaxed);
                        store.file.take();
                        store.segment += 1;
                        failed = true;
                        retry = Instant::now();
                    }
                }
            }
            Ok(Message::Flush(ack)) => {
                if store.sync().is_ok() && !failed {
                    let _ = ack.send(());
                } else {
                    warn(&health);
                }
            }
            Ok(Message::Stop(ack)) => {
                if store.sync().is_err() {
                    warn(&health);
                }
                let _ = ack.send(());
                break;
            }
            Err(mpsc::RecvTimeoutError::Disconnected) => {
                let _ = store.sync();
                break;
            }
            Err(mpsc::RecvTimeoutError::Timeout) => {}
        }
        if last_sync.elapsed() >= Duration::from_secs(1) {
            if store.sync().is_err() {
                warn(&health);
            }
            last_sync = Instant::now();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn directory() -> PathBuf {
        std::env::temp_dir().join(format!(
            "neural-racing-diag-test-{}-{}",
            std::process::id(),
            operation()
        ))
    }
    #[test]
    fn shutdown_drains_a_full_queue_and_retains_its_end_marker() {
        let dir = directory();
        let (sender, receiver) = mpsc::sync_channel(1);
        let health = Arc::new(Health::default());
        let recorder = Recorder {
            sender,
            health: health.clone(),
            sequence: AtomicU64::new(0),
            start: Instant::now(),
            run: "shutdown".into(),
        };
        recorder.emit("process_start", 0, json!({})); // Fill queue before writer exists.
        let writer_dir = dir.clone();
        let writer = std::thread::spawn(move || {
            std::thread::sleep(Duration::from_millis(20));
            writer_loop(writer_dir, "shutdown".into(), json!({}), receiver, health);
        });
        shutdown(&recorder);
        writer.join().unwrap();
        let file = fs::read_dir(&dir).unwrap().next().unwrap().unwrap().path();
        let records: Vec<Value> = fs::read_to_string(file)
            .unwrap()
            .lines()
            .map(|line| serde_json::from_str(line).unwrap())
            .collect();
        assert!(records.iter().any(|e| e["event"] == "process_end"));
        assert_eq!(recorder.health.dropped.load(Relaxed), 0);
        fs::remove_dir_all(dir).unwrap();
    }

    #[test]
    fn rotation_is_bounded_and_active_file_is_protected() {
        let dir = directory();
        let mut store = Store::new(
            dir.clone(),
            "rotation".into(),
            json!({"event":"segment_start"}),
        );
        let payload = vec![b'x'; SEGMENT_BYTES as usize / 2];
        for _ in 0..RUN_SEGMENTS + 4 {
            store.append(&payload).unwrap();
        }
        store.sync().unwrap();
        let files: Vec<_> = fs::read_dir(&dir).unwrap().flatten().collect();
        assert_eq!(files.len(), RUN_SEGMENTS as usize);
        #[cfg(unix)]
        {
            use std::os::unix::fs::PermissionsExt;
            assert_eq!(fs::metadata(&dir).unwrap().permissions().mode() & 0o077, 0);
            assert!(files
                .iter()
                .all(|f| f.metadata().unwrap().permissions().mode() & 0o077 == 0));
        }
        assert!(files
            .iter()
            .all(|f| f.metadata().unwrap().len() <= SEGMENT_BYTES));
        let current = dir.join(format!("nr-rotation-{:06}.jsonl", store.segment));
        let f = OpenOptions::new().write(true).open(&current).unwrap();
        assert!(
            f.try_lock().is_err(),
            "active segment must be protected from other cleaners"
        );
        drop(store);
        fs::remove_dir_all(dir).unwrap();
    }
    #[test]
    fn expired_evidence_is_removed_but_unrelated_files_survive() {
        let dir = directory();
        fs::create_dir_all(&dir).unwrap();
        let old = dir.join("nr-expired-000000.jsonl");
        let file = File::create(&old).unwrap();
        file.set_times(
            fs::FileTimes::new()
                .set_modified(SystemTime::now() - RETENTION - Duration::from_secs(1)),
        )
        .unwrap();
        fs::write(dir.join("keep.txt"), b"keep").unwrap();
        drop(file);
        let mut store = Store::new(dir.clone(), "new".into(), json!({}));
        store.append(b"{}").unwrap();
        assert!(!old.exists());
        assert!(dir.join("keep.txt").exists());
        drop(store);
        fs::remove_dir_all(dir).unwrap();
    }
    #[test]
    fn unavailable_sink_does_not_block_producer_and_recovery_counts_loss() {
        let dir = directory();
        fs::write(&dir, b"blocks directory").unwrap();
        let (tx, rx) = mpsc::sync_channel(256);
        let health = Arc::new(Health::default());
        let h = health.clone();
        let d = dir.clone();
        let writer =
            std::thread::spawn(move || writer_loop(d, "recovery".into(), json!({}), rx, h));
        tx.send(Message::Event(json!({"event":"before"}))).unwrap();
        let (a, b) = mpsc::channel();
        tx.send(Message::Flush(a)).unwrap();
        let _ = b.recv_timeout(Duration::from_secs(1));
        assert!(health.io_errors.load(Relaxed) > 0);
        fs::remove_file(&dir).unwrap();
        std::thread::sleep(Duration::from_millis(1100));
        tx.send(Message::Event(json!({"event":"after"}))).unwrap();
        let (a, b) = mpsc::channel();
        tx.send(Message::Stop(a)).unwrap();
        b.recv_timeout(Duration::from_secs(2)).unwrap();
        writer.join().unwrap();
        let path = fs::read_dir(&dir).unwrap().next().unwrap().unwrap().path();
        let data = fs::read_to_string(path).unwrap();
        assert!(data.contains("after"));
        let event: Value = serde_json::from_str(data.lines().last().unwrap()).unwrap();
        assert!(event["io_errors_total"].as_u64().unwrap() >= 1);
        assert_eq!(event["dropped_total"], 1);
        fs::remove_dir_all(dir).unwrap();
    }
    #[test]
    fn full_queue_and_oversized_records_are_counted() {
        let (sender, _rx) = mpsc::sync_channel(1);
        let r = Recorder {
            sender,
            health: Arc::new(Health::default()),
            sequence: AtomicU64::new(0),
            start: Instant::now(),
            run: "test".into(),
        };
        r.emit("first", 1, json!({}));
        r.emit("second", 1, json!({}));
        assert_eq!(r.health.dropped.load(Relaxed), 1);
        let dir = directory();
        let (tx, rx) = mpsc::sync_channel(4);
        let h = Arc::new(Health::default());
        tx.send(Message::Event(json!({"large":"x".repeat(MAX_EVENT_BYTES)})))
            .unwrap();
        tx.send(Message::Event(json!({"small":true}))).unwrap();
        drop(tx);
        writer_loop(dir.clone(), "bounded".into(), json!({}), rx, h.clone());
        assert_eq!(h.dropped.load(Relaxed), 1);
        assert_eq!(h.written.load(Relaxed), 1);
        fs::remove_dir_all(dir).unwrap();
    }
}
