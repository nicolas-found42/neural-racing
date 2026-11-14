//! Cached, asynchronously rasterised original ground tiles.
//!
//! The original ground overlaps itself in height, so whole-face painter order
//! cannot be correct. Core rasterises exact per-pixel depth into world-aligned
//! tiles; this module keeps a bounded tile cache, asks a single background
//! worker for the visible tiles, and places them at their exact world
//! coordinates.

use crate::render::{draw_neutral_source_ground, View};
use egui::{Color32, Context, Painter, Pos2, Rect, TextureHandle, TextureOptions};
use neural_racing_core::{
    Circuit, GroundTileGrid, SourceGroundRasterizer, Track, Vec2, TILE_PIXELS, UNITS_PER_METRE,
};
use neural_racing_diagnostics::{context, emit, json, operation};
use std::collections::{HashMap, HashSet};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{mpsc, Arc, Mutex};
use std::time::{Duration, Instant};

/// Texture memory bound: 128 tiles of 256² RGBA is 32 MiB.
pub const TILE_LIMIT: usize = 128;
/// Requests queued or in flight; the visible set stays below this.
pub const REQUEST_LIMIT: usize = 96;
/// Coarsen the level rather than exceed this many visible tiles.
const MAX_VISIBLE_TILES: usize = 96;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
struct TileKey {
    level: u8,
    x: i32,
    y: i32,
}

struct CompletedTile {
    generation: u64,
    key: TileKey,
    rgba: Vec<u8>,
}

enum Command {
    Course {
        generation: u64,
        operation: u64,
        raster: Option<Arc<SourceGroundRasterizer>>,
    },
    Render {
        generation: u64,
        key: TileKey,
    },
}

struct Worker {
    commands: mpsc::Sender<Command>,
    results: mpsc::Receiver<CompletedTile>,
}

impl Worker {
    fn spawn(wanted: Arc<AtomicU64>, worker: u64) -> Self {
        let (commands, command_rx) = mpsc::channel::<Command>();
        let (result_tx, results) = mpsc::channel::<CompletedTile>();
        let spawned = std::thread::Builder::new()
            .name("gt4-ground-tiles".into())
            .spawn(move || {
                let _context = context(worker);
                emit!("ground_worker_started", worker, json!({}));
                let mut course = None;
                while let Ok(command) = command_rx.recv() {
                    match command {
                        Command::Course {
                            generation,
                            operation,
                            raster,
                        } => {
                            emit!(
                                "ground_course_received",
                                operation,
                                json!({"worker":worker,"generation":generation,
                                "has_raster":raster.is_some()})
                            );
                            course = raster.map(|raster| (generation, operation, raster));
                        }
                        Command::Render { generation, key } => {
                            let Some((current, operation, raster)) = &course else {
                                continue;
                            };
                            if *current != generation
                                || wanted.load(Ordering::Relaxed) != pack(generation, key.level)
                            {
                                continue;
                            }
                            let Some(grid) = GroundTileGrid::new(key.level) else {
                                continue;
                            };
                            let _context = context(*operation);
                            let rgba = raster.render_tile(grid, key.x, key.y);
                            if result_tx
                                .send(CompletedTile {
                                    generation,
                                    key,
                                    rgba,
                                })
                                .is_err()
                            {
                                break;
                            }
                        }
                    }
                }
                emit!("ground_worker_stopped", worker, json!({}));
            });
        if let Err(error) = spawned {
            emit!(
                "ground_worker_spawn_failed",
                worker,
                json!({"error_kind":format!("{:?}",error.kind())})
            );
        }
        Self { commands, results }
    }
}

fn pack(generation: u64, level: u8) -> u64 {
    (generation << 32) | u64::from(level)
}

struct CachedTile {
    texture: TextureHandle,
    used: u64,
}

struct TileCache {
    course: Option<Circuit>,
    generation: u64,
    raster: Option<Arc<SourceGroundRasterizer>>,
    worker: Option<Worker>,
    wanted: Option<Arc<AtomicU64>>,
    tiles: HashMap<TileKey, CachedTile>,
    requested: HashSet<TileKey>,
    error: Option<String>,
    frame: u64,
    operation: u64,
    worker_id: u64,
    disconnected: bool,
    pending_since: Option<Instant>,
    stalled: bool,
    first_ready: bool,
    accepted: u64,
    discarded: u64,
    superseded: u64,
    invalid: u64,
}

impl Default for TileCache {
    fn default() -> Self {
        Self {
            course: None,
            generation: 0,
            raster: None,
            worker: None,
            wanted: None,
            tiles: HashMap::new(),
            requested: HashSet::new(),
            error: None,
            frame: 0,
            operation: 0,
            worker_id: 0,
            disconnected: false,
            pending_since: None,
            stalled: false,
            first_ready: false,
            accepted: 0,
            discarded: 0,
            superseded: 0,
            invalid: 0,
        }
    }
}

fn cache_id() -> egui::Id {
    egui::Id::unique("gt4-source-ground-tiles")
}

fn cache(ctx: &Context) -> Arc<Mutex<TileCache>> {
    ctx.data_mut(|data| {
        data.get_temp_mut_or_default::<Arc<Mutex<TileCache>>>(cache_id())
            .clone()
    })
}

/// Number of visible ground tiles still queued or rasterising.
pub fn pending(ctx: &Context) -> Option<usize> {
    let cache = ctx.data(|data| data.get_temp::<Arc<Mutex<TileCache>>>(cache_id()))?;
    let cache = cache.lock().expect("ground tile cache poisoned");
    Some(cache.requested.len())
}

/// Resident tile and in-flight request counts, for cache-bound diagnostics.
pub fn stats(ctx: &Context) -> Option<(usize, usize)> {
    let cache = ctx.data(|data| data.get_temp::<Arc<Mutex<TileCache>>>(cache_id()))?;
    let cache = cache.lock().expect("ground tile cache poisoned");
    Some((cache.tiles.len(), cache.requested.len()))
}

/// Draw the original textured ground. Returns `false` when the course has no
/// source visual, so the caller falls back to its neutral geometry.
pub fn draw(painter: &Painter, view: &View, track: &Track) -> bool {
    let Some(course) = track.circuit else {
        return false;
    };
    let cache = cache(painter.ctx());
    let mut cache = cache.lock().expect("ground tile cache poisoned");
    cache.frame = cache.frame.wrapping_add(1);
    if cache.course != Some(course) {
        begin_course(track, course, &mut cache);
    }
    if cache.raster.is_none() {
        if let Some(error) = &cache.error {
            painter.text(
                view.pos(track.centerline[0]),
                egui::Align2::CENTER_BOTTOM,
                format!("Source visual unavailable: {error}"),
                egui::FontId::proportional(12.0),
                Color32::LIGHT_RED,
            );
        }
        return false;
    }
    collect_results(painter.ctx(), &mut cache);

    let clip = painter.clip_rect();
    let (low, high) = view.world_bounds(clip);
    let metres = |v: Vec2| (v.x / UNITS_PER_METRE, v.y / UNITS_PER_METRE);
    let (min_x, min_y) = metres(low);
    let (max_x, max_y) = metres(high);

    let wanted = view.points_per_metre() * f64::from(painter.ctx().pixels_per_point());
    let raster = cache.raster.as_deref().expect("raster present");
    let (grid, x0, y0, x1, y1) = visible_tiles(raster, wanted, min_x, min_y, max_x, max_y);
    let ready = current_tiles(&cache, grid, x0, y0, x1, y1);
    if !ready {
        // Progressive fill: the neutral collision ground stays underneath
        // until the exact textured tiles cover the visible area.
        if !draw_neutral_source_ground(painter, view, track) {
            crate::render::draw_practice_ground(painter, view, track);
        }
    }

    draw_cached_tiles(painter, view, &mut cache, clip);
    request_tiles(
        painter.ctx(),
        &mut cache,
        grid,
        x0,
        y0,
        x1,
        y1,
        (min_x + max_x) * 0.5,
        (min_y + max_y) * 0.5,
    );
    evict(&mut cache);
    cache.observe_view(ready, grid.level());
    true
}

/// Tile range covering the canvas, clamped to the source extent, at the
/// coarsest level that keeps the visible tile count inside the cache budget.
fn visible_tiles(
    raster: &SourceGroundRasterizer,
    wanted: f64,
    min_x: f64,
    min_y: f64,
    max_x: f64,
    max_y: f64,
) -> (GroundTileGrid, i32, i32, i32, i32) {
    let mut level = GroundTileGrid::at_least(wanted).level();
    loop {
        let grid = GroundTileGrid::new(level).expect("level within table");
        let margin = grid.metres_per_tile();
        let mut low = (min_x - margin, min_y - margin);
        let mut high = (max_x + margin, max_y + margin);
        if let Some(bounds) = raster.bounds() {
            low.0 = low.0.max(bounds.0 - margin);
            low.1 = low.1.max(bounds.1 - margin);
            high.0 = high.0.min(bounds.2 + margin);
            high.1 = high.1.min(bounds.3 + margin);
        }
        let range = grid.covering(low.0, low.1, high.0, high.1);
        let count = i64::from(range.2 - range.0 + 1) * i64::from(range.3 - range.1 + 1);
        if count <= MAX_VISIBLE_TILES as i64 || level == 0 {
            return (grid, range.0, range.1, range.2, range.3);
        }
        level -= 1;
    }
}

impl TileCache {
    fn observe_view(&mut self, ready: bool, level: u8) {
        if !neural_racing_diagnostics::enabled() {
            return;
        }
        let (event, delay) = if ready {
            let delay = self.pending_since.take().map(|since| since.elapsed());
            if self.first_ready && !self.stalled {
                return;
            }
            self.first_ready = true;
            self.stalled = false;
            ("ground_visible_ready", delay.unwrap_or_default())
        } else {
            let delay = self
                .pending_since
                .get_or_insert_with(Instant::now)
                .elapsed();
            if self.stalled || delay < Duration::from_secs(2) {
                return;
            }
            self.stalled = true;
            ("ground_visible_delayed", delay)
        };
        emit!(
            event,
            self.operation,
            json!({"worker":self.worker_id,"generation":self.generation,
            "level":level,"wait_ms":delay.as_millis() as u64,"disconnected":self.disconnected,
            "pending":self.requested.len(),"resident":self.tiles.len(),"accepted":self.accepted,
            "discarded":self.discarded,"superseded":self.superseded,"invalid":self.invalid})
        );
    }

    fn disconnected(&mut self, stage: &'static str) {
        if !self.disconnected {
            self.disconnected = true;
            emit!(
                "ground_worker_disconnected",
                self.operation,
                json!({"worker":self.worker_id,"stage":stage,"pending":self.requested.len()})
            );
        }
    }
}

fn begin_course(track: &Track, course: Circuit, cache: &mut TileCache) {
    let previous = cache.operation;
    cache.operation = operation();
    emit!(
        "ground_course_requested",
        cache.operation,
        json!({"previous_course":previous,"circuit":course.slug(),
        "generation":cache.generation.wrapping_add(1),"previous_pending":cache.requested.len(),
        "previous_first_ready":cache.first_ready})
    );
    let started = Instant::now();
    cache.pending_since = None;
    cache.first_ready = false;
    cache.stalled = false;
    cache.accepted = 0;
    cache.discarded = 0;
    cache.superseded = 0;
    cache.invalid = 0;
    cache.disconnected = false;
    cache.course = Some(course);
    cache.generation = cache.generation.wrapping_add(1);
    cache.tiles.clear();
    cache.requested.clear();
    cache.error = None;
    cache.raster = None;
    let mut counts = None;
    let status = match track
        .source_visual()
        .and_then(|visual| visual.map(|visual| visual.prepare_for_clamp()).transpose())
    {
        Ok(Some(visual)) => {
            let raster = SourceGroundRasterizer::new(visual);
            counts = Some([
                raster.triangle_count(),
                raster.degenerate_faces(),
                raster.untextured_faces(),
            ]);
            if raster.triangle_count() == 0 {
                cache.error = Some(format!(
                    "no usable ground faces ({} degenerate, {} untextured)",
                    raster.degenerate_faces(),
                    raster.untextured_faces()
                ));
                "no_usable_faces"
            } else {
                cache.raster = Some(Arc::new(raster));
                "prepared"
            }
        }
        Ok(None) => "no_source_visual",
        Err(error) => {
            cache.error = Some(error);
            "decode_or_prepare_failed"
        }
    };
    emit!(
        "ground_course_prepared",
        cache.operation,
        json!({"status":status,
        "faces":counts,"duration_ms":started.elapsed().as_millis() as u64})
    );
    let wanted = cache
        .wanted
        .get_or_insert_with(|| Arc::new(AtomicU64::new(0)))
        .clone();
    if cache.worker.is_none() {
        cache.worker_id = operation();
        cache.worker = Some(Worker::spawn(wanted, cache.worker_id));
    }
    if cache
        .worker
        .as_ref()
        .unwrap()
        .commands
        .send(Command::Course {
            generation: cache.generation,
            operation: cache.operation,
            raster: cache.raster.clone(),
        })
        .is_err()
    {
        cache.disconnected("course_send");
    }
}

fn collect_results(ctx: &Context, cache: &mut TileCache) {
    let Some(worker) = &cache.worker else {
        return;
    };
    loop {
        let done = match worker.results.try_recv() {
            Ok(done) => done,
            Err(mpsc::TryRecvError::Empty) => break,
            Err(mpsc::TryRecvError::Disconnected) => {
                cache.disconnected("result_receive");
                break;
            }
        };
        if done.generation != cache.generation || !cache.requested.remove(&done.key) {
            cache.discarded += 1;
            continue;
        }
        if done.rgba.len() != (TILE_PIXELS * TILE_PIXELS * 4) as usize {
            cache.invalid += 1;
            continue;
        }
        let image = egui::ColorImage::from_rgba_unmultiplied(
            [TILE_PIXELS as usize, TILE_PIXELS as usize],
            &done.rgba,
        );
        let texture = ctx.load_texture(
            format!(
                "gt4-ground-{}-{}-{}-{}",
                cache.generation, done.key.level, done.key.x, done.key.y
            ),
            image,
            TextureOptions::LINEAR,
        );
        cache.accepted += 1;
        cache.tiles.insert(
            done.key,
            CachedTile {
                texture,
                used: cache.frame,
            },
        );
    }
}

fn current_tiles(
    cache: &TileCache,
    grid: GroundTileGrid,
    x0: i32,
    y0: i32,
    x1: i32,
    y1: i32,
) -> bool {
    (y0..=y1).all(|y| {
        (x0..=x1).all(|x| {
            cache.tiles.contains_key(&TileKey {
                level: grid.level(),
                x,
                y,
            })
        })
    })
}

fn draw_cached_tiles(painter: &Painter, view: &View, cache: &mut TileCache, clip: Rect) {
    let mut keys: Vec<TileKey> = cache.tiles.keys().copied().collect();
    // Coarser tiles first, so a finer level covers them while it loads.
    keys.sort_unstable();
    for key in keys {
        let Some(grid) = GroundTileGrid::new(key.level) else {
            continue;
        };
        let metres = grid.metres_per_tile();
        let low = view.pos(Vec2 {
            x: f64::from(key.x) * metres * UNITS_PER_METRE,
            y: f64::from(key.y) * metres * UNITS_PER_METRE,
        });
        let high = view.pos(Vec2 {
            x: f64::from(key.x + 1) * metres * UNITS_PER_METRE,
            y: f64::from(key.y + 1) * metres * UNITS_PER_METRE,
        });
        let rect = Rect::from_min_max(low, high);
        if !rect.intersects(clip) {
            continue;
        }
        let Some(tile) = cache.tiles.get_mut(&key) else {
            continue;
        };
        tile.used = cache.frame;
        painter.image(
            tile.texture.id(),
            rect,
            Rect::from_min_max(Pos2::ZERO, Pos2::new(1.0, 1.0)),
            Color32::WHITE,
        );
    }
}

fn request_tiles(
    ctx: &Context,
    cache: &mut TileCache,
    grid: GroundTileGrid,
    x0: i32,
    y0: i32,
    x1: i32,
    y1: i32,
    center_x: f64,
    center_y: f64,
) {
    let Some(commands) = cache.worker.as_ref().map(|worker| worker.commands.clone()) else {
        return;
    };
    if let Some(wanted) = &cache.wanted {
        wanted.store(pack(cache.generation, grid.level()), Ordering::Relaxed);
    }
    let metres = grid.metres_per_tile();
    let level = grid.level();
    // A request for a level the camera has left is skipped by the worker and
    // never answered. Drop it so it cannot occupy the request budget forever.
    let before = cache.requested.len();
    cache.requested.retain(|key| key.level == level);
    cache.superseded += (before - cache.requested.len()) as u64;
    let mut missing: Vec<(f64, TileKey)> = Vec::new();
    for y in y0..=y1 {
        for x in x0..=x1 {
            let key = TileKey { level, x, y };
            if cache.tiles.contains_key(&key) || cache.requested.contains(&key) {
                continue;
            }
            let dx = (f64::from(x) + 0.5) * metres - center_x;
            let dy = (f64::from(y) + 0.5) * metres - center_y;
            missing.push((dx * dx + dy * dy, key));
        }
    }
    missing.sort_unstable_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)));
    for (_, key) in missing {
        if cache.requested.len() >= REQUEST_LIMIT {
            break;
        }
        if commands
            .send(Command::Render {
                generation: cache.generation,
                key,
            })
            .is_ok()
        {
            cache.requested.insert(key);
        } else {
            cache.disconnected("tile_send");
            break;
        }
    }
    if !cache.requested.is_empty() {
        ctx.request_repaint();
    }
}

fn evict(cache: &mut TileCache) {
    while cache.tiles.len() > TILE_LIMIT {
        let Some(key) = cache
            .tiles
            .iter()
            .filter(|(key, tile)| !cache.requested.contains(key) && tile.used < cache.frame)
            .min_by_key(|(_, tile)| tile.used)
            .map(|(key, _)| *key)
        else {
            break;
        };
        cache.tiles.remove(&key);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    #[ignore = "uses a process-global recorder; run this fixture in isolation"]
    fn disconnected_worker_records_pending_view_once() {
        let dir =
            std::env::temp_dir().join(format!("neural-racing-tiles-loss-{}", std::process::id()));
        // Only initialize the process-global recorder in an isolated invocation.
        let retained = std::env::var_os("NEURAL_RACING_TILE_LOSS_TEST_DIR");
        let dir = retained.clone().map_or(dir, Into::into);
        std::env::set_var("NEURAL_RACING_DIAGNOSTICS_DIR", &dir);
        let diagnostics = neural_racing_diagnostics::init("ground_disconnect_test");
        let (commands, command_rx) = mpsc::channel();
        let (result_tx, results) = mpsc::channel();
        drop(command_rx);
        drop(result_tx);
        let id = operation();
        let mut cache = TileCache {
            worker: Some(Worker { commands, results }),
            operation: id,
            worker_id: operation(),
            pending_since: Some(Instant::now() - Duration::from_secs(3)),
            ..Default::default()
        };
        cache.requested.insert(TileKey {
            level: 0,
            x: 0,
            y: 0,
        });
        let ctx = Context::default();
        for _ in 0..10 {
            collect_results(&ctx, &mut cache);
            cache.observe_view(false, 0);
        }
        assert_eq!(cache.requested.len(), 1);
        drop(diagnostics);

        let output = std::process::Command::new("python3")
            .arg(concat!(
                env!("CARGO_MANIFEST_DIR"),
                "/../../scripts/diagnostics.py"
            ))
            .arg("--dir")
            .arg(&dir)
            .args(["--operation", &id.to_string()])
            .output()
            .unwrap();
        assert!(
            output.status.success(),
            "{}",
            String::from_utf8_lossy(&output.stderr)
        );
        let rows: Vec<neural_racing_diagnostics::Value> = std::str::from_utf8(&output.stdout)
            .unwrap()
            .lines()
            .map(|line| line.parse().unwrap())
            .collect();
        assert_eq!(rows.len(), 2);
        assert_eq!(rows[0]["event"], "ground_worker_disconnected");
        assert_eq!(rows[0]["data"]["stage"], "result_receive");
        assert_eq!(rows[1]["event"], "ground_visible_delayed");
        assert_eq!(rows[1]["data"]["disconnected"], true);
        assert_eq!(rows[1]["data"]["pending"], 1);
        if retained.is_none() {
            std::fs::remove_dir_all(dir).unwrap();
        }
    }
}
