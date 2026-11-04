use std::{fs, path::Path, process::Command};
fn sources(dir: &Path, hash: &mut u64) {
    println!("cargo:rerun-if-changed={}", dir.display());
    let mut paths: Vec<_> = fs::read_dir(dir)
        .unwrap()
        .flatten()
        .map(|e| e.path())
        .collect();
    paths.sort();
    for path in paths {
        if path.is_dir() {
            sources(&path, hash);
        } else if path.extension().is_some_and(|s| s == "rs")
            || path.file_name().is_some_and(|s| s == "Cargo.toml")
        {
            println!("cargo:rerun-if-changed={}", path.display());
            for byte in fs::read(path).unwrap() {
                *hash = (*hash ^ u64::from(byte)).wrapping_mul(0x100000001b3);
            }
        }
    }
}
fn main() {
    let root = Path::new("../..");
    let mut hash = 0xcbf29ce484222325;
    sources(&root.join("crates"), &mut hash);
    for name in ["Cargo.toml", "Cargo.lock"] {
        println!("cargo:rerun-if-changed=../../{name}");
        for byte in fs::read(root.join(name)).unwrap_or_default() {
            hash = (hash ^ u64::from(byte)).wrapping_mul(0x100000001b3);
        }
    }
    let revision = Command::new("git")
        .args(["rev-parse", "HEAD"])
        .current_dir(root)
        .output()
        .ok()
        .filter(|r| r.status.success())
        .map(|r| String::from_utf8_lossy(&r.stdout).trim().to_owned())
        .unwrap_or_else(|| "unknown".into());
    if let Ok(out) = Command::new("git")
        .args(["rev-parse", "--git-path", "HEAD"])
        .current_dir(root)
        .output()
    {
        let path = String::from_utf8_lossy(&out.stdout).trim().to_owned();
        println!("cargo:rerun-if-changed={}", root.join(path).display());
    }
    if let Ok(out) = Command::new("git")
        .args(["symbolic-ref", "-q", "HEAD"])
        .current_dir(root)
        .output()
    {
        let reference = String::from_utf8_lossy(&out.stdout).trim().to_owned();
        if !reference.is_empty() {
            if let Ok(path) = Command::new("git")
                .args(["rev-parse", "--git-path", &reference])
                .current_dir(root)
                .output()
            {
                println!(
                    "cargo:rerun-if-changed={}",
                    root.join(String::from_utf8_lossy(&path.stdout).trim())
                        .display()
                );
            }
        }
    }
    println!("cargo:rustc-env=DIAG_REVISION={revision}");
    println!("cargo:rustc-env=DIAG_SOURCE={hash:016x}");
    println!(
        "cargo:rustc-env=DIAG_PROFILE={}",
        std::env::var("PROFILE").unwrap_or_default()
    );
}
