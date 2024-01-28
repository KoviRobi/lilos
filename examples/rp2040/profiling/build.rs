//! This build script copies the `memory.x` file from the crate root into
//! a directory where the linker can always find it at build time.
//! For many projects this is optional, as the linker always searches the
//! project root directory -- wherever `Cargo.toml` is. However, if you
//! are using a workspace or have a more complicated build setup, this
//! build script becomes required. Additionally, by requesting that
//! Cargo re-run the build script whenever `memory.x` is changed,
//! updating `memory.x` ensures a rebuild of the application with the
//! new memory settings.
//!
//! It also copies the ram-link.x if building a copy-to-ram executable.

use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

fn main() {
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());

    if env::var_os("CARGO_FEATURE_FLASH").is_none() {
        File::create(out.join("memory.x"))
            .unwrap()
            .write_all(include_bytes!("ram-memory.x"))
            .unwrap();
        println!("cargo:rerun-if-changed=ram-memory.x");

        File::create(out.join("link.x"))
            .unwrap()
            .write_all(include_bytes!("ram-link.x"))
            .unwrap();
        println!("cargo:rerun-if-changed=ram-link.x");
    } else {
        File::create(out.join("memory.x"))
            .unwrap()
            .write_all(include_bytes!("flash-memory.x"))
            .unwrap();
        println!("cargo:rerun-if-changed=flash-memory.x");
        // link.x included from cortex-m-rt
    }

    println!("cargo:rustc-link-search={}", out.display());
}
