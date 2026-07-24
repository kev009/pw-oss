use std::env;
use std::process::{Command, Stdio};

const FREEBSD_15_2_OSREL: u32 = 1_502_000;
const FREEBSD_16_BASE_OSREL: u32 = 1_600_000;
const SCALED_SECONDARY_BUFFER_16_OSREL: u32 = 1_600_020;

fn main() {
    println!("cargo::rerun-if-changed=abi/compare.c");
    println!("cargo::rustc-check-cfg=cfg(freebsd_scaled_secondary_buffer)");

    // The probe must use the headers belonging to the target whose ABI is
    // being checked. Current CI builds this plugin natively in a FreeBSD VM;
    // a future cross build must supply a target C compiler and sysroot.
    if env::var("CARGO_CFG_TARGET_OS").as_deref() != Ok("freebsd") {
        return;
    }

    let mut build = cc::Build::new();
    build
        .file("abi/compare.c")
        .std("c11")
        .warnings(true)
        .extra_warnings(true)
        .flag_if_supported("-Werror")
        .flag_if_supported("-fvisibility=hidden");

    let osrel = target_freebsd_osrel(build.get_compiler().to_command());
    if scaled_secondary_buffer_available(osrel) {
        println!("cargo::rustc-cfg=freebsd_scaled_secondary_buffer");
    }

    build.compile("spa_freebsd_oss_abi_compare");
}

fn target_freebsd_osrel(mut compiler: Command) -> u32 {
    let output = compiler
        .args(["-dM", "-E", "-include", "sys/param.h", "-x", "c", "-"])
        .stdin(Stdio::null())
        .output()
        .unwrap_or_else(|error| panic!("failed to run the target C preprocessor: {error}"));
    assert!(
        output.status.success(),
        "target C preprocessor could not read <sys/param.h>:\n{}",
        String::from_utf8_lossy(&output.stderr)
    );

    String::from_utf8_lossy(&output.stdout)
        .lines()
        .find_map(|line| {
            let mut fields = line.split_whitespace();
            match (fields.next(), fields.next(), fields.next(), fields.next()) {
                (Some("#define"), Some("__FreeBSD_version"), Some(value), None) => {
                    value.parse().ok()
                }
                _ => None,
            }
        })
        .expect("target <sys/param.h> did not define a numeric __FreeBSD_version")
}

fn scaled_secondary_buffer_available(osrel: u32) -> bool {
    // The kernel change landed during 1600019 without an osreldate bump.
    // Package builds therefore use the first releases that unambiguously
    // contain it, matching the branch-aware policy used by event.rs.
    (FREEBSD_15_2_OSREL..FREEBSD_16_BASE_OSREL).contains(&osrel)
        || osrel >= SCALED_SECONDARY_BUFFER_16_OSREL
}
