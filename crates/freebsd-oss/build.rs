use std::env;

fn main() {
    println!("cargo::rerun-if-changed=abi/compare.c");

    // The probe must use the headers belonging to the target whose ABI is
    // being checked. Current CI builds this plugin natively in a FreeBSD VM;
    // a future cross build must supply a target C compiler and sysroot.
    if env::var("CARGO_CFG_TARGET_OS").as_deref() != Ok("freebsd") {
        return;
    }

    cc::Build::new()
        .file("abi/compare.c")
        .std("c11")
        .warnings(true)
        .extra_warnings(true)
        .flag_if_supported("-Werror")
        .flag_if_supported("-fvisibility=hidden")
        .compile("spa_freebsd_oss_abi_compare");
}
