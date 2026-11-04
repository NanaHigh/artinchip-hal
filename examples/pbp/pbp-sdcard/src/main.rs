//! PBP example: identify the TF card, mount the `hadris` embedded FAT driver and
//! benchmark file write/read.
//!
//! Two mutually exclusive features share `common.rs`: `async-sd` (each phase in
//! its own `block_on`) and `sync-sd` (blocking, split into `#[inline(never)]`
//! phases). No caller-provided driver buffer is used, and the image must fit the
//! 128 KiB boot partition (hence the size-first `pbp-release` profile).
//! Build/flash commands are in `README.md`.

#![no_std]
#![no_main]

use artinchip_rt::{Peripherals, default_clk_tree, pbp_entry, prelude::*};
use log::{error, info};

#[cfg(feature = "async-sd")]
mod async_sd;
mod common;
#[cfg(feature = "sync-sd")]
mod sync_sd;

#[cfg(all(feature = "async-sd", feature = "sync-sd"))]
compile_error!("choose either async-sd or sync-sd");
#[cfg(not(any(feature = "async-sd", feature = "sync-sd")))]
compile_error!("enable async-sd or sync-sd");
#[cfg(all(feature = "d13x", feature = "d21x"))]
compile_error!("choose either d13x or d21x");
#[cfg(all(feature = "d21x", feature = "async-sd"))]
compile_error!("d21x PBP SRAM is only 96 KiB: the async SD/FAT image does not fit, use `sync-sd`");
#[cfg(not(any(feature = "d13x", feature = "d21x")))]
compile_error!("enable d13x or d21x");

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    log::error!("PANIC");
    loop {
        core::hint::spin_loop();
    }
}

/// Keep the large startup frame out of `pbp_main`.
#[inline(never)]
fn startup() -> (Peripherals, artinchip_hal::cmu::ClockTree) {
    (Peripherals::take(), default_clk_tree())
}

#[pbp_entry]
fn pbp_main(boot_param: BootParam, _private_data: &[u8]) {
    check_startup(&boot_param);

    let (p, clocks) = startup();
    #[cfg(feature = "async-sd")]
    let result = async_sd::run(p, clocks);
    #[cfg(feature = "sync-sd")]
    let result = sync_sd::run(p, clocks);
    if let Err(err) = result {
        error!("sdcard phase failed: {err:?}");
    }
    info!("Done");

    loop {
        core::hint::spin_loop();
    }
}
