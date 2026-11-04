//! SD card file read/write benchmark.
//!
//! Booted by `app-bootloader` (DDR on D21x, PSRAM on D13x). The driver path is
//! chosen at build time: interrupt-driven by default, blocking with `sync-sd`.

#![no_std]
#![no_main]

#[cfg(not(feature = "sync-sd"))]
mod async_sd;
mod common;
#[cfg(feature = "sync-sd")]
mod sync_sd;

use artinchip_hal::cmu::ClockTree;
use artinchip_hal::gtc::CntFreq;
use artinchip_hal::prelude::*;
use artinchip_hal::uart::{UartConfig, uart_logger_init};
use artinchip_rt::embassy_time::Timer;
use artinchip_rt::{Peripherals, app_entry, default_clk_tree};
use log::{error, info};
use panic_halt as _;

#[inline(never)]
fn startup() -> (Peripherals, ClockTree) {
    (Peripherals::take(), default_clk_tree())
}

#[app_entry]
async fn app_main() {
    let (mut p, clocks) = startup();

    let tx = p.gpioa.pa0.into_uart0_tx();
    let rx = p.gpioa.pa1.into_uart0_rx();
    let _uart = uart_logger_init(p.uart0, tx, rx, UartConfig::default(), &clocks, &mut p.cmu).ok();
    info!("Welcome to app-sdcard by artinchip-hal🦀!");

    // SDMC1 pin mux (function 2) on both supported boards.
    let pads = (
        p.gpioc.pc2.into_sdmc1_clk(),
        p.gpioc.pc3.into_sdmc1_cmd(),
        p.gpioc.pc1.into_sdmc1_dat0(),
        p.gpioc.pc0.into_sdmc1_dat1(),
        p.gpioc.pc5.into_sdmc1_dat2(),
        p.gpioc.pc4.into_sdmc1_dat3(),
    );
    let _det = p.gpioc.pc6.into_sdmc1_det();
    let delay = p.gtc.new_timer_delay(CntFreq::Freq4M, &mut p.cmu);

    let pattern = common::PATTERN_BUFFER.take();
    let buffer = common::FILE_BUFFER.take();
    common::fill_pattern(pattern.as_mut_slice());

    #[cfg(feature = "sync-sd")]
    let result = sync_sd::run(
        p.sdmc1,
        pads,
        delay,
        &clocks,
        &mut p.cmu,
        pattern.as_slice(),
        buffer.as_mut_slice(),
    );
    #[cfg(not(feature = "sync-sd"))]
    let result = async_sd::run(
        p.sdmc1,
        pads,
        delay,
        &clocks,
        &mut p.cmu,
        pattern.as_slice(),
        buffer.as_mut_slice(),
    )
    .await;

    if let Err(err) = result {
        error!("sd: benchmark failed: {err}");
    }
    info!("Done");

    loop {
        Timer::after_secs(60).await;
    }
}
