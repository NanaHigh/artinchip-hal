//! SDMC clock setup: module clock (CMU + reset), card-clock divider selection,
//! programming through the `UPD_CLK` pseudo-command, and the clock ceiling
//! helpers shared by both drivers.

use embedded_time::rate::Hertz;

use super::config::SdmcConfig;
use super::params::MAX_CLOCK_HZ;
use super::register::{CommandConfig, ExtClkMux, RegisterBlock};
use crate::cmu::{ClockTree, Cmu};

/// SDMC module clock target: eight times the card clock, so the divider chain
/// can reach the 50 MHz maximum with `sclk = 200 MHz` and `CCLK = sclk / 4`.
const MODULE_CLOCK_HZ: u32 = MAX_CLOCK_HZ * 8;

/// Bounded spins while the controller applies a clock update command.
const CLOCK_UPDATE_SPINS: u32 = 100_000;

/// Enable the SDMC module clock and release its reset.
///
/// The module clock is derived from the real `PLL_FRA0` read back from the CMU
/// (a bootloader may program a different value than the static clock tree), and
/// sized for the requested card clock: the controller divides it by two before
/// the card-clock divider, so at least four times the card clock is needed.
///
/// Returns the SDMC module clock frequency.
pub(crate) fn configure_module_clock<const I: u8>(
    clocks: &ClockTree,
    cmu: &mut Cmu,
    card_clock: Hertz,
) -> u32 {
    const { assert!(I <= 2, "invalid SDMC instance") };
    // Read the PLL from the CMU: the bootloader (application case) may have
    // programmed a different PLL_FRA0 than the value baked into `ClockTree`.
    let parent = clocks.pll_fra0_hz(cmu).0;
    let needed = MODULE_CLOCK_HZ.max(card_clock.0.saturating_mul(4));
    let divisor = (parent / needed).clamp(1, 32);
    let reg = cmu.register_block();
    let clock = match I {
        0 => &reg.clock_sdmc0,
        1 => &reg.clock_sdmc1,
        _ => &reg.clock_sdmc2,
    };

    unsafe {
        clock.modify(|v| {
            v.set_module_clk_div((divisor - 1) as u8)
                .enable_bus_clk()
                .enable_module_clk()
        });
        clock.modify(|v| v.enable_module_reset());
    }
    for _ in 0..1_000 {
        core::hint::spin_loop();
    }
    unsafe {
        clock.modify(|v| v.disable_module_reset());
    }

    parent / divisor
}

/// Highest card clock the module clock can generate.
///
/// The divider chain needs at least two module-clock cycles per card clock,
/// and the controller input is the module clock divided by two.
pub(crate) const fn max_card_clock(module_hz: u32) -> Hertz {
    Hertz(module_hz / 4)
}

/// Clamp a requested card clock to the driver ceiling and the module clock.
pub(crate) const fn clamp_card_clock(requested: Hertz, available: Hertz) -> Hertz {
    let requested = if requested.0 < MAX_CLOCK_HZ {
        requested.0
    } else {
        MAX_CLOCK_HZ
    };
    if requested < available.0 {
        Hertz(requested)
    } else {
        Hertz(available.0)
    }
}

/// Pick the card clock divider: `(CCLK, EXT_CLK_MUX, clk_div)`.
///
/// The chain is `CCLK = sclk / (mux * 2 * clk_div)` with `mux in {1, 2, 4}`.
/// Both candidate dividers around the request are evaluated and the fastest
/// frequency within 0.1% of it wins, so PLL rounding cannot quantize the card
/// clock to a much lower value (D13x: 50,000,002 Hz for a 50 MHz request).
pub(crate) fn select_clock(sclk: u32, target: u32) -> (u32, u32, u8) {
    let target = target.max(1);
    let tolerance = (target / 1000).max(1);
    let mut best = (0u32, 1u32, 1u8);
    for mux in [1u32, 2, 4] {
        let base = sclk / (mux * 2);
        let ceil_div = base.div_ceil(target).clamp(1, u8::MAX as u32);
        for div in [ceil_div.saturating_sub(1).max(1), ceil_div] {
            let frequency = sclk / (mux * 2 * div);
            if frequency <= target.saturating_add(tolerance) && frequency > best.0 {
                best = (frequency, mux, div as u8);
            }
        }
    }
    if best.0 == 0 {
        // The request is below the lowest achievable frequency.
        best = (sclk / (4 * 2 * 255), 4, 255);
    }
    best
}

/// Program the card clock and return the frequency the controller produces.
///
/// `clock` is the frequency requested for `CCLK`; the divider chain is chosen
/// by [`select_clock`]. The clock is disabled while the divider changes and the
/// new setting is applied through the `UPD_CLK` pseudo-command.
pub(crate) fn set_card_clock(
    reg: &'static RegisterBlock,
    config: &SdmcConfig,
    module_hz: u32,
    clock: Hertz,
) -> Hertz {
    let (actual, mux, clk_div) = select_clock(module_hz / 2, clock.0.max(1));
    let ext_mux = match mux {
        1 => ExtClkMux::ClkInDiv1,
        2 => ExtClkMux::ClkInDiv2,
        _ => ExtClkMux::ClkInDiv4,
    };

    // SAFETY: the register block is a valid MMIO region.
    unsafe {
        reg.card_delay_chain_phase_ctrl.modify(|v| {
            v.set_ext_clk_mux(ext_mux)
                .set_drv_clk_phase(config.drv_phase)
                .set_samp_clk_phase(config.smp_phase)
        });
        // Disable the card clock before changing the divider.
        reg.clk_ctrl.modify(|v| v.disable_clk());
        reg.clk_ctrl.modify(|v| v.set_clk_div(clk_div));
    }
    update_clock(reg);
    // SAFETY: the register block is a valid MMIO region.
    unsafe {
        reg.clk_ctrl.modify(|v| v.enable_clk().enable_clk_lp());
    }
    update_clock(reg);

    Hertz(actual)
}

/// Apply the clock divider through the `UPD_CLK` pseudo-command.
fn update_clock(reg: &'static RegisterBlock) {
    let config = CommandConfig::zeroed()
        .enable_wait_prvdata_done()
        .enable_update_clk_reg()
        .set_start_cmd(true);
    // SAFETY: the register block is a valid MMIO region.
    unsafe {
        reg.cmd_config.write(config);
    }
    for _ in 0..CLOCK_UPDATE_SPINS {
        if !reg.cmd_config.read().start_cmd() {
            return;
        }
        core::hint::spin_loop();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn divider_hits_target_when_dividable() {
        // CCLK = sclk / (mux * 2 * clk_div).
        assert_eq!(select_clock(100_000_000, 25_000_000), (25_000_000, 1, 2));
        assert_eq!(select_clock(100_000_000, 50_000_000), (50_000_000, 1, 1));
        assert_eq!(select_clock(100_000_000, 400_000), (400_000, 1, 125));
    }

    #[test]
    fn divider_handles_pll_rounding() {
        // D13x PLL_FRA0 = 800_000_030 Hz -> sclk = 200_000_007 Hz; the 50 MHz
        // candidate is 50_000_001 Hz and must not quantize to 33.33 MHz.
        assert_eq!(select_clock(200_000_007, 50_000_000), (50_000_001, 1, 2));
        // The next mux step down (100 MHz / 4 = 25 MHz) is far above a
        // 33.33 MHz request, so only the 0.1% tolerance is allowed.
        assert_eq!(select_clock(100_000_000, 33_000_000), (25_000_000, 1, 2));
    }

    #[test]
    fn divider_never_far_above_request() {
        let (frequency, _, _) = select_clock(200_000_007, 25_000_000);
        assert!(frequency <= 25_025_000);
        let (frequency, _, _) = select_clock(200_000_007, 400_000);
        assert!(frequency <= 400_400 && frequency > 0);
    }

    #[test]
    fn clamp_uses_ceiling_and_module_limit() {
        assert_eq!(
            clamp_card_clock(Hertz(50_000_000u32), Hertz(100_000_000u32)),
            Hertz(50_000_000u32)
        );
        assert_eq!(
            clamp_card_clock(Hertz(80_000_000u32), Hertz(100_000_000u32)),
            Hertz(50_000_000u32)
        );
        assert_eq!(
            clamp_card_clock(Hertz(50_000_000u32), Hertz(25_000_000u32)),
            Hertz(25_000_000u32)
        );
    }

    #[test]
    fn module_clock_limits_the_card_clock() {
        assert_eq!(max_card_clock(400_000_000), Hertz(100_000_000u32));
    }
}
