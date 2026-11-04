//! `SdmcExt` extension trait, following the HAL's `*_ext.rs` convention.

#[cfg(feature = "interrupts")]
use super::SdmcIrq;
use super::blocking::BlockingSdmc;
#[cfg(feature = "interrupts")]
use super::non_blocking::{AsyncSdmc, AsyncSdmcHandler};
use super::{Sdmc, SdmcConfig, SdmcError, SdmcPads};
use embedded_hal::delay::DelayNs;

use crate::cmu::{ClockTree, Cmu};
#[cfg(feature = "interrupts")]
use crate::interrupt::typelevel;

/// Creates SDMC drivers from an instance and its pads.
///
/// Both constructors identify the card, so the returned driver is ready for
/// block transfers. The async constructor is `async` because identification
/// waits for controller interrupts.
#[allow(async_fn_in_trait)] // embedded callers are single-threaded; no `Send` bound needed
pub trait SdmcExt<const I: u8> {
    /// Create a blocking SDMC driver and identify the card.
    fn new_blocking<PADS, D>(
        self,
        pads: PADS,
        config: SdmcConfig,
        delay: D,
        clocks: &ClockTree,
        cmu: &mut Cmu,
    ) -> Result<BlockingSdmc<I, PADS, D>, SdmcError>
    where
        PADS: SdmcPads<I>,
        D: DelayNs;

    /// Create an interrupt-driven SDMC driver and identify the card.
    ///
    /// `IRQS` installs [`AsyncSdmcHandler`] for the controller's interrupt and
    /// enables the source before identification starts.
    #[cfg(feature = "interrupts")]
    async fn new_async<PADS, D, IRQS>(
        self,
        pads: PADS,
        config: SdmcConfig,
        delay: D,
        clocks: &ClockTree,
        cmu: &mut Cmu,
        irqs: IRQS,
    ) -> Result<AsyncSdmc<I, PADS, D>, SdmcError>
    where
        PADS: SdmcPads<I>,
        D: DelayNs,
        Sdmc<I>: SdmcIrq<I>,
        AsyncSdmcHandler<I>: typelevel::Handler<<Sdmc<I> as SdmcIrq<I>>::Irq>,
        IRQS: typelevel::Binding<<Sdmc<I> as SdmcIrq<I>>::Irq, AsyncSdmcHandler<I>>;
}

impl<const I: u8> SdmcExt<I> for Sdmc<I> {
    fn new_blocking<PADS, D>(
        self,
        pads: PADS,
        config: SdmcConfig,
        delay: D,
        clocks: &ClockTree,
        cmu: &mut Cmu,
    ) -> Result<BlockingSdmc<I, PADS, D>, SdmcError>
    where
        PADS: SdmcPads<I>,
        D: DelayNs,
    {
        BlockingSdmc::new(self.register_block(), pads, config, delay, clocks, cmu)
    }

    #[cfg(feature = "interrupts")]
    async fn new_async<PADS, D, IRQS>(
        self,
        pads: PADS,
        config: SdmcConfig,
        delay: D,
        clocks: &ClockTree,
        cmu: &mut Cmu,
        _irqs: IRQS,
    ) -> Result<AsyncSdmc<I, PADS, D>, SdmcError>
    where
        PADS: SdmcPads<I>,
        D: DelayNs,
        Sdmc<I>: SdmcIrq<I>,
        AsyncSdmcHandler<I>: typelevel::Handler<<Sdmc<I> as SdmcIrq<I>>::Irq>,
        IRQS: typelevel::Binding<<Sdmc<I> as SdmcIrq<I>>::Irq, AsyncSdmcHandler<I>>,
    {
        use crate::interrupt::typelevel::Interrupt as _;

        <Sdmc<I> as SdmcIrq<I>>::Irq::disable();
        IRQS::bind();
        <Sdmc<I> as SdmcIrq<I>>::Irq::clear_pending();
        <Sdmc<I> as SdmcIrq<I>>::Irq::enable();
        match AsyncSdmc::new(self.register_block(), pads, config, delay, clocks, cmu).await {
            Ok(driver) => Ok(driver),
            Err(error) => {
                // Identification failed, so no handler is needed: park the
                // interrupt source again instead of leaving it enabled.
                <Sdmc<I> as SdmcIrq<I>>::Irq::disable();
                Err(error)
            }
        }
    }
}
