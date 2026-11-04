//! Interrupt-driven SDMC driver.
//!
//! [`AsyncSdmc`] wraps the same [`Card`] engine and [`SdmcBus`] sequence as the
//! blocking driver; completions are awaited through the interrupt handler's waker
//! while DAT0/FIFO waits stay bounded spins. Data phases always use the internal
//! DMA, so buffers must be cache-line aligned block multiples (the raw API stages
//! unaligned callers).
//!
//! The handler samples `OINTST` before W1C, merges it into `pending`, clears the
//! interrupt source and wakes the task; the task registers the waker before
//! sampling `pending | OINTST`, so a completion racing registration is not lost.
//! The IDMA status is not cleared in interrupt context (fatal errors must stay
//! visible) and `SdmcBus::begin` clears it. There is no software timeout: the
//! controller timeouts end a dead-card await, and `CMD12` is the only phase with a
//! bounded best-effort spin.

use core::future::poll_fn;
use core::marker::PhantomData;
use core::sync::atomic::{AtomicU32, Ordering};
use core::task::Poll;

use embassy_sync::waitqueue::AtomicWaker;
use embedded_hal::delay::DelayNs;
use embedded_time::rate::Hertz;

#[cfg(feature = "hadris-async")]
use super::WRITEBACK_SLOTS;
use super::blocking::apply_configured_speed;
use super::bus::{SdmcBus, is_dma_aligned};
use super::card::{Card, Command, Data, Response, ResponseKind, SdHostAsync};
use super::register::{BusWidth, RegisterBlock};
use super::{BLOCK_SIZE, CardInfo, DataPath, Sdmc, SdmcConfig, SdmcError, SdmcIrq, SdmcPads};
use crate::cmu::{ClockTree, Cmu};
use crate::types::DmaBuffer;

use crate::interrupt::typelevel::{Handler, Interrupt as _};

/// Interrupt bookkeeping shared by the handler and the await loops.
pub(super) struct IrqState {
    /// Task waiting for the next controller interrupt.
    pub(super) waker: AtomicWaker,
    /// Raw `OINTST` bits captured by the handler and not consumed yet.
    pending: AtomicU32,
}

impl IrqState {
    const fn new() -> Self {
        Self {
            waker: AtomicWaker::new(),
            pending: AtomicU32::new(0),
        }
    }
}

/// Interrupt state of SDMC0 and SDMC1, indexed by instance number.
static IRQ_STATES: [IrqState; 2] = [IrqState::new(), IrqState::new()];

/// Interrupt state of controller `I`.
///
/// The instance number is validated at compile time, so an invalid binding can
/// never turn into a bounds panic inside interrupt context.
#[inline]
pub(super) fn irq_state<const I: u8>() -> &'static IrqState {
    const { assert!((I as usize) < IRQ_STATES.len()) };
    &IRQ_STATES[I as usize]
}

/// Consume the interrupt bits captured by the handler since the last call.
#[inline]
pub(super) fn take_pending<const I: u8>() -> u32 {
    irq_state::<I>().pending.swap(0, Ordering::AcqRel)
}

/// Wait until the command phase completes, waking on controller interrupts.
///
/// The waker is registered before the first sample and the status is sampled
/// again before pausing, so an interrupt that fires during registration is not
/// missed.
async fn await_command<const I: u8, PADS>(bus: &mut SdmcBus<I, PADS>) -> Result<Response, SdmcError>
where
    PADS: SdmcPads<I>,
{
    poll_fn(|cx| {
        irq_state::<I>().waker.register(cx.waker());

        for _ in 0..2 {
            let status = take_pending::<I>() | bus.read_status();
            match bus.poll_command(status) {
                Poll::Pending => {}
                Poll::Ready(result) => return Poll::Ready(result),
            }
        }

        Poll::Pending
    })
    .await
}

/// Wait until the data phase completes, waking on controller interrupts.
///
/// `pump_data` is a no-op for the IDMA transfers this driver uses; it is still
/// called so the phase sequence matches the bus contract.
async fn await_data<const I: u8, PADS>(bus: &mut SdmcBus<I, PADS>) -> Result<(), SdmcError>
where
    PADS: SdmcPads<I>,
{
    poll_fn(|cx| {
        irq_state::<I>().waker.register(cx.waker());

        for _ in 0..2 {
            if let Err(error) = bus.pump_data() {
                return Poll::Ready(Err(error));
            }
            let status = take_pending::<I>() | bus.read_status();
            match bus.poll_data(status) {
                Poll::Pending => {}
                Poll::Ready(result) => return Poll::Ready(result),
            }
        }

        Poll::Pending
    })
    .await
}

/// Close a multi-block transfer with a manual `CMD12`.
///
/// `CMD12` must not wait for the card to release DAT0, and its response is polled
/// with a bounded spin: this driver has no software timeout, so awaiting a quirk
/// that never completes would hang the transfer.
pub(super) fn stop_transfer<const I: u8, PADS>(bus: &mut SdmcBus<I, PADS>)
where
    PADS: SdmcPads<I>,
{
    const STOP_SPINS: u32 = 4_000_000;
    bus.begin();
    // `begin` clears the controller status; drop bits captured by the handler
    // for the transfer that just finished so they cannot complete the stop
    // command early.
    let _ = take_pending::<I>();

    let stop = Command::new(12, 0, ResponseKind::Short).stop();
    if bus.start_command(stop).is_err() {
        return;
    }

    for _ in 0..STOP_SPINS {
        match bus.poll_command(take_pending::<I>() | bus.read_status()) {
            // The stop is best effort: a failed or missing response does not
            // invalidate the data that was already transferred.
            Poll::Ready(_) => return,
            Poll::Pending => core::hint::spin_loop(),
        }
    }
}

/// Run one full bus transaction: card-busy wait, command, data, and stop.
async fn execute_transaction<const I: u8, PADS>(
    bus: &mut SdmcBus<I, PADS>,
    command: Command,
    data: Data<'_>,
) -> Result<Response, SdmcError>
where
    PADS: SdmcPads<I>,
{
    // A card that is still busy must release DAT0 before the next command.
    // `CMD12` is the command that ends a busy data transfer, so it cannot wait.
    if !command.is_stop() {
        bus.wait_card_ready()?;
    }

    bus.begin();
    // `begin` clears the controller status; discard bits captured for an earlier
    // transaction so a stale completion cannot satisfy this command's wait.
    let _ = take_pending::<I>();

    let data_len = data.len();

    if data_len != 0 {
        let blocks = u16::try_from(data_len / BLOCK_SIZE).map_err(|_| SdmcError::InvalidBuffer)?;
        // SAFETY: the slice is borrowed from `data`, which lives until the end
        // of this future, so it stays valid for the whole DMA transfer.
        unsafe { bus.prepare_data(blocks, data)? };
    }

    bus.start_command(command)?;

    let response = await_command(bus).await?;

    if data_len != 0 {
        await_data(bus).await?;
        bus.finish_data()?;

        let blocks = data_len / BLOCK_SIZE;
        if blocks > 1 && matches!(command.index(), 18 | 25) {
            stop_transfer(bus);
        }
    }

    Ok(response)
}

impl<const I: u8, PADS> SdHostAsync for SdmcBus<I, PADS>
where
    PADS: SdmcPads<I>,
{
    async fn execute(&mut self, command: Command, data: Data<'_>) -> Result<Response, SdmcError> {
        match execute_transaction(self, command, data).await {
            Ok(response) => Ok(response),
            Err(error) => {
                self.abort();
                Err(error)
            }
        }
    }

    fn configure_bus(&mut self, width: BusWidth) {
        self.set_bus_width(width);
    }

    fn configure_clock(&mut self, clock: Hertz) {
        self.set_clock(clock);
    }
}

/// Interrupt handler bound to one SDMC instance.
pub struct AsyncSdmcHandler<const I: u8>;

impl<const I: u8> Handler<<Sdmc<I> as SdmcIrq<I>>::Irq> for AsyncSdmcHandler<I>
where
    Sdmc<I>: SdmcIrq<I>,
{
    unsafe fn on_interrupt() {
        // SAFETY: this handler is installed for interrupt source `I` only, so
        // it is the sole interrupt-context user of this controller and no
        // overlapping `&mut` access can be created.
        let reg = unsafe { Sdmc::<I>::regs_at_index() };
        let state = irq_state::<I>();

        // Sample the status first, then write the sampled bits back: the
        // original interrupt status register clears a bit when software writes
        // a one to it.
        let status = reg.original_int_status.read();
        let raw = status.raw();
        // SAFETY: this handler owns the controller status register in interrupt
        // context; the volatile write only drops the sampled bits.
        unsafe {
            reg.original_int_status.write(status);
        }

        // Publish the bits before waking the task. The task consumes them with
        // `swap(0)` and also samples `OINTST`, so a completion can be observed
        // exactly once even when it races with the await loop.
        state.pending.fetch_or(raw, Ordering::AcqRel);

        // Drop the controller-level request; the controller source is already
        // deasserted by the write-one-to-clear above.
        <Sdmc<I> as SdmcIrq<I>>::Irq::clear_pending();
        state.waker.wake();
    }
}

/// Write-back cache state (only with the `hadris-async` feature).
///
/// Recently written metadata blocks are held in the caller buffer so mirrored FAT
/// updates and repeated writes of one block cost one card write. Only written
/// blocks are held, never speculatively read ones.
#[cfg(feature = "hadris-async")]
pub(super) struct WriteBackCache {
    /// Caller-provided storage, set by [`AsyncSdmc::load_buffer`].
    pub(super) buffer: Option<&'static mut [u8]>,
    /// Block held by each slot, stored as `block + 1`; 0 means free.
    pub(super) blocks: [u32; WRITEBACK_SLOTS],
    /// Number of slots in use.
    pub(super) slots: u8,
}

#[cfg(feature = "hadris-async")]
impl WriteBackCache {
    const fn new() -> Self {
        Self {
            buffer: None,
            blocks: [0; WRITEBACK_SLOTS],
            slots: 0,
        }
    }
}

/// Interrupt-driven SDMC driver bound to one controller and one pin group.
///
/// Creating the driver identifies the card (which is why the constructor is
/// `async`), so a constructed value is ready for block transfers.
pub struct AsyncSdmc<const I: u8, PADS, D>
where
    PADS: SdmcPads<I>,
    D: DelayNs,
    Sdmc<I>: SdmcIrq<I>,
{
    pub(super) card: Card,
    pub(super) bus: SdmcBus<I, PADS>,
    delay: D,
    info: CardInfo,
    #[cfg(feature = "hadris-async")]
    pub(super) writeback: WriteBackCache,
    _irq: PhantomData<AsyncSdmcHandler<I>>,
}

impl<const I: u8, PADS, D> AsyncSdmc<I, PADS, D>
where
    PADS: SdmcPads<I>,
    D: DelayNs,
    Sdmc<I>: SdmcIrq<I>,
{
    /// Build the driver, identify the card and enable the controller interrupt sources.
    pub async fn new(
        reg: &'static RegisterBlock,
        pads: PADS,
        config: SdmcConfig,
        mut delay: D,
        clocks: &ClockTree,
        cmu: &mut Cmu,
    ) -> Result<Self, SdmcError> {
        // The async data path is IDMA only; a PIO configuration would never
        // complete because the FIFO handshake interrupts stay disabled.
        let mut requested = config;
        requested.data_path = DataPath::Idma;
        // Identification always runs at the default speed; the requested clock
        // is applied afterwards through `CMD6` (or directly below 25 MHz).
        let init_config = SdmcConfig {
            clock: requested.initial_clock,
            ..requested
        };

        let mut bus = SdmcBus::new(reg, pads, requested, clocks, cmu);
        bus.configure_interrupts(true);

        // Fail fast with a clear error when the slot is empty. A failure here
        // must not leave the controller interrupt sources enabled.
        if let Err(error) = bus.wait_card_present(&mut delay) {
            bus.configure_interrupts(false);
            return Err(error);
        }

        let mut card = Card::new(init_config);
        let mut info = match card.init_async(&mut bus, &mut delay).await {
            Ok(info) => info,
            Err(error) => {
                bus.configure_interrupts(false);
                return Err(error);
            }
        };

        // The `CMD6` status phase is FIFO-only and the verification read is a
        // short polling transaction, so the controller interrupt stays parked
        // for the whole speed switch. On failure the sources stay parked; this
        // driver value is dropped.
        bus.configure_interrupts(false);
        apply_configured_speed(&mut card, &mut bus, &mut delay, requested.clock)?;
        bus.configure_interrupts(true);
        info = card.info().unwrap_or(info);

        Ok(Self {
            card,
            bus,
            delay,
            info,
            #[cfg(feature = "hadris-async")]
            writeback: WriteBackCache::new(),
            _irq: PhantomData,
        })
    }

    /// Card information reported during identification.
    pub fn info(&self) -> CardInfo {
        self.info
    }

    /// Number of 512-byte blocks on the card.
    pub fn block_count(&self) -> u32 {
        self.info.blocks
    }

    /// Card clock currently programmed.
    pub fn clock(&self) -> Hertz {
        self.bus.clock()
    }

    /// Attach the caller-owned write-back buffer (up to [`WRITEBACK_SLOTS`]
    /// 512-byte slots are used).
    ///
    /// Without a buffer every metadata block is written straight to the card:
    /// correct, but a file write loses the FAT-block coalescing and gets slower.
    #[cfg(feature = "hadris-async")]
    pub fn load_buffer<const N: usize>(&mut self, buffer: &'static mut DmaBuffer<N>) {
        assert!(
            N != 0 && N.is_multiple_of(BLOCK_SIZE),
            "the SDMC buffer size must be a multiple of 512 bytes"
        );
        let slots = (N / BLOCK_SIZE).min(WRITEBACK_SLOTS) as u8;
        self.writeback.buffer = Some(buffer.as_mut_slice());
        self.writeback.blocks = [0; WRITEBACK_SLOTS];
        self.writeback.slots = slots;
    }

    /// Read whole 512-byte blocks starting at `start`.
    ///
    /// The interrupt-driven data path moves data through the internal DMA, so
    /// callers that are not cache-line aligned are staged through a temporary
    /// aligned block; `buffer` must be a whole number of 512-byte blocks.
    pub async fn read_blocks(&mut self, start: u32, buffer: &mut [u8]) -> Result<(), SdmcError> {
        #[cfg(feature = "hadris-async")]
        self.flush_writeback_before_raw().await?;
        if !is_dma_aligned(buffer.as_ptr()) {
            let mut block = DmaBuffer::<BLOCK_SIZE>::new();
            let mut done = 0;
            while done < buffer.len() {
                self.read_block_once(start + (done / BLOCK_SIZE) as u32, block.as_mut_slice())
                    .await?;
                buffer[done..done + BLOCK_SIZE].copy_from_slice(block.as_slice());
                done += BLOCK_SIZE;
            }
            return Ok(());
        }
        let Self { card, bus, .. } = self;
        card.read_blocks_async(bus, start, buffer).await
    }

    /// Write whole 512-byte blocks starting at `start`.
    ///
    /// Buffers that are not cache-line aligned are staged through a temporary
    /// aligned block, like in the read path.
    pub async fn write_blocks(&mut self, start: u32, buffer: &[u8]) -> Result<(), SdmcError> {
        #[cfg(feature = "hadris-async")]
        self.flush_writeback_before_raw().await?;
        if !is_dma_aligned(buffer.as_ptr()) {
            let mut block = DmaBuffer::<BLOCK_SIZE>::new();
            let mut done = 0;
            while done < buffer.len() {
                block
                    .as_mut_slice()
                    .copy_from_slice(&buffer[done..done + BLOCK_SIZE]);
                self.write_block_once(start + (done / BLOCK_SIZE) as u32, block.as_slice())
                    .await?;
                done += BLOCK_SIZE;
            }
            return Ok(());
        }
        let Self { card, bus, .. } = self;
        card.write_blocks_async(bus, start, buffer).await
    }

    /// Flush metadata held by the write-back cache before a raw block access.
    ///
    /// The raw API bypasses the hadris adapter, so a block that is still cached
    /// there would otherwise be read stale or overwritten later.
    #[cfg(feature = "hadris-async")]
    async fn flush_writeback_before_raw(&mut self) -> Result<(), SdmcError> {
        if !self.writeback_pending() {
            return Ok(());
        }
        self.flush_writeback()
            .await
            // The flush issues real card writes, so a failure is a device error.
            .map_err(|error| error.into_device_error().unwrap_or(SdmcError::Timeout))
    }

    /// One aligned-block transaction used by the staging paths above.
    async fn read_block_once(&mut self, start: u32, block: &mut [u8]) -> Result<(), SdmcError> {
        let Self { card, bus, .. } = self;
        card.read_blocks_async(bus, start, block).await
    }

    async fn write_block_once(&mut self, start: u32, block: &[u8]) -> Result<(), SdmcError> {
        let Self { card, bus, .. } = self;
        card.write_blocks_async(bus, start, block).await
    }

    /// Read the card-detect input, when a detect pin is muxed.
    pub fn card_present(&self) -> bool {
        self.bus.card_present()
    }

    /// Give back the controller handle, pads and delay source.
    ///
    /// The controller interrupt sources and the interrupt source binding are
    /// parked, so the caller can hand the controller to another driver mode.
    pub fn free(mut self) -> (Sdmc<I>, PADS, D) {
        self.bus.configure_interrupts(false);
        <Sdmc<I> as SdmcIrq<I>>::Irq::disable();
        let Self { bus, delay, .. } = self;
        let (reg, pads) = bus.free();
        (Sdmc::__new(reg), pads, delay)
    }
}
