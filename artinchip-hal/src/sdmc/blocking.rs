//! Blocking SDMC driver.
//!
//! [`BlockingSdmc`] pairs the protocol engine [`Card`] with [`SdmcBus`] and a
//! [`DelayNs`] source, polling each phase with bounded spin timeouts. This module
//! decides when to move to the next phase, appends the manual `CMD12` and aborts
//! on failure; the register sequences live in [`SdmcBus`].
//!
//! It also hosts the polling speed-switch helpers shared with the interrupt-driven
//! driver; the hadris adapter lives in `hadris.rs`.

use core::hint::spin_loop;
use core::task::Poll;

use embedded_hal::delay::DelayNs;
use embedded_time::rate::Hertz;

use super::bus::SdmcBus;
#[cfg(feature = "hadris")]
use super::bus::is_dma_aligned;
use super::card::{Card, Command, Data, MAX_BLOCKS_PER_TRANSFER, Response, ResponseKind, SdHost};
use super::clock::clamp_card_clock;
use super::params::{CMD6_SWITCH_ARG, HIGH_SPEED_MIN_HZ};
use super::register::{BusWidth, RegisterBlock};
use super::{BLOCK_SIZE, CardInfo, Sdmc, SdmcConfig, SdmcError, SdmcPads};
use crate::cmu::{ClockTree, Cmu};
use crate::types::DmaBuffer;

/// Bounded spins while the controller completes a command phase.
const COMMAND_SPINS: u32 = 4_000_000;
/// Bounded spins while a data phase completes.
const DATA_SPINS: u32 = 40_000_000;
/// Bounded spins while a stop command response arrives.
const STOP_SPINS: u32 = 4_000_000;

/// `CMD6` function 1 selector: the HighSpeed speed mode.
const HIGH_SPEED_MODE: u32 = 1;

/// Probe reads attempted after a `CMD6` speed switch.
const SPEED_PROBE_ATTEMPTS: u32 = 3;

/// Read-ahead windows; the FAT has two copies that are read alternately.
#[cfg(feature = "hadris")]
const READ_CACHE_LINES: usize = 2;

/// Read-ahead cache over the caller buffer (only with the `hadris` feature).
#[cfg(feature = "hadris")]
struct ReadAheadCache {
    /// Caller-provided storage, set by [`BlockingSdmc::load_buffer`].
    buffer: Option<&'static mut [u8]>,
    base: [u32; READ_CACHE_LINES],
    blocks: [u16; READ_CACHE_LINES],
    line_blocks: u16,
    lines: u8,
    next: u8,
}

#[cfg(feature = "hadris")]
impl ReadAheadCache {
    const fn new() -> Self {
        Self {
            buffer: None,
            base: [0; READ_CACHE_LINES],
            blocks: [0; READ_CACHE_LINES],
            line_blocks: 0,
            lines: 0,
            next: 0,
        }
    }

    /// Mark every cached line as empty.
    fn clear(&mut self) {
        self.blocks = [0; READ_CACHE_LINES];
    }
}

/// Blocking SDMC driver bound to one controller and one pin group.
///
/// Creating the driver identifies the card and applies the configured clock and
/// speed mode, so a constructed value is ready for transfers.
pub struct BlockingSdmc<const I: u8, PADS, D>
where
    PADS: SdmcPads<I>,
    D: DelayNs,
{
    card: Card,
    bus: SdmcBus<I, PADS>,
    delay: D,
    info: CardInfo,
    #[cfg(feature = "hadris")]
    read_cache: ReadAheadCache,
}

impl<const I: u8, PADS, D> BlockingSdmc<I, PADS, D>
where
    PADS: SdmcPads<I>,
    D: DelayNs,
{
    /// Build the driver, identify the card and switch to the configured speed.
    pub fn new(
        reg: &'static RegisterBlock,
        pads: PADS,
        config: SdmcConfig,
        delay: D,
        clocks: &ClockTree,
        cmu: &mut Cmu,
    ) -> Result<Self, SdmcError> {
        let init_config = SdmcConfig {
            clock: config.initial_clock,
            ..config
        };
        let mut bus = SdmcBus::new(reg, pads, config, clocks, cmu);
        let mut delay = delay;
        // Fail fast with a clear error instead of a command timeout when the
        // slot is empty. SDMC1 debounces the card-detect input, so a freshly
        // inserted card needs a few periods before `CDET_STAT` updates.
        bus.wait_card_present(&mut delay)?;
        let mut card = Card::new(init_config);
        let info = card.init_blocking(&mut bus, &mut delay)?;
        apply_configured_speed(&mut card, &mut bus, &mut delay, config.clock)?;
        let info = card.info().unwrap_or(info);
        Ok(Self {
            card,
            bus,
            delay,
            info,
            #[cfg(feature = "hadris")]
            read_cache: ReadAheadCache::new(),
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

    /// Read whole 512-byte blocks starting at `start`.
    pub fn read_blocks(&mut self, start: u32, buffer: &mut [u8]) -> Result<(), SdmcError> {
        let Self { card, bus, .. } = self;
        card.read_blocks(bus, start, buffer)
    }

    /// Write whole 512-byte blocks starting at `start`.
    pub fn write_blocks(&mut self, start: u32, buffer: &[u8]) -> Result<(), SdmcError> {
        // A raw write can invalidate cached read-ahead lines.
        #[cfg(feature = "hadris")]
        self.read_cache.clear();
        let Self { card, bus, .. } = self;
        card.write_blocks(bus, start, buffer)
    }

    /// Attach the caller-owned read-ahead buffer.
    ///
    /// The buffer is split into [`READ_CACHE_LINES`] lines of whole blocks and is
    /// also the staging area for unaligned hadris requests. Without it, 512-byte
    /// reads are issued one block at a time.
    #[cfg(feature = "hadris")]
    pub fn load_buffer<const N: usize>(&mut self, buffer: &'static mut DmaBuffer<N>) {
        assert!(
            N != 0 && N.is_multiple_of(BLOCK_SIZE),
            "the SDMC buffer size must be a multiple of 512 bytes"
        );
        let blocks = (N / BLOCK_SIZE) as u16;
        self.read_cache.lines = if blocks >= READ_CACHE_LINES as u16 {
            READ_CACHE_LINES as u8
        } else {
            1
        };
        self.read_cache.line_blocks = (blocks / u16::from(self.read_cache.lines)).min(16);
        self.read_cache.buffer = Some(buffer.as_mut_slice());
        self.read_cache.clear();
        self.read_cache.next = 0;
    }

    /// Line and index of the block that holds `start`, if it is cached.
    #[cfg(feature = "hadris")]
    fn read_cache_hit(&self, start: u32) -> Option<(usize, usize)> {
        for line in 0..READ_CACHE_LINES {
            let blocks = self.read_cache.blocks[line];
            if blocks == 0 || start < self.read_cache.base[line] {
                continue;
            }
            let index = (start - self.read_cache.base[line]) as usize;
            if index < blocks as usize {
                return Some((line, index));
            }
        }
        None
    }

    #[cfg(feature = "hadris")]
    fn read_cache_copy(&self, start: u32, out: &mut [u8]) -> bool {
        let Some((line, index)) = self.read_cache_hit(start) else {
            return false;
        };
        let Some(buffer) = self.read_cache.buffer.as_deref() else {
            return false;
        };
        let line_bytes = self.read_cache.line_blocks as usize * BLOCK_SIZE;
        let off = line * line_bytes + index * BLOCK_SIZE;
        out.copy_from_slice(&buffer[off..off + BLOCK_SIZE]);
        true
    }

    #[cfg(feature = "hadris")]
    fn fill_read_cache(&mut self, start: u32) -> Result<(), SdmcError> {
        let lines = self.read_cache.lines.max(1) as usize;
        let line = self.read_cache.next as usize % lines;
        self.read_cache.next = ((line + 1) % lines) as u8;
        let line_blocks = self.read_cache.line_blocks.max(1) as usize;
        let available = self.block_count().saturating_sub(start) as usize;
        let blocks = available.clamp(1, line_blocks);
        let bytes = blocks * BLOCK_SIZE;
        let off = line * line_blocks * BLOCK_SIZE;
        {
            let Self {
                card,
                bus,
                read_cache,
                ..
            } = self;
            let cache = read_cache
                .buffer
                .as_deref_mut()
                .ok_or(SdmcError::InvalidBuffer)?;
            card.read_blocks(bus, start, &mut cache[off..off + bytes])?;
        }
        self.read_cache.base[line] = start;
        self.read_cache.blocks[line] = blocks as u16;
        Ok(())
    }

    /// hadris read: 512-byte reads are served by the caller-supplied read-ahead
    /// buffer; aligned bulk reads go through the DMA, unaligned ones are staged
    /// through the buffer (or a stack block when no buffer was loaded).
    #[cfg(feature = "hadris")]
    pub(crate) fn hadris_read(
        &mut self,
        start: u32,
        length: usize,
        buffer: &mut [u8],
    ) -> Result<(), SdmcError> {
        self.read_once(start, length, buffer)
    }

    #[cfg(feature = "hadris")]
    fn read_once(&mut self, start: u32, length: usize, buffer: &mut [u8]) -> Result<(), SdmcError> {
        let buffer = &mut buffer[..length];
        if length == BLOCK_SIZE {
            if self.read_cache_copy(start, buffer) {
                return Ok(());
            }
            if self.read_cache.buffer.is_some() {
                self.fill_read_cache(start)?;
                if self.read_cache_copy(start, buffer) {
                    return Ok(());
                }
            }
        }
        if is_dma_aligned(buffer.as_ptr()) {
            return self.read_blocks(start, buffer);
        }
        if self.read_cache.buffer.is_some() {
            let mut done = 0;
            while done < length {
                let blocks =
                    ((length - done) / BLOCK_SIZE).min(self.read_cache.line_blocks.max(1) as usize);
                let bytes = blocks * BLOCK_SIZE;
                {
                    let Self {
                        card,
                        bus,
                        read_cache,
                        ..
                    } = self;
                    let cache = read_cache
                        .buffer
                        .as_deref_mut()
                        .ok_or(SdmcError::InvalidBuffer)?;
                    card.read_blocks(bus, start + (done / BLOCK_SIZE) as u32, &mut cache[..bytes])?;
                }
                let cache = self
                    .read_cache
                    .buffer
                    .as_deref()
                    .ok_or(SdmcError::InvalidBuffer)?;
                buffer[done..done + bytes].copy_from_slice(&cache[..bytes]);
                done += bytes;
            }
            return Ok(());
        }
        let mut block = DmaBuffer::<BLOCK_SIZE>::new();
        let mut done = 0;
        while done < length {
            self.read_blocks(start + (done / BLOCK_SIZE) as u32, block.as_mut_slice())?;
            buffer[done..done + BLOCK_SIZE].copy_from_slice(block.as_slice());
            done += BLOCK_SIZE;
        }
        Ok(())
    }

    /// hadris write: aligned buffers go through the DMA directly, unaligned ones
    /// are staged through the caller buffer (or a stack block as fallback).
    #[cfg(feature = "hadris")]
    pub(crate) fn hadris_write(
        &mut self,
        start: u32,
        length: usize,
        buffer: &[u8],
    ) -> Result<(), SdmcError> {
        self.read_cache.clear();
        self.write_once(start, length, buffer)
    }

    #[cfg(feature = "hadris")]
    fn write_once(&mut self, start: u32, length: usize, buffer: &[u8]) -> Result<(), SdmcError> {
        let buffer = &buffer[..length];
        if is_dma_aligned(buffer.as_ptr()) {
            return self.write_blocks(start, buffer);
        }
        if self.read_cache.buffer.is_some() {
            let mut done = 0;
            while done < length {
                let blocks =
                    ((length - done) / BLOCK_SIZE).min(self.read_cache.line_blocks.max(1) as usize);
                let bytes = blocks * BLOCK_SIZE;
                {
                    let Self {
                        card,
                        bus,
                        read_cache,
                        ..
                    } = self;
                    let cache = read_cache
                        .buffer
                        .as_deref_mut()
                        .ok_or(SdmcError::InvalidBuffer)?;
                    cache[..bytes].copy_from_slice(&buffer[done..done + bytes]);
                    card.write_blocks(bus, start + (done / BLOCK_SIZE) as u32, &cache[..bytes])?;
                }
                done += bytes;
            }
            return Ok(());
        }
        let mut block = DmaBuffer::<BLOCK_SIZE>::new();
        let mut done = 0;
        while done < length {
            block
                .as_mut_slice()
                .copy_from_slice(&buffer[done..done + BLOCK_SIZE]);
            self.write_blocks(start + (done / BLOCK_SIZE) as u32, block.as_slice())?;
            done += BLOCK_SIZE;
        }
        Ok(())
    }

    /// Read the card-detect input, when a detect pin is muxed.
    pub fn card_present(&self) -> bool {
        self.bus.card_present()
    }

    /// Card clock currently programmed.
    pub fn clock(&self) -> Hertz {
        self.bus.clock()
    }

    /// Give back the controller handle, pads and delay source.
    pub fn free(self) -> (Sdmc<I>, PADS, D) {
        let Self { bus, delay, .. } = self;
        let (reg, pads) = bus.free();
        (Sdmc::__new(reg), pads, delay)
    }
}

/// Program the configured card clock and switch the card to a `CMD6` speed mode
/// when the target is at least [`HIGH_SPEED_MIN_HZ`]. Both drivers call this
/// during construction (the async one with the controller interrupts parked).
pub(super) fn apply_configured_speed<const I: u8, PADS, D>(
    card: &mut Card,
    bus: &mut SdmcBus<I, PADS>,
    delay: &mut D,
    requested: Hertz,
) -> Result<(), SdmcError>
where
    PADS: SdmcPads<I>,
    D: DelayNs,
{
    let target = clamp_card_clock(requested, bus.max_card_clock());
    if target.0 >= HIGH_SPEED_MIN_HZ {
        switch_speed_polling(card, bus, delay, HIGH_SPEED_MODE, target)
    } else {
        bus.set_clock(target);
        card.set_clock(target);
        Ok(())
    }
}

/// Switch the card to a `CMD6` speed mode with polling waits.
///
/// The status phase is FIFO-only and the verification read is a short polling
/// transaction, so the switch is blocking in both drivers. The aligned probe read
/// is retried a few times because a card may need a few clocks after the switch.
fn switch_speed_polling<const I: u8, PADS, D>(
    card: &mut Card,
    bus: &mut SdmcBus<I, PADS>,
    delay: &mut D,
    mode: u32,
    target: Hertz,
) -> Result<(), SdmcError>
where
    PADS: SdmcPads<I>,
    D: DelayNs,
{
    let mut status = [0u8; 64];
    bus.wait_card_ready()?;
    bus.begin();
    bus.prepare_switch_data(&mut status)?;
    bus.start_command(Command::new(6, CMD6_SWITCH_ARG | mode, ResponseKind::Short))?;

    let mut response = None;
    for _ in 0..COMMAND_SPINS {
        match bus.poll_command(bus.read_status()) {
            Poll::Ready(Ok(value)) => {
                response = Some(value);
                break;
            }
            Poll::Ready(Err(error)) => return Err(error),
            Poll::Pending => spin_loop(),
        }
    }
    let response = response.ok_or(SdmcError::CommandTimeout)?;

    let mut data_done = false;
    for _ in 0..DATA_SPINS {
        bus.pump_data()?;
        match bus.poll_data(bus.read_status()) {
            Poll::Ready(Ok(())) => {
                // The last FIFO words can arrive together with `DAT_DONE`; the
                // PIO finish check fails if they are not drained first.
                bus.pump_data()?;
                bus.finish_data()?;
                data_done = true;
                break;
            }
            Poll::Ready(Err(error)) => return Err(error),
            Poll::Pending => spin_loop(),
        }
    }
    if !data_done {
        return Err(SdmcError::DataTimeout);
    }
    Card::check_r1(response)?;

    bus.set_clock(target);
    card.set_clock(target);

    let mut probe = DmaBuffer::<BLOCK_SIZE>::new();
    let mut last_error = SdmcError::CommandTimeout;
    for _ in 0..SPEED_PROBE_ATTEMPTS {
        delay.delay_ms(1);
        match card.read_blocks(bus, 0, probe.as_mut_slice()) {
            Ok(()) => return Ok(()),
            Err(error) => last_error = error,
        }
    }
    Err(last_error)
}

impl<const I: u8, PADS> SdmcBus<I, PADS>
where
    PADS: SdmcPads<I>,
{
    /// Run one command, including its data phase and the trailing stop command.
    fn execute_transaction(
        &mut self,
        command: Command,
        data: Data<'_>,
    ) -> Result<Response, SdmcError> {
        let has_data = !data.is_none();
        let blocks = data.blocks();
        if has_data && (blocks == 0 || blocks > MAX_BLOCKS_PER_TRANSFER as usize) {
            return Err(SdmcError::InvalidBlock);
        }
        if !command.is_stop() {
            self.wait_card_ready()?;
        }
        self.begin();

        if has_data {
            // SAFETY: the slice is borrowed from `data`, which lives for the
            // whole synchronous transaction.
            unsafe {
                self.prepare_data(blocks as u16, data)?;
            }
        }

        self.start_command(command)?;
        let response = self.poll_command_phase()?;
        if has_data {
            self.poll_data_phase()?;
            if blocks > 1 {
                self.stop_transfer()?;
            }
        }
        Ok(response)
    }

    /// Poll the command phase until it completes or the bounded timeout expires.
    fn poll_command_phase(&mut self) -> Result<Response, SdmcError> {
        for _ in 0..COMMAND_SPINS {
            match self.poll_command(self.read_status()) {
                Poll::Pending => spin_loop(),
                Poll::Ready(Ok(response)) => return Ok(response),
                Poll::Ready(Err(error)) => return Err(error),
            }
        }
        Err(SdmcError::CommandTimeout)
    }

    /// Poll the data phase, moving PIO data as the FIFO allows.
    fn poll_data_phase(&mut self) -> Result<(), SdmcError> {
        for _ in 0..DATA_SPINS {
            let status = self.read_status();
            self.pump_data()?;
            match self.poll_data(status) {
                Poll::Pending => spin_loop(),
                Poll::Ready(Ok(())) => {
                    // The last FIFO words may arrive together with `DAT_DONE`.
                    self.pump_data()?;
                    return self.finish_data();
                }
                Poll::Ready(Err(error)) => return Err(error),
            }
        }
        Err(SdmcError::DataTimeout)
    }

    /// Send the manual `CMD12` that closes a multi-block transfer.
    fn stop_transfer(&mut self) -> Result<(), SdmcError> {
        // CMD12 must be issued while the card still owns the data lines: do not
        // wait for DAT0 to go idle first.
        self.begin();
        self.start_command(Command::new(12, 0, ResponseKind::Short).stop())?;
        for _ in 0..STOP_SPINS {
            match self.poll_command(self.read_status()) {
                // Either a response or a failure ends the stop attempt: some
                // controllers do not complete the CMD12 response after a large
                // IDMA transfer, and the card still leaves the data state.
                Poll::Ready(_) => break,
                Poll::Pending => spin_loop(),
            }
        }
        Ok(())
    }
}

impl<const I: u8, PADS> SdHost for SdmcBus<I, PADS>
where
    PADS: SdmcPads<I>,
{
    fn execute(&mut self, command: Command, data: Data<'_>) -> Result<Response, SdmcError> {
        match self.execute_transaction(command, data) {
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
