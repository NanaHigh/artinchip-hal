//! Poll-based adaptor: `AsyncSdmc` as a hadris block device.
//!
//! Aligned requests go straight into the caller buffer; other requests bounce one
//! command's worth of data through an aligned static buffer. Four write-back slots
//! hold recently written metadata blocks (so mirrored FAT updates cost one card
//! write) and are flushed on eviction, overlap and `poll_flush`.

use core::cell::UnsafeCell;
use core::task::{Context, Poll};

use embedded_hal::delay::DelayNs;
use hadris_io::{Error, ErrorType};
use hadris_storage::async_::BlockDevice;
use hadris_storage::{BlockIndex, BlockSize};

use crate::sdmc::bus::is_dma_aligned;
use crate::sdmc::card::{Data, MAX_BLOCKS_PER_TRANSFER};
use crate::sdmc::instance::{Sdmc, SdmcIrq};
use crate::sdmc::non_blocking::{AsyncSdmc, irq_state, take_pending};
use crate::sdmc::params::{BLOCK_SIZE, MAX_WRITE_BLOCKS_PER_TRANSFER};
use crate::sdmc::register::{OriginalIntStatus, SdmcInterrupt};
use crate::sdmc::{SdmcError, SdmcPads};
use crate::types::DmaBuffer;

/// Blocks one bounce-buffer command carries for an unaligned caller buffer.
const BOUNCE_BLOCKS: usize = 1;

/// Blocks one direct-DMA read command may span: one IDMA descriptor chain.
const DIRECT_READ_BLOCKS: u16 = MAX_BLOCKS_PER_TRANSFER;

/// Blocks one direct-DMA write command may span (see
/// [`MAX_WRITE_BLOCKS_PER_TRANSFER`]).
const DIRECT_WRITE_BLOCKS: u16 = MAX_WRITE_BLOCKS_PER_TRANSFER;

/// One command's worth of bounce buffer.
const BOUNCE_BYTES: usize = BOUNCE_BLOCKS * BLOCK_SIZE;

struct BounceCell(UnsafeCell<DmaBuffer<BOUNCE_BYTES>>);

// SAFETY: a single async driver exists per controller, and the buffer is only
// reached through `&mut self`, so there is no aliasing.
unsafe impl Sync for BounceCell {}

static BOUNCE: [BounceCell; 2] = [
    BounceCell(UnsafeCell::new(DmaBuffer::new())),
    BounceCell(UnsafeCell::new(DmaBuffer::new())),
];

/// `block` value of a free write-back slot.
const WRITE_FREE: u32 = 0;

/// Transfer state of one hadris block operation.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct AsyncBlockState {
    phase: TransferPhase,
    operation: Operation,
    /// First block of the hadris request.
    first: u32,
    /// Blocks in the request; zero while idle.
    blocks: u16,
    /// Blocks of the request already transferred.
    cursor: u16,
    /// Blocks in the controller transfer that is currently running.
    fill: u16,
    /// The caller buffer is DMA capable and used directly (no bounce copy).
    direct: bool,
    /// DataDone was already seen while waiting for the command completion.
    data_done: bool,
    /// Write-back slot the flush state machine is working on.
    flush_slot: u8,
    /// Operation to resume once the write-back slots have been flushed.
    after_flush: Operation,
    /// Insert the pending 512-byte write after flushing the slots.
    insert: bool,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
enum TransferPhase {
    #[default]
    Idle,
    Command,
    Data,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
enum Operation {
    #[default]
    Idle,
    Read,
    Write,
    Flush,
}

impl<const I: u8, PADS, D> AsyncSdmc<I, PADS, D>
where
    PADS: SdmcPads<I>,
    D: DelayNs,
    Sdmc<I>: SdmcIrq<I>,
{
    /// Abort the controller and forget the operation.
    fn fail(
        &mut self,
        state: &mut AsyncBlockState,
        error: SdmcError,
    ) -> Poll<Result<(), Error<SdmcError>>> {
        self.bus.abort();
        self.writeback_clear();
        *state = AsyncBlockState::default();
        Poll::Ready(Err(Error::device(error, "SD card block transfer failed")))
    }

    /// Bounce buffer of this controller.
    fn bounce() -> &'static mut [u8] {
        const { assert!((I as usize) < BOUNCE.len()) };
        // SAFETY: a single async driver exists per controller, so the static is
        // only reached through `&mut self`.
        unsafe { (*BOUNCE[I as usize].0.get()).as_mut_slice() }
    }

    /// Slot that holds `block`, if any.
    fn writeback_slot(&self, block: u32) -> Option<usize> {
        let slots = self.writeback.slots as usize;
        self.writeback.blocks[..slots]
            .iter()
            .position(|&held| held == block.wrapping_add(1))
    }

    fn writeback_room(&self) -> Option<usize> {
        let slots = self.writeback.slots as usize;
        self.writeback.blocks[..slots]
            .iter()
            .position(|&held| held == WRITE_FREE)
    }

    fn writeback_count(&self) -> usize {
        let slots = self.writeback.slots as usize;
        self.writeback.blocks[..slots]
            .iter()
            .filter(|&&held| held != WRITE_FREE)
            .count()
    }

    /// Blocks currently held in the write-back slots.
    ///
    /// The raw `AsyncSdmc::read_blocks`/`write_blocks` API flushes these before
    /// touching the card, so raw accesses cannot observe or overwrite stale
    /// metadata blocks.
    fn writeback_held_blocks(&self) -> impl Iterator<Item = u32> + '_ {
        self.writeback
            .blocks
            .iter()
            .take(self.writeback.slots as usize)
            .copied()
            .filter(|&held| held != WRITE_FREE)
            .map(|held| held - 1)
    }

    fn writeback_store(&mut self, slot: usize, block: u32) {
        self.writeback.blocks[slot] = block.wrapping_add(1);
    }

    fn writeback_free(&mut self, slot: usize) {
        self.writeback.blocks[slot] = WRITE_FREE;
    }

    fn writeback_clear(&mut self) {
        for slot in 0..self.writeback.slots as usize {
            self.writeback.blocks[slot] = WRITE_FREE;
        }
    }

    fn writeback_data(&self, slot: usize) -> &[u8] {
        let buffer = self
            .writeback
            .buffer
            .as_deref()
            .expect("write-back buffer loaded");
        &buffer[slot * BLOCK_SIZE..][..BLOCK_SIZE]
    }

    fn writeback_data_mut(&mut self, slot: usize) -> &mut [u8] {
        let buffer = self
            .writeback
            .buffer
            .as_deref_mut()
            .expect("write-back buffer loaded");
        &mut buffer[slot * BLOCK_SIZE..][..BLOCK_SIZE]
    }

    /// Whether the write-back cache currently holds blocks.
    pub(crate) fn writeback_pending(&self) -> bool {
        self.writeback_count() != 0
    }

    /// Write every buffered 512-byte block back to the card.
    ///
    /// hadris does not flush on unmount, so a caller that keeps using the driver
    /// after unmounting must empty the cache explicitly.
    pub async fn flush_writeback(&mut self) -> Result<(), Error<SdmcError>> {
        use core::future::poll_fn;
        let mut state = AsyncBlockState::default();
        poll_fn(
            |cx| match <Self as BlockDevice>::poll_flush(self, &mut state, cx) {
                Poll::Ready(Ok(())) => Poll::Ready(Ok(())),
                Poll::Ready(Err(error)) => Poll::Ready(Err(error)),
                Poll::Pending => Poll::Pending,
            },
        )
        .await
    }

    /// Blocks one command carries: the rest of the request, capped to `limit`.
    fn fetch_blocks(remaining: u16, start: u32, block_count: u32, limit: u16) -> u16 {
        let rest = block_count.saturating_sub(start).min(u32::from(u16::MAX)) as u16;
        (remaining.min(limit).min(rest)).max(1)
    }

    /// Return to the idle transfer state.
    fn finish(state: &mut AsyncBlockState) {
        *state = AsyncBlockState::default();
    }

    /// Drive the command and data phases of one block transfer.
    ///
    /// Returns `Ready` once the controller transfer that was started at
    /// `state.fill` blocks has completed.
    fn poll_transfer(
        &mut self,
        state: &mut AsyncBlockState,
        cx: &mut Context<'_>,
    ) -> Poll<Result<(), Error<SdmcError>>> {
        loop {
            match state.phase {
                TransferPhase::Command => {
                    irq_state::<I>().waker.register(cx.waker());
                    let status = take_pending::<I>() | self.bus.read_status();
                    match self.bus.poll_command(status) {
                        Poll::Pending => return Poll::Pending,
                        Poll::Ready(Err(error)) => return self.fail(state, error),
                        Poll::Ready(Ok(_)) => {
                            // A short write can signal DataDone while the command
                            // completion is still being sampled; `take_pending`
                            // consumes it, so remember it for the data phase.
                            state.data_done = OriginalIntStatus::from_bits(status)
                                .is_interrupt_pending(SdmcInterrupt::DataDone);
                            state.phase = TransferPhase::Data;
                        }
                    }
                }
                TransferPhase::Data => {
                    if state.data_done {
                        state.data_done = false;
                        if let Err(error) = self.bus.finish_data() {
                            return self.fail(state, error);
                        }
                        if state.fill > 1 {
                            super::super::non_blocking::stop_transfer(&mut self.bus);
                        }
                        state.phase = TransferPhase::Idle;
                        return Poll::Ready(Ok(()));
                    }
                    irq_state::<I>().waker.register(cx.waker());
                    if let Err(error) = self.bus.pump_data() {
                        return self.fail(state, error);
                    }
                    let pending = take_pending::<I>();
                    let status = pending | self.bus.read_status();
                    match self.bus.poll_data(status) {
                        Poll::Pending => return Poll::Pending,
                        Poll::Ready(Err(error)) => return self.fail(state, error),
                        Poll::Ready(Ok(())) => {
                            if let Err(error) = self.bus.finish_data() {
                                return self.fail(state, error);
                            }
                            // A multi-block read or write must be closed with
                            // `CMD12` before the card accepts the next command.
                            if state.fill > 1 {
                                super::super::non_blocking::stop_transfer(&mut self.bus);
                            }
                            state.phase = TransferPhase::Idle;
                            return Poll::Ready(Ok(()));
                        }
                    }
                }
                TransferPhase::Idle => return Poll::Ready(Ok(())),
            }
        }
    }

    /// Program one buffered slot for write-back.
    fn start_slot_write(&mut self, slot: usize) -> Result<(), SdmcError> {
        let block = self.writeback.blocks[slot] - 1;
        let command = self.card.write_command(block, 1)?;
        self.bus.wait_card_ready()?;
        self.bus.begin();
        let _ = take_pending::<I>();
        let Self { bus, writeback, .. } = self;
        let buffer = writeback
            .buffer
            .as_deref_mut()
            .ok_or(SdmcError::InvalidBuffer)?;
        let data = Data::Write(&buffer[slot * BLOCK_SIZE..][..BLOCK_SIZE]);
        // SAFETY: the slot lives in the caller buffer and is not touched until
        // the transfer completes.
        unsafe { bus.prepare_data(1, data) }?;
        bus.start_command(command)?;
        Ok(())
    }

    /// Write the buffered 512-byte blocks back to the card.
    fn poll_writeback_flush(
        &mut self,
        state: &mut AsyncBlockState,
        cx: &mut Context<'_>,
    ) -> Poll<Result<(), Error<SdmcError>>> {
        loop {
            match state.phase {
                TransferPhase::Idle => {
                    let slots = self.writeback.slots as usize;
                    let mut slot = state.flush_slot as usize;
                    while slot < slots && self.writeback.blocks[slot] == WRITE_FREE {
                        slot += 1;
                    }
                    state.flush_slot = slot as u8;
                    if slot == slots {
                        return Poll::Ready(Ok(()));
                    }
                    if let Err(error) = self.start_slot_write(slot) {
                        return self.fail(state, error);
                    }
                    state.fill = 1;
                    state.phase = TransferPhase::Command;
                }
                TransferPhase::Command | TransferPhase::Data => {
                    match self.poll_transfer(state, cx) {
                        Poll::Ready(Ok(())) => {
                            self.writeback_free(state.flush_slot as usize);
                            state.flush_slot += 1;
                            state.phase = TransferPhase::Idle;
                        }
                        other => return other,
                    }
                }
            }
        }
    }
}

impl<const I: u8, PADS, D> ErrorType for AsyncSdmc<I, PADS, D>
where
    PADS: SdmcPads<I>,
    D: DelayNs,
    Sdmc<I>: SdmcIrq<I>,
{
    type Error = SdmcError;
}

impl<const I: u8, PADS, D> BlockDevice for AsyncSdmc<I, PADS, D>
where
    PADS: SdmcPads<I>,
    D: DelayNs,
    Sdmc<I>: SdmcIrq<I>,
{
    type State = AsyncBlockState;

    fn block_size(&self) -> BlockSize {
        BlockSize::new(BLOCK_SIZE as u32).expect("512-byte blocks are a valid non-zero block size")
    }

    fn block_count(&self) -> u64 {
        AsyncSdmc::block_count(self) as u64
    }

    fn writable(&self) -> bool {
        true
    }

    fn poll_read_blocks(
        &mut self,
        state: &mut Self::State,
        cx: &mut Context<'_>,
        first: BlockIndex,
        buffer: &mut [u8],
    ) -> Poll<Result<(), Error<SdmcError>>> {
        if state.operation == Operation::Idle {
            let (start, blocks) =
                match super::check_request(first, buffer.len(), self.block_count()) {
                    Ok(request) => request,
                    Err(error) => return Poll::Ready(Err(error)),
                };
            // A buffered metadata block is answered from the buffer.
            if blocks == 1
                && let Some(slot) = self.writeback_slot(start)
            {
                buffer.copy_from_slice(self.writeback_data(slot));
                return Poll::Ready(Ok(()));
            }
            // A larger read that overlaps a buffered block must see its contents
            // on the card, so flush the cached blocks first.
            let overlaps = self
                .writeback_held_blocks()
                .any(|held| held >= start && held < start + u32::from(blocks));
            state.operation = if overlaps {
                state.after_flush = Operation::Read;
                state.insert = false;
                Operation::Flush
            } else {
                Operation::Read
            };
            state.first = start;
            state.blocks = blocks;
            state.cursor = 0;
            state.fill = 0;
            state.direct = is_dma_aligned(buffer.as_ptr());
        }
        loop {
            match state.operation {
                Operation::Flush => match self.poll_writeback_flush(state, cx) {
                    Poll::Ready(Ok(())) => {
                        state.operation = state.after_flush;
                        state.phase = TransferPhase::Idle;
                        continue;
                    }
                    other => return other,
                },
                Operation::Read => {
                    if state.phase != TransferPhase::Idle {
                        match self.poll_transfer(state, cx) {
                            Poll::Ready(Ok(())) => {
                                let from = state.cursor as usize * BLOCK_SIZE;
                                if !state.direct {
                                    let bytes = state.fill as usize * BLOCK_SIZE;
                                    buffer[from..from + bytes]
                                        .copy_from_slice(&Self::bounce()[..bytes]);
                                }
                                state.cursor += state.fill;
                                if state.cursor == state.blocks {
                                    Self::finish(state);
                                    return Poll::Ready(Ok(()));
                                }
                                continue;
                            }
                            other => return other,
                        }
                    }
                    let remaining = state.blocks - state.cursor;
                    let start = state.first + u32::from(state.cursor);
                    let limit = if state.direct {
                        DIRECT_READ_BLOCKS
                    } else {
                        BOUNCE_BLOCKS as u16
                    };
                    let blocks =
                        Self::fetch_blocks(remaining, start, AsyncSdmc::block_count(self), limit);
                    let bytes = blocks as usize * BLOCK_SIZE;
                    let from = state.cursor as usize * BLOCK_SIZE;
                    let data = if state.direct {
                        Data::Read(&mut buffer[from..from + bytes])
                    } else {
                        Data::Read(&mut Self::bounce()[..bytes])
                    };
                    let command = match self.card.read_command(start, blocks) {
                        Ok(command) => command,
                        Err(error) => return self.fail(state, error),
                    };
                    if let Err(error) = self.bus.wait_card_ready() {
                        return self.fail(state, error);
                    }
                    self.bus.begin();
                    // Drop stale completion bits so they cannot satisfy this
                    // transfer.
                    let _ = take_pending::<I>();
                    // SAFETY: `data` borrows memory that stays valid for the
                    // whole transfer (the caller's slice or the static bounce
                    // buffer).
                    if let Err(error) = unsafe { self.bus.prepare_data(blocks, data) } {
                        return self.fail(state, error);
                    }
                    if let Err(error) = self.bus.start_command(command) {
                        return self.fail(state, error);
                    }
                    state.fill = blocks;
                    state.phase = TransferPhase::Command;
                }
                _ => return Poll::Ready(Ok(())),
            }
        }
    }

    fn poll_write_blocks(
        &mut self,
        state: &mut Self::State,
        cx: &mut Context<'_>,
        first: BlockIndex,
        buffer: &[u8],
    ) -> Poll<Result<(), Error<SdmcError>>> {
        if state.operation == Operation::Idle {
            let (start, blocks) =
                match super::check_request(first, buffer.len(), self.block_count()) {
                    Ok(request) => request,
                    Err(error) => return Poll::Ready(Err(error)),
                };
            state.first = start;
            state.blocks = blocks;
            state.cursor = 0;
            state.fill = 0;
            state.data_done = false;
            state.direct = is_dma_aligned(buffer.as_ptr());
            if self.writeback.slots == 0 {
                // No caller buffer loaded: write metadata straight to the card.
                state.operation = Operation::Write;
            } else if blocks == 1 {
                // 512-byte metadata: coalesce updates of the same block.
                if let Some(slot) = self.writeback_slot(start) {
                    self.writeback_data_mut(slot).copy_from_slice(buffer);
                    Self::finish(state);
                    return Poll::Ready(Ok(()));
                }
                if let Some(slot) = self.writeback_room() {
                    self.writeback_data_mut(slot).copy_from_slice(buffer);
                    self.writeback_store(slot, start);
                    Self::finish(state);
                    return Poll::Ready(Ok(()));
                }
                state.operation = Operation::Flush;
                state.after_flush = Operation::Write;
                state.insert = true;
            } else {
                // Data runs may overtake buffered metadata: reads that need the
                // buffered blocks hit the slots (or flush them) instead, so the
                // updates of one FAT block stay merged across the whole file.
                state.operation = Operation::Write;
            }
        }
        loop {
            match state.operation {
                Operation::Flush => match self.poll_writeback_flush(state, cx) {
                    Poll::Ready(Ok(())) => {
                        if state.insert {
                            state.insert = false;
                            let slot = self
                                .writeback_room()
                                .expect("a flushed cache has a free slot");
                            self.writeback_data_mut(slot).copy_from_slice(buffer);
                            self.writeback_store(slot, state.first);
                            Self::finish(state);
                            return Poll::Ready(Ok(()));
                        }
                        state.operation = state.after_flush;
                        state.phase = TransferPhase::Idle;
                        continue;
                    }
                    other => return other,
                },
                Operation::Write => {
                    if state.phase != TransferPhase::Idle {
                        match self.poll_transfer(state, cx) {
                            Poll::Ready(Ok(())) => {
                                state.cursor += state.fill;
                                if state.cursor == state.blocks {
                                    Self::finish(state);
                                    return Poll::Ready(Ok(()));
                                }
                                continue;
                            }
                            other => return other,
                        }
                    }
                    let remaining = state.blocks - state.cursor;
                    let start = state.first + u32::from(state.cursor);
                    let limit = if state.direct {
                        DIRECT_WRITE_BLOCKS
                    } else {
                        BOUNCE_BLOCKS as u16
                    };
                    let blocks =
                        Self::fetch_blocks(remaining, start, AsyncSdmc::block_count(self), limit);
                    let bytes = blocks as usize * BLOCK_SIZE;
                    let from = state.cursor as usize * BLOCK_SIZE;
                    let data = if state.direct {
                        Data::Write(&buffer[from..from + bytes])
                    } else {
                        Self::bounce()[..bytes].copy_from_slice(&buffer[from..from + bytes]);
                        Data::Write(&Self::bounce()[..bytes])
                    };
                    let command = match self.card.write_command(start, blocks) {
                        Ok(command) => command,
                        Err(error) => return self.fail(state, error),
                    };
                    if let Err(error) = self.bus.wait_card_ready() {
                        return self.fail(state, error);
                    }
                    self.bus.begin();
                    let _ = take_pending::<I>();
                    // SAFETY: `data` borrows memory that stays valid for the
                    // whole transfer (the caller's slice or the static bounce
                    // buffer).
                    if let Err(error) = unsafe { self.bus.prepare_data(blocks, data) } {
                        return self.fail(state, error);
                    }
                    if let Err(error) = self.bus.start_command(command) {
                        return self.fail(state, error);
                    }
                    state.fill = blocks;
                    state.phase = TransferPhase::Command;
                }
                _ => return Poll::Ready(Ok(())),
            }
        }
    }

    fn poll_flush(
        &mut self,
        state: &mut Self::State,
        cx: &mut Context<'_>,
    ) -> Poll<Result<(), Error<SdmcError>>> {
        if state.operation == Operation::Idle {
            if self.writeback_count() == 0 {
                return Poll::Ready(Ok(()));
            }
            state.flush_slot = 0;
            state.operation = Operation::Flush;
            state.insert = false;
            state.after_flush = Operation::Idle;
        }
        match state.operation {
            Operation::Flush => match self.poll_writeback_flush(state, cx) {
                Poll::Ready(Ok(())) => {
                    Self::finish(state);
                    Poll::Ready(Ok(()))
                }
                other => other,
            },
            _ => Poll::Ready(Ok(())),
        }
    }

    fn cancel(&mut self, state: &mut Self::State) {
        if state.operation != Operation::Idle {
            self.bus.abort();
        }
        self.writeback_clear();
        *state = AsyncBlockState::default();
    }
}
