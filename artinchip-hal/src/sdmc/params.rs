//! Numeric limits shared by the protocol engine, the bus and the drivers.
//!
//! Per-transfer chunk sizes derive from these values instead of repeating the
//! numbers in each module.

/// Every SD memory-card block transferred by this driver is 512 bytes.
pub const BLOCK_SIZE: usize = 512;

/// Block size as a `u32`, matching the SD command argument domain.
pub const BLOCK_SIZE_U32: u32 = 512;

/// Highest card clock the 3.3 V SDMC interface supports (HighSpeed, SDR 4-bit).
pub const MAX_CLOCK_HZ: u32 = 50_000_000;

/// Lowest card clock that is programmed through a `CMD6` speed mode.
///
/// Below this frequency the blocking/async constructors program the divider
/// directly and leave the card in its default speed mode; at or above it the
/// card is switched with `CMD6` before the clock is applied.
pub(crate) const HIGH_SPEED_MIN_HZ: u32 = 25_000_000;

/// Write-back cache slots the async adapter can use, 512 bytes each.
pub const WRITEBACK_SLOTS: usize = 4;

/// `CMD6` mode-1 request; the requested function value is OR-ed in.
pub(crate) const CMD6_SWITCH_ARG: u32 = 0x80FF_FFF0;

/// Largest multi-block write the async adapter issues, in 512-byte blocks.
///
/// A write spanning the whole descriptor chain desynchronised the following
/// metadata writes on the reference board; 128 blocks (64 KiB) is the largest
/// command that stayed reliable.
pub const MAX_WRITE_BLOCKS_PER_TRANSFER: u16 = 128;
