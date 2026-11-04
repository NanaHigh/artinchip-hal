//! hadris block-device seam of the SDMC drivers: request validation here plus the
//! `sync.rs` / `async_.rs` adapters, so the driver modules stay protocol-only.
//!
//! hadris hands arbitrary pointers to the block device, so sub-cache-line aligned
//! buffers are staged through one 512-byte aligned block (sync: caller buffer or
//! stack; async: per-controller bounce block). The poll-based trait must not touch
//! the caller buffer after returning `Pending`; staging guarantees it is only
//! accessed while returning `Ready`.

use hadris_io::{Error, ErrorKind, Location};
use hadris_storage::BlockIndex;

use super::{BLOCK_SIZE, SdmcError};

mod sync;

#[cfg(feature = "hadris-async")]
mod async_;

/// Validate a hadris block request against the card geometry.
fn check_request(
    first: BlockIndex,
    length: usize,
    block_count: u64,
) -> Result<(u32, u16), Error<SdmcError>> {
    if length == 0 || !length.is_multiple_of(BLOCK_SIZE) {
        return Err(Error::new(
            ErrorKind::InvalidInput,
            "buffer is not a whole number of 512-byte blocks",
        ));
    }
    let blocks = length / BLOCK_SIZE;
    let start = first.get();
    let end = start.checked_add(blocks as u64).ok_or_else(|| {
        Error::new(
            ErrorKind::InvalidInput,
            "block range overflows the address space",
        )
    })?;
    if end > block_count {
        return Err(
            Error::new(ErrorKind::InvalidInput, "block range leaves the card")
                .with_location(Location::Block(start)),
        );
    }
    let start = u32::try_from(start).map_err(|_| {
        Error::new(
            ErrorKind::InvalidInput,
            "block index exceeds the SD address space",
        )
    })?;
    let blocks = u16::try_from(blocks)
        .map_err(|_| Error::new(ErrorKind::LimitExceeded, "transfer exceeds one SD command"))?;
    Ok((start, blocks))
}

#[cfg(test)]
mod tests {
    use super::*;
    use hadris_storage::BlockIndex;

    fn check(first: u64, length: usize, block_count: u64) -> Result<(u32, u16), Error<SdmcError>> {
        check_request(BlockIndex::new(first), length, block_count)
    }

    #[test]
    fn accepts_whole_block_requests() {
        assert_eq!(check(0, BLOCK_SIZE, 1024), Ok((0, 1)));
        assert_eq!(check(100, 8 * BLOCK_SIZE, 1024), Ok((100, 8)));
        assert_eq!(
            check(0, 512 * BLOCK_SIZE, 512 * BLOCK_SIZE as u64),
            Ok((0, 512))
        );
    }

    #[test]
    fn rejects_non_block_lengths() {
        assert_eq!(
            check(0, 0, 1024).unwrap_err().kind(),
            hadris_io::ErrorKind::InvalidInput
        );
        assert_eq!(
            check(0, 100, 1024).unwrap_err().kind(),
            hadris_io::ErrorKind::InvalidInput
        );
    }

    #[test]
    fn rejects_out_of_range_requests() {
        assert_eq!(
            check(1024, BLOCK_SIZE, 1024).unwrap_err().kind(),
            hadris_io::ErrorKind::InvalidInput
        );
        assert_eq!(
            check(1023, 2 * BLOCK_SIZE, 1024).unwrap_err().kind(),
            hadris_io::ErrorKind::InvalidInput
        );
    }

    #[test]
    fn rejects_overflowing_and_oversized_requests() {
        // The block range leaves the 64-bit address space.
        assert_eq!(
            check(u64::MAX, BLOCK_SIZE, u64::MAX).unwrap_err().kind(),
            hadris_io::ErrorKind::InvalidInput
        );
        // The start block exceeds the 32-bit SD address space.
        assert_eq!(
            check(u32::MAX as u64 + 1, BLOCK_SIZE, u64::MAX)
                .unwrap_err()
                .kind(),
            hadris_io::ErrorKind::InvalidInput
        );
        // More blocks than one SD block-count field can carry.
        assert_eq!(
            check(0, (u16::MAX as usize + 1) * BLOCK_SIZE, u64::MAX)
                .unwrap_err()
                .kind(),
            hadris_io::ErrorKind::LimitExceeded
        );
    }
}
