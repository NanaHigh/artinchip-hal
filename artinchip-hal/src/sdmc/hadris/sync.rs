//! Synchronous adaptor: `BlockingSdmc` as a hadris block device.
//!
//! Request validation lives in the parent module; the transfer itself (with the
//! read-ahead cache that turns 512-byte FAT reads into multi-block IDMA) is a
//! method of [`BlockingSdmc`].

use embedded_hal::delay::DelayNs;
use hadris_io::{Error, ErrorType};
use hadris_storage::sync::BlockDevice;
use hadris_storage::{BlockIndex, BlockSize};

use crate::sdmc::{BLOCK_SIZE, BlockingSdmc, SdmcError, SdmcPads};

impl<const I: u8, PADS, D> ErrorType for BlockingSdmc<I, PADS, D>
where
    PADS: SdmcPads<I>,
    D: DelayNs,
{
    type Error = SdmcError;
}

impl<const I: u8, PADS, D> BlockDevice for BlockingSdmc<I, PADS, D>
where
    PADS: SdmcPads<I>,
    D: DelayNs,
{
    fn block_size(&self) -> BlockSize {
        BlockSize::new(BLOCK_SIZE as u32).expect("512-byte blocks are a valid non-zero block size")
    }

    fn block_count(&self) -> u64 {
        BlockingSdmc::block_count(self) as u64
    }

    fn writable(&self) -> bool {
        true
    }

    fn read_blocks(
        &mut self,
        first: BlockIndex,
        buffer: &mut [u8],
    ) -> Result<(), Error<SdmcError>> {
        let (start, blocks) = super::check_request(first, buffer.len(), self.block_count())?;
        let length = blocks as usize * BLOCK_SIZE;
        self.hadris_read(start, length, buffer)
            .map_err(|error| Error::device(error, "SD card read failed"))
    }

    fn write_blocks(&mut self, first: BlockIndex, buffer: &[u8]) -> Result<(), Error<SdmcError>> {
        let (start, blocks) = super::check_request(first, buffer.len(), self.block_count())?;
        let length = blocks as usize * BLOCK_SIZE;
        self.hadris_write(start, length, buffer)
            .map_err(|error| Error::device(error, "SD card write failed"))
    }
}
