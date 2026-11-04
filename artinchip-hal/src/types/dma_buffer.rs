//! Cache-line aligned DMA buffers.

use crate::cache::CACHE_LINE;

// 64-byte alignment must cover the D-cache line of every supported SoC
// (32 bytes otherwise, 64 bytes on D21x). A SoC with a larger line has to
// increase the alignment and update this assertion.
const _: () = assert!(64 % CACHE_LINE == 0);

/// 64-byte aligned DMA buffer owned by the caller.
///
/// Used as the caller-provided buffer (`load_buffer`) and as a one-block stack
/// staging area for unaligned block-device requests. `N` is the byte length;
/// an SDMC buffer additionally requires a non-zero multiple of 512 bytes.
#[repr(C, align(64))]
pub struct DmaBuffer<const N: usize> {
    bytes: [u8; N],
}

impl<const N: usize> DmaBuffer<N> {
    /// Create a zeroed buffer.
    pub const fn new() -> Self {
        assert!(N != 0, "DmaBuffer size must be non-zero");
        Self { bytes: [0; N] }
    }

    /// Borrow the storage as a byte slice.
    pub fn as_slice(&self) -> &[u8] {
        &self.bytes
    }

    /// Mutably borrow the storage as a byte slice.
    pub fn as_mut_slice(&mut self) -> &mut [u8] {
        &mut self.bytes
    }
}

impl<const N: usize> Default for DmaBuffer<N> {
    fn default() -> Self {
        Self::new()
    }
}
