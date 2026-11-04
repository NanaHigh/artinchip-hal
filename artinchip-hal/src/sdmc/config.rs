//! Driver configuration and identified-card information.

use embedded_time::rate::Hertz;

use super::params::BLOCK_SIZE_U32;
use super::register::{BusWidth, ClockPhase};

/// Data path used for block transfers.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DataPath {
    /// Use the SDMC internal DMA (IDMA) when the buffer is DMA capable,
    /// otherwise fall back to CPU FIFO transfers.
    Auto,
    /// Always move data through the CPU FIFO.
    Pio,
    /// Require the SDMC internal DMA; reject buffers that are not DMA capable.
    Idma,
}

/// Configuration of one SDMC host instance.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SdmcConfig {
    /// Card clock used after identification.
    pub clock: Hertz,
    /// Card clock used while the card is identified. SD requires 400 kHz here.
    pub initial_clock: Hertz,
    /// Bus width requested after identification.
    pub bus_width: BusWidth,
    /// Driver clock phase.
    pub drv_phase: ClockPhase,
    /// Sample clock phase.
    pub smp_phase: ClockPhase,
    /// Data path for block transfers.
    pub data_path: DataPath,
}

impl Default for SdmcConfig {
    fn default() -> Self {
        Self {
            clock: Hertz(25_000_000),
            initial_clock: Hertz(400_000),
            bus_width: BusWidth::FourWire,
            drv_phase: ClockPhase::Deg0,
            smp_phase: ClockPhase::Deg0,
            data_path: DataPath::Auto,
        }
    }
}

/// Information read from an identified SD card.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CardInfo {
    /// Relative card address (RCA) assigned by `CMD3`.
    pub rca: u16,
    /// Whether the card uses block addressing (SDHC/SDXC).
    pub high_capacity: bool,
    /// Capacity in 512-byte blocks.
    pub blocks: u32,
    /// Read block length reported by the CSD, capped at 512 bytes.
    pub read_block_len: usize,
    /// Bus width negotiated with the card.
    pub bus_width: BusWidth,
    /// Card clock currently selected.
    pub clock: Hertz,
    /// CID response words, most significant word first.
    pub cid: [u32; 4],
    /// CSD response words, most significant word first.
    pub csd: [u32; 4],
}

impl CardInfo {
    /// Card capacity in bytes.
    pub const fn capacity_bytes(&self) -> u64 {
        self.blocks as u64 * BLOCK_SIZE_U32 as u64
    }
}
