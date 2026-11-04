//! Driver error type.

use core::fmt;

/// Error returned by the SDMC host drivers and the SD card protocol engine.
///
/// The variants keep enough context to tell a controller problem (timeout,
/// FIFO, DMA) apart from a card problem (R1 status, unsupported card). Card
/// errors that echo a register value carry the raw value so a caller can log
/// it and, for R1, inspect the individual status bits.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SdmcError {
    /// A controller wait exceeded its software timeout.
    Timeout,
    /// The card kept DAT0 busy longer than the software timeout.
    Busy,
    /// The command did not complete before the software timeout.
    CommandTimeout,
    /// The card did not return a response.
    ResponseTimeout,
    /// The response CRC did not match.
    ResponseCrc,
    /// The card returned an R1 status word with an error bit set.
    ResponseStatus(u32),
    /// The data phase ended with an end-bit error.
    DataEndBit,
    /// The data phase ended with a CRC error.
    DataCrc,
    /// The data phase did not finish before the software timeout.
    DataTimeout,
    /// The FIFO overran or underran during the data phase.
    Fifo,
    /// The internal DMA engine reported a fatal bus error or a descriptor error.
    Idma,
    /// No card is inserted in the slot (SDMC1 hardware card detect).
    NoCard,
    /// The card is not an SD memory card this driver understands.
    UnsupportedCard,
    /// The card has not been identified yet.
    NotInitialized,
    /// The transfer is not a whole number of 512-byte blocks.
    InvalidBlock,
    /// The transfer buffer does not satisfy the selected data path requirements.
    InvalidBuffer,
    /// The requested block range leaves the card.
    OutOfRange,
    /// An address or size calculation overflowed.
    Overflow,
}

impl fmt::Display for SdmcError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Timeout => f.write_str("SDMC timeout"),
            Self::Busy => f.write_str("SDMC card busy"),
            Self::CommandTimeout => f.write_str("SDMC command timeout"),
            Self::ResponseTimeout => f.write_str("SDMC response timeout"),
            Self::ResponseCrc => f.write_str("SDMC response CRC error"),
            Self::ResponseStatus(status) => write!(f, "SDMC card R1 status 0x{status:08X}"),
            Self::DataEndBit => f.write_str("SDMC data end-bit error"),
            Self::DataCrc => f.write_str("SDMC data CRC error"),
            Self::DataTimeout => f.write_str("SDMC data timeout"),
            Self::Fifo => f.write_str("SDMC FIFO overrun/underrun"),
            Self::Idma => f.write_str("SDMC internal DMA error"),
            Self::NoCard => f.write_str("no SD card detected in the slot"),
            Self::UnsupportedCard => f.write_str("unsupported SD card"),
            Self::NotInitialized => f.write_str("SD card is not initialized"),
            Self::InvalidBlock => f.write_str("SDMC transfer is not a whole 512-byte block"),
            Self::InvalidBuffer => f.write_str("SDMC transfer buffer is not DMA capable"),
            Self::OutOfRange => f.write_str("SDMC block range leaves the card"),
            Self::Overflow => f.write_str("SDMC address or size overflow"),
        }
    }
}

impl core::error::Error for SdmcError {}
