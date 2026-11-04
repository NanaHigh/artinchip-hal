//! Codec-typed SDMC pad traits.
//!
//! The 6-tuple implements the 4-bit bus used by the examples; the extra data
//! lines, strobe, reset and card-detect traits are used by the pin-mux tables.

/// SDMC pin set with statically known instance number.
pub trait SdmcPads<const I: u8> {}

/// SDMC clock pad.
pub trait SerialClock<const I: u8> {}

/// SDMC command pad.
pub trait CommandPin<const I: u8> {}

/// SDMC data line 0.
pub trait Data0<const I: u8> {}

/// SDMC data line 1.
pub trait Data1<const I: u8> {}

/// SDMC data line 2.
pub trait Data2<const I: u8> {}

/// SDMC data line 3.
pub trait Data3<const I: u8> {}

/// SDMC data line 4.
pub trait Data4<const I: u8> {}

/// SDMC data line 5.
pub trait Data5<const I: u8> {}

/// SDMC data line 6.
pub trait Data6<const I: u8> {}

/// SDMC data line 7.
pub trait Data7<const I: u8> {}

/// SDMC data strobe pad (8-bit bus).
pub trait DataStrobe<const I: u8> {}

/// SDMC card reset pad.
pub trait ResetPin<const I: u8> {}

/// SDMC card-detect pad.
pub trait CardDetectPin<const I: u8> {}

impl<const I: u8, CLK, CMD, D0, D1, D2, D3> SdmcPads<I> for (CLK, CMD, D0, D1, D2, D3)
where
    CLK: SerialClock<I>,
    CMD: CommandPin<I>,
    D0: Data0<I>,
    D1: Data1<I>,
    D2: Data2<I>,
    D3: Data3<I>,
{
}
