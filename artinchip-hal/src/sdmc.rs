//! Secure Digital Host Controller (SDMC) and SD memory-card driver.
//!
//! `register`/`config`/`params`/`error`/`pads`/`instance` describe the hardware
//! interface, `clock`/`bus` program it, `card` implements the SD protocol over
//! `SdHost`/`SdHostAsync`, `blocking`/`non_blocking` are the two drivers and
//! `hadris/` adapts them to hadris block devices.
//!
//! Identification, raw IDMA transfers and the hadris FAT paths are validated on
//! D13x and D21x reference boards.

mod blocking;
mod bus;
mod card;
mod clock;
mod config;
mod error;
mod instance;
mod pads;
mod params;
mod register;
mod sdmc_ext;

#[cfg(feature = "hadris")]
mod hadris;
#[cfg(feature = "interrupts")]
mod non_blocking;

pub use embedded_time::rate::Hertz;

pub use blocking::BlockingSdmc;
pub use config::{CardInfo, DataPath, SdmcConfig};
pub use error::SdmcError;
pub use instance::Sdmc;
#[cfg(feature = "interrupts")]
pub use instance::SdmcIrq;
pub use pads::*;
pub use params::*;
pub use register::*;
pub use sdmc_ext::SdmcExt;

#[cfg(feature = "interrupts")]
pub use non_blocking::{AsyncSdmc, AsyncSdmcHandler};
