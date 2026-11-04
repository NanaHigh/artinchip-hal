//! SDMC instance handle and interrupt-source mapping.

use core::marker::PhantomData;

#[cfg(feature = "interrupts")]
use crate::interrupt::typelevel;

use super::register::RegisterBlock;

/// SDMC with statically known instance number.
pub struct Sdmc<const I: u8> {
    reg: *const RegisterBlock,
    _private: PhantomData<()>,
}

impl<const I: u8> Sdmc<I> {
    /// Create a new SDMC instance.
    pub const fn __new(reg: *const RegisterBlock) -> Self {
        Self {
            reg,
            _private: PhantomData,
        }
    }

    /// Get a reference to the register block.
    pub const fn register_block(&self) -> &'static RegisterBlock {
        unsafe { &*self.reg }
    }

    /// Get the register block for interrupt handlers.
    ///
    /// # Safety
    /// The caller must be the interrupt handler bound to this instance's
    /// interrupt source and must not create overlapping `&mut` access.
    #[cfg(feature = "interrupts")]
    pub(crate) unsafe fn regs_at_index() -> &'static RegisterBlock {
        const BASE: usize = 0x1044_0000;
        const STRIDE: usize = 0x1_0000;
        // SAFETY: the interrupt handler only touches the instance it is bound
        // to and all register accesses are volatile.
        unsafe { &*((BASE + (I as usize) * STRIDE) as *const RegisterBlock) }
    }
}

/// Trait mapping const generic `I` to its type-level interrupt source.
#[cfg(feature = "interrupts")]
pub trait SdmcIrq<const I: u8> {
    /// Interrupt source of this controller.
    type Irq: typelevel::Interrupt;
}

// Not every SoC exposes the SDMC interrupt sources in its type-level set
// (`m6800` has no SDMC entries). Gate the concrete mappings on the SoCs that do.
#[cfg(all(
    feature = "interrupts",
    any(feature = "d12x", feature = "d13x", feature = "g73x", feature = "d21x")
))]
impl SdmcIrq<0> for Sdmc<0> {
    type Irq = typelevel::SDMC0;
}

#[cfg(all(
    feature = "interrupts",
    any(feature = "d12x", feature = "d13x", feature = "g73x", feature = "d21x")
))]
impl SdmcIrq<1> for Sdmc<1> {
    type Irq = typelevel::SDMC1;
}
