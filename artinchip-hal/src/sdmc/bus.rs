//! Shared SDMC controller bus primitives.
//!
//! [`SdmcBus`] owns the register block, pads, clock/bus-width state and IDMA
//! descriptor table, and exposes one transaction as explicit phases shared by
//! both drivers:
//!
//! `begin -> prepare_data? -> start_command -> poll_command
//!  -> pump/poll_data* -> finish_data -> stop command`
//!
//! The register sequences follow the vendor C SDK (`hal_sdmc.c` / `sdmc.c`); the
//! timing-sensitive parts are validated on the reference boards. The bus and its
//! helpers are `pub(crate)` implementation details.

use core::cell::UnsafeCell;
use core::hint::spin_loop;
use core::ptr;
use core::task::Poll;

use embedded_hal::delay::DelayNs;
use embedded_time::rate::Hertz;

use super::card::{Command, Data, Response, ResponseKind};
use super::clock::{configure_module_clock, set_card_clock};
use super::register::{
    BusWidth, CmdRespLen, CmdTransferDir, CommandConfig, IntEnable, OriginalIntStatus,
    RegisterBlock, SdmcBurstLength, SdmcInterrupt,
};
use super::{BLOCK_SIZE, DataPath, SdmcConfig, SdmcError, SdmcPads};
use crate::cache::{CACHE_LINE, dcache_clean_invalidate_range, dcache_invalidate_range};
use crate::cmu::{ClockTree, Cmu};

/// Number of 32-bit words in the controller data FIFO.
///
/// The vendor HAL derives the watermarks from the FIFO depth; this controller
/// is documented with a 16-word data FIFO, which yields the recommended
/// `MSIZE(2) | RX_WMARK(depth / 2 - 1) | TX_WMARK(depth / 2)`.
const FIFO_DEPTH_WORDS: usize = 16;

/// Bounded spins while the self-clearing controller reset bits settle.
const RESET_SPINS: u32 = 100_000;

/// Bounded spins while waiting for the card to release DAT0.
const BUSY_SPINS: u32 = 20_000_000;

/// Card-detect debounce attempts after power-on.
const CARD_DETECT_ATTEMPTS: u32 = 20;

/// Card-detect debounce period in milliseconds.
const CARD_DETECT_PERIOD_MS: u32 = 10;

/// IDMA descriptor table per controller.
///
/// The chain table is required by the controller itself and is not a data
/// buffer; keeping it out of the driver value removes 1 KiB from every driver
/// instance that lives on the stack.
struct DescriptorCell(UnsafeCell<DescriptorTable>);

// SAFETY: a single driver exists per controller and the table is only reached
// through `&mut self`.
unsafe impl Sync for DescriptorCell {}

static DESCRIPTORS: [DescriptorCell; 2] = [
    DescriptorCell(UnsafeCell::new(DescriptorTable::new())),
    DescriptorCell(UnsafeCell::new(DescriptorTable::new())),
];

/// SDMC controller bus with one statically known instance number.
pub(crate) struct SdmcBus<const I: u8, PADS>
where
    PADS: SdmcPads<I>,
{
    reg: &'static RegisterBlock,
    _pads: PADS,
    config: SdmcConfig,
    module_hz: u32,
    clock: Hertz,
    direction: Option<CmdTransferDir>,
    command: Option<Command>,
    buffer: *mut u8,
    buffer_len: usize,
    cursor: usize,
    idma: bool,
}

impl<const I: u8, PADS> SdmcBus<I, PADS>
where
    PADS: SdmcPads<I>,
{
    /// IDMA descriptor table of this controller.
    fn descriptors() -> &'static mut DescriptorTable {
        const { assert!((I as usize) < DESCRIPTORS.len()) };
        // SAFETY: a single driver exists per controller and the table is only
        // reached through `&mut self`.
        unsafe { &mut *DESCRIPTORS[I as usize].0.get() }
    }

    /// Configure the SDMC module clock and reset the controller.
    ///
    /// Selects the configured initial card clock. Pads are consumed by the bus
    /// so the pin mux cannot be reused while the controller drives the bus.
    pub(crate) fn new(
        reg: &'static RegisterBlock,
        pads: PADS,
        config: SdmcConfig,
        clocks: &ClockTree,
        cmu: &mut Cmu,
    ) -> Self {
        let module_hz = configure_module_clock::<I>(clocks, cmu, config.clock);
        let mut this = Self {
            reg,
            _pads: pads,
            config,
            module_hz,
            clock: Hertz(0),
            direction: None,
            command: None,
            buffer: ptr::null_mut(),
            buffer_len: 0,
            cursor: 0,
            idma: false,
        };
        this.reset_controller();
        this.set_clock(config.initial_clock);
        this.init_controller();
        this.configure_fifo();
        if I == 1 {
            // Hardware card-detect unit: about 10 ms debounce. The counter runs
            // at half the module clock, so scale it with the real module clock
            // instead of a fixed count (the PLL differs per SoC/board).
            let counts = (this.module_hz / 200).clamp(1, 0xFF_FFFF);
            unsafe {
                this.reg
                    .card_detect
                    .modify(|v| v.set_debounce_count(counts));
            }
        }
        this.set_bus_width(BusWidth::OneWire);
        this
    }

    /// Current card clock.
    pub(crate) fn clock(&self) -> Hertz {
        self.clock
    }

    /// Whether a card is present in the slot.
    ///
    /// Only SDMC1 has the hardware card-detect unit; `CDET_STAT` reads 1 when
    /// the detect pin is high (no card, external pull-up) and 0 when the card
    /// seat pulls it low. Other controllers have no hardware detect here (the
    /// board would use a GPIO and a different driver path), so they report
    /// "present".
    pub(crate) fn card_present(&self) -> bool {
        if I == 1 {
            !self.reg.card_detect.read().card_detect_status()
        } else {
            true
        }
    }

    /// Program the controller bus width.
    pub(crate) fn set_bus_width(&mut self, width: BusWidth) {
        // SAFETY: the register block is a valid MMIO region.
        unsafe {
            self.reg.host_ctrl2.modify(|v| v.set_bus_width(width));
        }
    }

    /// Wait for the card-detect debounce after power-on.
    ///
    /// Only SDMC1 has the hardware card-detect unit; other controllers report
    /// [`SdmcError::NoCard`] never and return immediately. A freshly inserted
    /// card needs a few debounce periods before `CDET_STAT` updates, so this
    /// polls for up to [`CARD_DETECT_ATTEMPTS`] × [`CARD_DETECT_PERIOD_MS`].
    pub(crate) fn wait_card_present<D>(&self, delay: &mut D) -> Result<(), SdmcError>
    where
        D: DelayNs,
    {
        if I != 1 {
            return Ok(());
        }
        for _ in 0..CARD_DETECT_ATTEMPTS {
            if self.card_present() {
                return Ok(());
            }
            delay.delay_ms(CARD_DETECT_PERIOD_MS);
        }
        Err(SdmcError::NoCard)
    }

    /// Highest card clock the module clock can generate.
    pub(crate) fn max_card_clock(&self) -> Hertz {
        super::clock::max_card_clock(self.module_hz)
    }

    /// Program the card clock. `clock` is the frequency requested for `CCLK`.
    ///
    /// The divider chain is chosen by [`super::clock::select_clock`]; the
    /// register update lives in [`super::clock::set_card_clock`].
    pub(crate) fn set_clock(&mut self, clock: Hertz) {
        self.clock = set_card_clock(self.reg, &self.config, self.module_hz, clock);
    }

    /// Enable or disable the interrupt sources the async handler needs.
    ///
    /// Blocking drivers keep the sources disabled; the interrupt-driven driver
    /// calls this with `true` before the first transaction.
    #[cfg(feature = "interrupts")]
    pub(crate) fn configure_interrupts(&mut self, enabled: bool) {
        // SAFETY: the register block is a valid MMIO region.
        unsafe {
            if enabled {
                let enable = IntEnable::zeroed()
                    .enable_interrupt(SdmcInterrupt::CmdComplete)
                    .enable_interrupt(SdmcInterrupt::DataDone)
                    .enable_interrupt(SdmcInterrupt::RespTimeout)
                    .enable_interrupt(SdmcInterrupt::RespError)
                    .enable_interrupt(SdmcInterrupt::RespCrcError)
                    .enable_interrupt(SdmcInterrupt::DataCrcError)
                    .enable_interrupt(SdmcInterrupt::ReadTimeout)
                    .enable_interrupt(SdmcInterrupt::FifoUndOverrun)
                    .enable_interrupt(SdmcInterrupt::HostTimeoutOrVSwitch)
                    .enable_interrupt(SdmcInterrupt::EndBitOrWriteNoCrc);
                self.reg.int_enable.write(enable);
                self.reg.host_ctrl1.modify(|v| v.enable_int());
            } else {
                self.reg.host_ctrl1.modify(|v| v.disable_int());
                self.reg
                    .int_enable
                    .modify(|v| v.disable_all().disable_sdmc_interrupt());
            }
        }
    }

    /// Reset transient controller state before starting a transaction.
    ///
    /// Clears the pending interrupt status, resets the FIFO and drops any stale
    /// data-path state.
    pub(crate) fn begin(&mut self) {
        self.drop_data_state();
        self.clear_interrupts();
        self.clear_idma_status();
        // The data FIFO is reset by `prepare_data`/`prepare_switch_data` for the
        // commands that use it, so a reset here would only add latency.
    }

    /// Wait until the card releases DAT0 after a command with a busy phase.
    pub(crate) fn wait_card_ready(&mut self) -> Result<(), SdmcError> {
        for _ in 0..BUSY_SPINS {
            if !self.reg.ctrl_status.read().data0_busy() {
                return Ok(());
            }
            spin_loop();
        }
        Err(SdmcError::Busy)
    }

    /// Program the block count/size and the data path.
    ///
    /// # Safety
    /// The buffer must stay valid for `len` bytes for the whole data phase;
    /// callers pass a slice borrowed from state that outlives the transfer.
    pub(crate) unsafe fn prepare_data(
        &mut self,
        blocks: u16,
        data: Data<'_>,
    ) -> Result<(), SdmcError> {
        let (direction, buffer, len) = match data {
            Data::Read(slice) => (CmdTransferDir::Card2Host, slice.as_mut_ptr(), slice.len()),
            Data::Write(slice) => (
                CmdTransferDir::Host2Card,
                slice.as_ptr().cast_mut(),
                slice.len(),
            ),
            Data::None => return Err(SdmcError::InvalidBlock),
        };

        if blocks == 0 || len != blocks as usize * BLOCK_SIZE {
            return Err(SdmcError::InvalidBlock);
        }

        // The async driver requires the internal DMA; explicit `Idma` rejects
        // buffers the bus master cannot reach, `Auto` falls back to the FIFO.
        let idma = match self.config.data_path {
            DataPath::Pio => false,
            DataPath::Idma => {
                if !is_dma_capable(buffer.cast_const(), len) {
                    return Err(SdmcError::InvalidBuffer);
                }
                true
            }
            DataPath::Auto => is_dma_capable(buffer.cast_const(), len),
        };

        // SAFETY: the register block is a valid MMIO region.
        unsafe {
            self.reg
                .block_size
                .write(self.reg.block_size.read().set_block_size(BLOCK_SIZE as u16));
            self.reg
                .block_count
                .write(self.reg.block_count.read().set_block_count(blocks));
        }
        self.reset_fifo()?;

        self.direction = Some(direction);
        self.command = None;
        self.buffer = buffer;
        self.buffer_len = len;
        self.cursor = 0;
        self.idma = idma;

        if idma {
            // SAFETY: the caller guarantees `buffer` is valid for `len` bytes
            // for the whole transfer.
            let count = unsafe { Self::descriptors().build(buffer, len)? };
            let desc = Self::descriptors().as_ptr();
            // SAFETY: the register block is a valid MMIO region and `desc`
            // points at the cache-line aligned descriptor table owned by `self`.
            unsafe {
                self.reg.desc_list_addr.write(desc as u32);
                dcache_clean_invalidate_range(desc as usize, DescriptorTable::bytes(count));
                match direction {
                    // Card-to-host: the buffer is overwritten, and the CPU cache
                    // lines are invalidated before the DMA, so a single
                    // invalidate is enough and no dirty line has to be written
                    // back (a full maintenance pass per transfer is expensive on
                    // this core).
                    CmdTransferDir::Card2Host => dcache_invalidate_range(buffer as usize, len),
                    // Host-to-card: the card must read the CPU's latest bytes.
                    CmdTransferDir::Host2Card => {
                        dcache_clean_invalidate_range(buffer as usize, len)
                    }
                }
                self.reg
                    .host_ctrl1
                    .modify(|v| v.enable_idmac().set_dma_rst(true));
                self.reg
                    .periph_bus_config
                    .modify(|v| v.enable_fixed_burst_length().enable_idma());
            }
        } else {
            self.disable_idma();
        }
        Ok(())
    }
    /// Program a 64-byte `CMD6` switch-status read.
    ///
    /// `CMD6` moves 512 bits of switch status, not whole 512-byte blocks, so it
    /// uses a 64-byte block on the CPU FIFO path. The next transfer programs the
    /// normal block size again through [`Self::prepare_data`].
    pub(crate) fn prepare_switch_data(&mut self, buffer: &mut [u8; 64]) -> Result<(), SdmcError> {
        // SAFETY: the register block is a valid MMIO region.
        unsafe {
            self.reg
                .block_size
                .write(self.reg.block_size.read().set_block_size(64));
            self.reg
                .block_count
                .write(self.reg.block_count.read().set_block_count(1));
        }
        self.reset_fifo()?;

        self.direction = Some(CmdTransferDir::Card2Host);
        self.command = None;
        self.buffer = buffer.as_mut_ptr();
        self.buffer_len = 64;
        self.cursor = 0;
        self.idma = false;
        self.disable_idma();
        Ok(())
    }

    /// Program `CMDARG` and start `command`.
    ///
    /// The data direction programmed by [`Self::prepare_data`] is used to set
    /// `DATA_EXP`/`TRANS_DIR`; without [`Self::prepare_data`] the command is a
    /// no-data command.
    pub(crate) fn start_command(&mut self, command: Command) -> Result<(), SdmcError> {
        let mut config = CommandConfig::zeroed();
        match command.response_kind() {
            ResponseKind::None => {}
            ResponseKind::Short => {
                config = config
                    .set_resp_expected(true)
                    .set_resp_len(CmdRespLen::ShortResp);
                if command.checks_crc() {
                    config = config.enable_check_resp_crc();
                }
            }
            ResponseKind::Long => {
                config = config
                    .set_resp_expected(true)
                    .set_resp_len(CmdRespLen::LongResp);
                if command.checks_crc() {
                    config = config.enable_check_resp_crc();
                }
            }
        }
        if let Some(direction) = self.direction {
            config = config.set_data_expected(true).set_transfer_dir(direction);
        }
        if command.is_send_init() {
            config = config.enable_send_init();
        }
        if command.is_stop() {
            // CMD12 aborts an open data transfer while the card still drives
            // the data lines, so it must not wait for a previous data phase.
            config = config.enable_send_stop();
        } else {
            config = config.enable_wait_prvdata_done();
        }
        config = config
            .set_cmd_index(command.index())
            .enable_hold_reg()
            .set_start_cmd(true);

        // SAFETY: the register block is a valid MMIO region.
        unsafe {
            self.reg.cmd_arg.write(command.argument());
            self.reg.cmd_config.write(config);
        }
        self.command = Some(command);
        Ok(())
    }

    /// Read the raw original interrupt status (`SDMC_OINTST`).
    pub(crate) fn read_status(&self) -> u32 {
        self.reg.original_int_status.read().raw()
    }

    /// Poll the command phase using the supplied interrupt status bits.
    pub(crate) fn poll_command(&mut self, status: u32) -> Poll<Result<Response, SdmcError>> {
        let status = OriginalIntStatus::from_bits(status);
        let command = match self.command {
            Some(command) => command,
            None => return Poll::Ready(Err(SdmcError::CommandTimeout)),
        };

        if status.is_interrupt_pending(SdmcInterrupt::RespTimeout) {
            return Poll::Ready(Err(SdmcError::ResponseTimeout));
        }
        if status.is_interrupt_pending(SdmcInterrupt::RespError) {
            let resp = self.reg.resp.read() as u32;
            return Poll::Ready(Err(SdmcError::ResponseStatus(resp)));
        }
        if command.checks_crc() && status.is_interrupt_pending(SdmcInterrupt::RespCrcError) {
            return Poll::Ready(Err(SdmcError::ResponseCrc));
        }
        if !status.is_interrupt_pending(SdmcInterrupt::CmdComplete) {
            return Poll::Pending;
        }

        let response = match command.response_kind() {
            ResponseKind::None => Response::None,
            ResponseKind::Short => Response::Short(self.reg.resp.read() as u32),
            ResponseKind::Long => {
                let raw = self.reg.resp.read();
                // `RESP3` (most significant) first, matching the SD R2 order.
                Response::Long([
                    (raw >> 96) as u32,
                    (raw >> 64) as u32,
                    (raw >> 32) as u32,
                    raw as u32,
                ])
            }
        };
        Poll::Ready(Ok(response))
    }

    /// Move as much PIO data as the FIFO accepts (or provides).
    ///
    /// A no-op for IDMA transfers.
    pub(crate) fn pump_data(&mut self) -> Result<(), SdmcError> {
        let direction = match (self.idma, self.direction) {
            (false, Some(direction)) => direction,
            _ => return Ok(()),
        };
        let status = self.reg.ctrl_status.read();
        match direction {
            CmdTransferDir::Card2Host => {
                let mut words = status.fifo_count() as usize;
                while words > 0 && self.cursor < self.buffer_len {
                    let word = self.reg.fifo.read();
                    let bytes = (self.buffer_len - self.cursor).min(4);
                    // SAFETY: `cursor + bytes <= buffer_len` and `buffer` points
                    // at a buffer valid for the whole data phase.
                    unsafe {
                        let dst =
                            core::slice::from_raw_parts_mut(self.buffer.add(self.cursor), bytes);
                        dst.copy_from_slice(&word.to_le_bytes()[..bytes]);
                    }
                    self.cursor += bytes;
                    words -= 1;
                }
            }
            CmdTransferDir::Host2Card => {
                let used = (status.fifo_count() as usize).min(FIFO_DEPTH_WORDS);
                let mut words = (FIFO_DEPTH_WORDS - used).min((self.buffer_len - self.cursor) / 4);
                while words > 0 {
                    // SAFETY: `cursor + 4 <= buffer_len` and `buffer` points at
                    // a buffer valid for the whole data phase.
                    let src =
                        unsafe { core::slice::from_raw_parts(self.buffer.add(self.cursor), 4) };
                    let word = u32::from_le_bytes([src[0], src[1], src[2], src[3]]);
                    // SAFETY: the register block is a valid MMIO region.
                    unsafe {
                        self.reg.fifo.write(word);
                    }
                    self.cursor += 4;
                    words -= 1;
                }
            }
        }
        Ok(())
    }

    /// Poll the data phase using the supplied interrupt status bits.
    pub(crate) fn poll_data(&mut self, status: u32) -> Poll<Result<(), SdmcError>> {
        let status = OriginalIntStatus::from_bits(status);
        if status.is_interrupt_pending(SdmcInterrupt::EndBitOrWriteNoCrc)
            || status.is_interrupt_pending(SdmcInterrupt::StartBitOrBusyClear)
            || status.is_interrupt_pending(SdmcInterrupt::HwLockError)
        {
            return Poll::Ready(Err(SdmcError::DataEndBit));
        }
        if status.is_interrupt_pending(SdmcInterrupt::FifoUndOverrun) {
            return Poll::Ready(Err(SdmcError::Fifo));
        }
        if status.is_interrupt_pending(SdmcInterrupt::DataCrcError) {
            return Poll::Ready(Err(SdmcError::DataCrc));
        }
        if status.is_interrupt_pending(SdmcInterrupt::HostTimeoutOrVSwitch)
            || status.is_interrupt_pending(SdmcInterrupt::ReadTimeout)
        {
            return Poll::Ready(Err(SdmcError::DataTimeout));
        }
        if self.idma {
            let idma = self.reg.idma_status.read();
            if idma.is_fatal_bus_error_interrupt_pending()
                || idma.is_desc_unusable_interrupt_pending()
                || idma.is_abnormal_interrupt_pending()
            {
                return Poll::Ready(Err(SdmcError::Idma));
            }
        }
        if status.is_interrupt_pending(SdmcInterrupt::DataDone) {
            Poll::Ready(Ok(()))
        } else {
            Poll::Pending
        }
    }

    /// Shut down the data path after the data phase completed.
    ///
    /// Stops the internal DMA engine and validates the FIFO byte count of a PIO
    /// transfer. Card-to-host buffers need no cache maintenance here: they are
    /// invalidated before the DMA starts.
    pub(crate) fn finish_data(&mut self) -> Result<(), SdmcError> {
        let direction = self.direction;
        let result = if self.idma {
            // SAFETY: the register block is a valid MMIO region; the buffer was
            // published to the bus master by `prepare_data`.
            unsafe {
                self.reg.host_ctrl1.modify(|v| v.disable_dma());
                self.reg
                    .host_ctrl1
                    .modify(|v| v.disable_idmac().set_dma_rst(true));
                self.reg.periph_bus_config.modify(|v| {
                    v.disable_fixed_burst_length()
                        .disable_idma()
                        .set_software_reset(true)
                });
            }
            self.clear_idma_status();
            Ok(())
        } else if direction.is_some() && self.cursor != self.buffer_len {
            // A PIO phase that did not move every byte underran the FIFO.
            Err(SdmcError::Fifo)
        } else {
            Ok(())
        };
        self.drop_data_state();
        result
    }

    /// Abort the current transaction and return the controller to idle.
    pub(crate) fn abort(&mut self) {
        self.drop_data_state();
        // SAFETY: the register block is a valid MMIO region.
        unsafe {
            self.reg.host_ctrl1.modify(|v| {
                v.set_abort_rdata(true)
                    .disable_dma()
                    .disable_idmac()
                    .set_dma_rst(true)
                    .set_fifo_reset(true)
            });
            self.reg.periph_bus_config.modify(|v| {
                v.disable_fixed_burst_length()
                    .disable_idma()
                    .set_software_reset(true)
            });
        }
        self.clear_idma_status();
        self.clear_interrupts();
        // The reset bits are self-clearing; wait for them before the next
        // transaction programs the same registers.
        for _ in 0..RESET_SPINS {
            let ctrl = self.reg.host_ctrl1.read();
            if !ctrl.fifo_reset() && !ctrl.dma_rst() {
                return;
            }
            spin_loop();
        }
    }

    /// Give back the register block and pads so another driver can take over.
    pub(crate) fn free(self) -> (&'static RegisterBlock, PADS) {
        (self.reg, self._pads)
    }

    /// Issue the self-clearing controller reset and wait for it to settle.
    fn reset_controller(&mut self) {
        // SAFETY: the register block is a valid MMIO region.
        unsafe {
            self.reg.host_ctrl1.modify(|v| {
                v.set_ctrl_reset(true)
                    .set_fifo_reset(true)
                    .set_dma_rst(true)
            });
        }
        for _ in 0..RESET_SPINS {
            let ctrl = self.reg.host_ctrl1.read();
            if !ctrl.ctrl_reset() && !ctrl.fifo_reset() && !ctrl.dma_rst() {
                return;
            }
            spin_loop();
        }
    }

    /// Program the controller defaults from the vendor `hal_sdmc_init` sequence.
    fn init_controller(&mut self) {
        // SAFETY: the register block is a valid MMIO region.
        unsafe {
            self.reg
                .original_int_status
                .write(OriginalIntStatus::zeroed().clear_all());
            self.reg.int_enable.write(IntEnable::zeroed());
            self.reg
                .timeout
                .modify(|v| v.set_data_timeout(0xFF_FFFF).set_resp_timeout(0xFF));
            self.reg.idma_int_enable.modify(|v| {
                v.disable_abnormal_interrupt()
                    .disable_normal_interrupt()
                    .disable_card_error_interrupt()
                    .disable_desc_unusable_interrupt()
                    .disable_fatal_bus_error_interrupt()
                    .disable_rx_done_interrupt()
                    .disable_tx_done_interrupt()
            });
            self.reg.periph_bus_config.modify(|v| {
                v.disable_fixed_burst_length()
                    .disable_idma()
                    .set_desc_skip_length(0)
                    .set_software_reset(true)
            });
        }
    }

    /// Program the FIFO watermarks and the card-read threshold.
    fn configure_fifo(&mut self) {
        // SAFETY: the register block is a valid MMIO region.
        unsafe {
            self.reg.fifo_config.modify(|v| {
                v.set_dma_burst_length(SdmcBurstLength::WordX8)
                    .set_fifo_rx_watermark((FIFO_DEPTH_WORDS / 2 - 1) as u16)
                    .set_fifo_tx_watermark((FIFO_DEPTH_WORDS / 2) as u16)
            });
            self.reg.card_threshold_ctrl.modify(|v| {
                v.set_card_threshold_size(BLOCK_SIZE as u16)
                    .enable_card_read_threshold()
            });
        }
    }

    /// Reset the data FIFO and wait until the self-clearing bit is cleared.
    fn reset_fifo(&mut self) -> Result<(), SdmcError> {
        // SAFETY: the register block is a valid MMIO region.
        unsafe {
            self.reg.host_ctrl1.modify(|v| v.set_fifo_reset(true));
        }
        for _ in 0..RESET_SPINS {
            if !self.reg.host_ctrl1.read().fifo_reset() {
                return Ok(());
            }
            spin_loop();
        }
        Err(SdmcError::Timeout)
    }

    /// Turn the internal DMA data path off.
    fn disable_idma(&mut self) {
        // SAFETY: the register block is a valid MMIO region.
        unsafe {
            self.reg
                .host_ctrl1
                .modify(|v| v.disable_dma().disable_idmac().set_dma_rst(true));
            self.reg.periph_bus_config.modify(|v| {
                v.disable_fixed_burst_length()
                    .disable_idma()
                    .set_software_reset(true)
            });
        }
    }

    /// Write one to every defined bit of the original interrupt status.
    fn clear_interrupts(&mut self) {
        // SAFETY: the register block is a valid MMIO region.
        unsafe {
            self.reg
                .original_int_status
                .write(OriginalIntStatus::zeroed().clear_all());
        }
    }

    /// Write one to every defined interrupt bit of the internal DMA status.
    fn clear_idma_status(&mut self) {
        // SAFETY: the register block is a valid MMIO region. The status bits of
        // the internal DMA controller are write-one-to-clear.
        unsafe {
            let status = self.reg.idma_status.read();
            self.reg.idma_status.write(status.clear_all());
        }
    }

    /// Forget the buffer/direction of the previous transaction.
    fn drop_data_state(&mut self) {
        self.direction = None;
        self.command = None;
        self.buffer = ptr::null_mut();
        self.buffer_len = 0;
        self.cursor = 0;
        self.idma = false;
    }
}

// ---------------------------------------------------------------------------
// Internal DMA (IDMAC) descriptor chain
// ---------------------------------------------------------------------------
//
// The engine walks a chain of 16-byte descriptors; each descriptor carries at
// most one 4 KiB page. The controller therefore needs one descriptor per page,
// with `CH` set on every descriptor and `LD` on the last one in the chain.
//
// DMA buffers and the descriptor table must be cache-line aligned so the driver
// can perform exactly the cache maintenance the hardware requires: clean +
// invalidate before the transfer and invalidate after a card-to-host transfer.

/// Descriptor owner bit: set by software, cleared by the DMA engine.
const OWN: u32 = 1 << 31;
/// Chained descriptor.
const CH: u32 = 1 << 4;
/// First descriptor of the chain.
const FS: u32 = 1 << 3;
/// Last descriptor of the chain.
const LD: u32 = 1 << 2;

/// Maximum bytes a single descriptor can carry (one hardware page).
pub(crate) const DESCRIPTOR_MAX_BYTES: usize = 4096;

/// Number of descriptors in a [`DescriptorTable`].
pub(crate) const MAX_DESCRIPTORS: usize = 64;

/// Maximum bytes one IDMA transaction can move.
pub(crate) const MAX_TRANSFER_BYTES: usize = MAX_DESCRIPTORS * DESCRIPTOR_MAX_BYTES;

/// One SDMC IDMAC descriptor, matching the hardware layout.
#[repr(C, align(8))]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct IdmaDescriptor {
    /// Control and status bits (`DES0`).
    pub flags: u32,
    /// Buffer size in bytes (`DES1`).
    pub count: u32,
    /// Buffer address (`DES2`).
    pub address: u32,
    /// Address of the next descriptor (`DES3`).
    pub next: u32,
}

impl IdmaDescriptor {
    const fn zeroed() -> Self {
        Self {
            flags: 0,
            count: 0,
            address: 0,
            next: 0,
        }
    }
}

/// Staging area for an IDMA descriptor chain.
///
/// The table is cache-line aligned; it must live in memory the controller can
/// reach (always true for this SoC's on-chip SRAM and DRAM).
#[repr(C, align(64))]
pub(crate) struct DescriptorTable {
    descriptors: [IdmaDescriptor; MAX_DESCRIPTORS],
}

impl DescriptorTable {
    /// Create an empty descriptor table.
    pub(crate) const fn new() -> Self {
        Self {
            descriptors: [IdmaDescriptor::zeroed(); MAX_DESCRIPTORS],
        }
    }

    /// Raw pointer to the first descriptor, for `IDMASADDR`.
    pub(crate) fn as_ptr(&self) -> *const IdmaDescriptor {
        self.descriptors.as_ptr()
    }

    /// Byte length of `count` programmed descriptors.
    pub(crate) const fn bytes(count: usize) -> usize {
        count * core::mem::size_of::<IdmaDescriptor>()
    }

    /// Build a descriptor chain for `len` bytes at `buffer`.
    ///
    /// Returns the number of descriptors used. The caller must clean the
    /// descriptor table and, for a host-to-card transfer, the buffer before
    /// starting the command, and invalidate a card-to-host buffer afterwards.
    ///
    /// # Safety
    /// `buffer` must be valid for `len` bytes of reads or writes for the whole
    /// duration of the DMA transfer.
    pub(crate) unsafe fn build(&mut self, buffer: *mut u8, len: usize) -> Result<usize, SdmcError> {
        if !is_dma_capable(buffer.cast_const(), len) {
            return Err(SdmcError::InvalidBuffer);
        }

        let count = descriptor_count(len);
        for (index, slot) in self.descriptors[..count].iter_mut().enumerate() {
            let offset = index * DESCRIPTOR_MAX_BYTES;
            let remaining = len - offset;
            let mut flags = OWN | CH;
            if index == 0 {
                flags |= FS;
            }
            if index + 1 == count {
                flags |= LD;
            }
            slot.flags = flags;
            slot.count = remaining.min(DESCRIPTOR_MAX_BYTES) as u32;
            // SAFETY: `buffer` is valid for `len` bytes and every descriptor
            // address is derived from it.
            slot.address = unsafe { buffer.add(offset) } as u32;
            slot.next = (slot as *const IdmaDescriptor as usize
                + core::mem::size_of::<IdmaDescriptor>()) as u32;
        }
        Ok(count)
    }
}

impl Default for DescriptorTable {
    fn default() -> Self {
        Self::new()
    }
}

/// Number of descriptors needed for a transfer of `len` bytes.
///
/// `len` must have been validated by [`is_dma_capable`].
pub(crate) const fn descriptor_count(len: usize) -> usize {
    len.div_ceil(DESCRIPTOR_MAX_BYTES)
}

/// Whether `len` is a whole number of 512-byte blocks within the IDMA limit.
pub(crate) const fn is_dma_length(len: usize) -> bool {
    len != 0 && len.is_multiple_of(BLOCK_SIZE) && len <= MAX_TRANSFER_BYTES
}

/// Whether a buffer pointer is cache-line aligned for the internal DMA engine.
pub(crate) fn is_dma_aligned(buffer: *const u8) -> bool {
    (buffer as usize).is_multiple_of(CACHE_LINE)
}

/// Whether a whole buffer can be handed to the internal DMA engine in one go.
///
/// The SDMC bus master requires a cache-line aligned, block-multiple buffer no
/// larger than [`MAX_TRANSFER_BYTES`]. A larger aligned buffer is valid as well
/// (the protocol engine splits it into commands), so callers that drive chunked
/// transfers only need [`is_dma_aligned`]. DMA addresses are 32-bit on these
/// SoCs; the address itself is not range-checked here because the host build of
/// the unit tests uses addresses above 4 GiB.
pub(crate) fn is_dma_capable(buffer: *const u8, len: usize) -> bool {
    is_dma_length(len) && is_dma_aligned(buffer)
}

#[cfg(test)]
mod descriptor_tests {
    use super::*;
    use crate::types::DmaBuffer;

    #[test]
    fn single_page_chain() {
        let mut buffer = DmaBuffer::<4096>::new();
        let mut table = DescriptorTable::new();
        let count = unsafe { table.build(buffer.as_mut_slice().as_mut_ptr(), 512) }.unwrap();
        assert_eq!(count, 1);
        let desc = &table.descriptors[0];
        assert_eq!(desc.flags, OWN | CH | FS | LD);
        assert_eq!(desc.count, 512);
        assert_eq!(desc.address, buffer.as_slice().as_ptr() as u32);
    }

    #[test]
    fn multi_page_chain_links_descriptors() {
        let mut buffer = DmaBuffer::<8192>::new();
        let mut table = DescriptorTable::new();
        let count = unsafe { table.build(buffer.as_mut_slice().as_mut_ptr(), 8192) }.unwrap();
        assert_eq!(count, 2);

        let first = &table.descriptors[0];
        assert_eq!(first.flags, OWN | CH | FS);
        assert_eq!(first.count, 4096);
        assert_eq!(first.address, buffer.as_slice().as_ptr() as u32);
        assert_eq!(first.next, &table.descriptors[1] as *const _ as u32);

        let last = &table.descriptors[1];
        assert_eq!(last.flags, OWN | CH | LD);
        assert_eq!(last.count, 4096);
        assert_eq!(
            last.address,
            unsafe { buffer.as_slice().as_ptr().add(4096) } as u32
        );
    }

    #[test]
    fn partial_last_descriptor() {
        let mut buffer = DmaBuffer::<8192>::new();
        let mut table = DescriptorTable::new();
        let count =
            unsafe { table.build(buffer.as_mut_slice().as_mut_ptr(), 4096 + 1024) }.unwrap();
        assert_eq!(count, 2);
        assert_eq!(table.descriptors[0].count, 4096);
        assert_eq!(table.descriptors[1].count, 1024);
    }

    #[test]
    fn rejects_invalid_lengths() {
        let mut buffer = DmaBuffer::<4096>::new();
        let mut table = DescriptorTable::new();
        unsafe {
            assert_eq!(
                table.build(buffer.as_mut_slice().as_mut_ptr(), 0),
                Err(SdmcError::InvalidBuffer)
            );
            assert_eq!(
                table.build(buffer.as_mut_slice().as_mut_ptr(), 1000),
                Err(SdmcError::InvalidBuffer)
            );
        }
    }

    #[test]
    fn rejects_unaligned_buffer() {
        let mut buffer = DmaBuffer::<4096>::new();
        let mut table = DescriptorTable::new();
        unsafe {
            assert_eq!(
                table.build(buffer.as_mut_slice().as_mut_ptr().add(4), 512),
                Err(SdmcError::InvalidBuffer)
            );
        }
    }

    #[test]
    fn rejects_too_large_transfer() {
        let mut buffer = DmaBuffer::<{ MAX_TRANSFER_BYTES + 4096 }>::new();
        let mut table = DescriptorTable::new();
        unsafe {
            assert_eq!(
                table.build(
                    buffer.as_mut_slice().as_mut_ptr(),
                    MAX_TRANSFER_BYTES + 4096
                ),
                Err(SdmcError::InvalidBuffer)
            );
        }
    }

    #[test]
    fn descriptor_count_for_sizes() {
        assert_eq!(descriptor_count(512), 1);
        assert_eq!(descriptor_count(4096), 1);
        assert_eq!(descriptor_count(4097 * 2), 3);
        assert_eq!(descriptor_count(MAX_TRANSFER_BYTES), MAX_DESCRIPTORS);
    }

    #[test]
    fn dma_length_limits() {
        assert!(is_dma_length(512));
        assert!(is_dma_length(MAX_TRANSFER_BYTES));
        assert!(!is_dma_length(0));
        assert!(!is_dma_length(513));
        assert!(!is_dma_length(MAX_TRANSFER_BYTES + 512));
    }

    #[test]
    fn dma_alignment_is_checked_separately_from_length() {
        let buffer = DmaBuffer::<{ MAX_TRANSFER_BYTES + 4096 }>::new();
        let pointer = buffer.as_slice().as_ptr();
        // A large aligned buffer is valid for the chunked path even though it
        // does not fit one command.
        assert!(is_dma_aligned(pointer));
        assert!(!is_dma_capable(pointer, MAX_TRANSFER_BYTES + 4096));
        assert!(!is_dma_aligned(unsafe { pointer.add(4) }));
    }
}
