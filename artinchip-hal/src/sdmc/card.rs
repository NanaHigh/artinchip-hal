//! SD card protocol engine.
//!
//! [`Card`] implements identification, CSD/CID decoding and block-transfer
//! commands over [`SdHost`]/[`SdHostAsync`], so both drivers share one protocol
//! implementation that is testable with a mock host.
//!
//! `CMD0 -> CMD8 -> CMD55/ACMD41 loop -> CMD2/3/9/7/16 -> ACMD6 (4-bit) ->
//! bus width -> clock -> CMD13`; only a failed `CMD8` or `ACMD6` is recoverable.
//! Reads use up to [`MAX_BLOCKS_PER_TRANSFER`] blocks per command and writes the
//! smaller [`MAX_WRITE_BLOCKS_PER_TRANSFER`] chunk (larger writes were unreliable
//! on the reference boards). Validated on D13x/D21x; unit tests cover the state
//! machine with a mock host.

use embedded_hal::delay::DelayNs;
use embedded_time::rate::Hertz;

use super::register::BusWidth;
use super::{
    BLOCK_SIZE, BLOCK_SIZE_U32, CardInfo, MAX_WRITE_BLOCKS_PER_TRANSFER, SdmcConfig, SdmcError,
};

/// Maximum number of 512-byte blocks handled by one read command chunk.
///
/// One chunk is one IDMA descriptor chain, so the limit follows
/// [`MAX_TRANSFER_BYTES`](super::bus::MAX_TRANSFER_BYTES). Writes use the
/// smaller [`MAX_WRITE_BLOCKS_PER_TRANSFER`] chunk limit.
pub(crate) const MAX_BLOCKS_PER_TRANSFER: u16 =
    (super::bus::MAX_TRANSFER_BYTES / BLOCK_SIZE) as u16;

/// Power-up delay before the first command, in milliseconds.
const POWER_UP_DELAY_MS: u32 = 10;

/// Delay between two `ACMD41` attempts, in milliseconds.
const ACMD41_RETRY_DELAY_MS: u32 = 1;

/// Number of `ACMD41` attempts before identification gives up.
const MAX_ACMD41_ATTEMPTS: u32 = 1000;

/// `OCR` bit 31: the card finished its power-up initialization.
const OCR_READY: u32 = 1 << 31;

/// `OCR` bit 30: `HCS` in the `ACMD41` argument, `CCS` in the response.
const OCR_HCS: u32 = 1 << 30;

/// `OCR` voltage window bits for 2.7 V to 3.6 V.
const OCR_VOLTAGE_WINDOW: u32 = 0x00FF_8000;

/// `CMD8` argument check pattern.
const CMD8_CHECK_PATTERN: u32 = 0x1AA;

/// Mask of the check pattern echoed in the `CMD8` response.
const CMD8_PATTERN_MASK: u32 = 0xFFF;

/// `ACMD6` argument selecting four data lines.
const ACMD6_FOUR_BIT: u32 = 2;

/// Bits of an `R1` response that signal a card-side error.
///
/// The reserved bits (18:17) and the state/status field (15:0) are valid after a
/// successful command and are not treated as errors.
const R1_ERROR_MASK: u32 = 0xFFF9_0000;

/// Bit position of the RCA in a `CMD3`/`R6` response.
const RCA_SHIFT: u32 = 16;

// ---------------------------------------------------------------------------
// Host abstraction
// ---------------------------------------------------------------------------

/// Response shape expected for a command.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum ResponseKind {
    None,
    Short,
    Long,
}

/// One SD command.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct Command {
    index: u8,
    argument: u32,
    response: ResponseKind,
    crc: bool,
    send_init: bool,
    stop: bool,
}

impl Command {
    /// Build a command with CRC checking enabled for responses that have a CRC.
    pub const fn new(index: u8, argument: u32, response: ResponseKind) -> Self {
        Self {
            index,
            argument,
            response,
            crc: !matches!(response, ResponseKind::None),
            send_init: false,
            stop: false,
        }
    }

    /// Disable response CRC checking, as required for the `R3` response of
    /// `ACMD41`.
    pub const fn without_crc(mut self) -> Self {
        self.crc = false;
        self
    }

    /// Send the 80-clock initialization sequence before the command (`CMD0`).
    pub const fn send_init(mut self) -> Self {
        self.send_init = true;
        self
    }

    /// Mark this command as a stop-transmission command (`CMD12`).
    pub const fn stop(mut self) -> Self {
        self.stop = true;
        self
    }

    pub const fn index(&self) -> u8 {
        self.index
    }

    pub const fn argument(&self) -> u32 {
        self.argument
    }

    pub const fn response_kind(&self) -> ResponseKind {
        self.response
    }

    pub const fn checks_crc(&self) -> bool {
        self.crc
    }

    pub const fn is_send_init(&self) -> bool {
        self.send_init
    }

    pub const fn is_stop(&self) -> bool {
        self.stop
    }
}

/// Response returned by a host implementation.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Response {
    None,
    Short(u32),
    Long([u32; 4]),
}

/// Data phase passed to a host implementation.
#[derive(Debug)]
pub(crate) enum Data<'a> {
    None,
    Read(&'a mut [u8]),
    Write(&'a [u8]),
}

impl Data<'_> {
    /// Buffer length in bytes (zero for [`Data::None`]).
    pub const fn len(&self) -> usize {
        match self {
            Self::None => 0,
            Self::Read(buffer) => buffer.len(),
            Self::Write(buffer) => buffer.len(),
        }
    }

    pub const fn is_none(&self) -> bool {
        matches!(self, Self::None)
    }

    pub const fn blocks(&self) -> usize {
        self.len() / BLOCK_SIZE
    }
}

/// Blocking host: owns the controller registers, data path, clock and bus width.
pub(crate) trait SdHost {
    /// Execute one command including its data phase (and the stop command).
    fn execute(&mut self, command: Command, data: Data<'_>) -> Result<Response, SdmcError>;

    /// Program the controller bus width.
    fn configure_bus(&mut self, width: BusWidth);

    /// Program the card clock.
    fn configure_clock(&mut self, clock: Hertz);
}

/// Asynchronous, interrupt-driven host implementation.
#[cfg(feature = "interrupts")]
#[allow(async_fn_in_trait)]
pub(crate) trait SdHostAsync {
    /// Execute one command including its data phase over the internal DMA.
    async fn execute(&mut self, command: Command, data: Data<'_>) -> Result<Response, SdmcError>;

    /// Program the controller bus width.
    fn configure_bus(&mut self, width: BusWidth);

    /// Program the card clock.
    fn configure_clock(&mut self, clock: Hertz);
}

// ---------------------------------------------------------------------------
// Identification state machine
// ---------------------------------------------------------------------------

/// One step produced by the identification state machine.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum InitStep {
    Command(Command),
    DelayMs(u32),
    BusWidth(BusWidth),
    Clock(Hertz),
    Complete(CardInfo),
    Failed(SdmcError),
}

/// SD interface version detected by the `CMD8` probe.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Version {
    V1,
    V2,
}

/// Current identification state.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum State {
    Idle,
    PowerUpDelay,
    Cmd0,
    Cmd8,
    Acmd41Cmd55,
    Acmd41Cmd41,
    Acmd41Delay,
    Cmd2,
    Cmd3,
    Cmd9,
    Cmd7,
    Cmd16,
    Acmd6Cmd55,
    Acmd6,
    BusWidth,
    Clock,
    Cmd13,
    Complete,
    Failed(SdmcError),
}

/// SD card protocol engine.
pub(crate) struct Card {
    config: SdmcConfig,
    state: State,
    info: Option<CardInfo>,
    ocr: u32,
    rca: u16,
    cid: [u32; 4],
    csd: [u32; 4],
    version: Version,
    attempts: u32,
    negotiated_bus_width: BusWidth,
    high_capacity: bool,
    blocks: u32,
    read_block_len: usize,
}

impl Card {
    /// Create a protocol engine for the given configuration.
    pub const fn new(config: SdmcConfig) -> Self {
        Self {
            config,
            state: State::Idle,
            info: None,
            ocr: 0,
            rca: 0,
            cid: [0; 4],
            csd: [0; 4],
            version: Version::V1,
            attempts: 0,
            negotiated_bus_width: config.bus_width,
            high_capacity: false,
            blocks: 0,
            read_block_len: BLOCK_SIZE,
        }
    }

    /// Information of the identified card.
    pub const fn info(&self) -> Option<CardInfo> {
        self.info
    }

    /// Record the clock the driver switched to after identification.
    pub(crate) fn set_clock(&mut self, clock: Hertz) {
        self.config.clock = clock;
        if let Some(info) = self.info.as_mut() {
            info.clock = clock;
        }
    }

    /// Start (or restart) the identification sequence.
    pub fn start_init(&mut self) -> InitStep {
        self.state = State::PowerUpDelay;
        self.info = None;
        self.ocr = 0;
        self.rca = 0;
        self.cid = [0; 4];
        self.csd = [0; 4];
        self.version = Version::V1;
        self.attempts = 0;
        self.negotiated_bus_width = self.config.bus_width;
        self.high_capacity = false;
        self.blocks = 0;
        self.read_block_len = BLOCK_SIZE;
        InitStep::DelayMs(POWER_UP_DELAY_MS)
    }

    /// Feed the response of the last [`InitStep::Command`] back into the
    /// state machine.
    pub fn on_response(&mut self, response: Response) -> InitStep {
        match self.state {
            State::Cmd0 => {
                // The card is in idle state; probe for SD v2 support.
                self.state = State::Cmd8;
                InitStep::Command(Command::new(8, CMD8_CHECK_PATTERN, ResponseKind::Short))
            }
            State::Cmd8 => {
                // SD v2 cards echo the check pattern and accept 2.7-3.6 V.
                let is_v2 = matches!(
                    response,
                    Response::Short(word) if word & CMD8_PATTERN_MASK == CMD8_CHECK_PATTERN
                );
                self.version = if is_v2 { Version::V2 } else { Version::V1 };
                self.begin_acmd41()
            }
            State::Acmd41Cmd55 => match Self::check_r1(response) {
                Ok(()) => {
                    self.attempts += 1;
                    self.state = State::Acmd41Cmd41;
                    InitStep::Command(self.acmd41_command())
                }
                Err(error) => self.fail(error),
            },
            State::Acmd41Cmd41 => match response {
                Response::Short(word) => {
                    self.ocr = word;
                    if word & OCR_READY != 0 {
                        // The CCS bit is only meaningful for SD v2 cards.
                        self.high_capacity = self.version == Version::V2 && (word & OCR_HCS != 0);
                        self.state = State::Cmd2;
                        InitStep::Command(Command::new(2, 0, ResponseKind::Long))
                    } else if self.attempts >= MAX_ACMD41_ATTEMPTS {
                        self.fail(SdmcError::Timeout)
                    } else {
                        self.state = State::Acmd41Delay;
                        InitStep::DelayMs(ACMD41_RETRY_DELAY_MS)
                    }
                }
                _ => self.fail(SdmcError::UnsupportedCard),
            },
            State::Cmd2 => match response {
                Response::Long(words) => {
                    self.cid = words;
                    self.state = State::Cmd3;
                    InitStep::Command(Command::new(3, 0, ResponseKind::Short))
                }
                _ => self.fail(SdmcError::UnsupportedCard),
            },
            State::Cmd3 => match Self::expect_short(response) {
                Ok(word) => {
                    // The R6 response carries the RCA in its upper half; a
                    // zero RCA means the card rejected the selection.
                    let rca = (word >> RCA_SHIFT) as u16;
                    if rca == 0 {
                        self.fail(SdmcError::UnsupportedCard)
                    } else {
                        self.rca = rca;
                        self.state = State::Cmd9;
                        InitStep::Command(Command::new(9, self.rca_argument(), ResponseKind::Long))
                    }
                }
                Err(error) => self.fail(error),
            },
            State::Cmd9 => match response {
                Response::Long(words) => {
                    self.csd = words;
                    match Self::decode_csd(&words) {
                        Ok((blocks, read_block_len)) => {
                            self.blocks = blocks;
                            self.read_block_len = read_block_len;
                            self.state = State::Cmd7;
                            InitStep::Command(Command::new(
                                7,
                                self.rca_argument(),
                                ResponseKind::Short,
                            ))
                        }
                        Err(error) => self.fail(error),
                    }
                }
                _ => self.fail(SdmcError::UnsupportedCard),
            },
            State::Cmd7 => match Self::check_r1(response) {
                Ok(()) => {
                    self.state = State::Cmd16;
                    InitStep::Command(Command::new(16, BLOCK_SIZE_U32, ResponseKind::Short))
                }
                Err(error) => self.fail(error),
            },
            State::Cmd16 => match Self::check_r1(response) {
                Ok(()) => {
                    if self.config.bus_width == BusWidth::FourWire {
                        self.state = State::Acmd6Cmd55;
                        InitStep::Command(Command::new(
                            55,
                            self.rca_argument(),
                            ResponseKind::Short,
                        ))
                    } else {
                        self.negotiated_bus_width = BusWidth::OneWire;
                        self.state = State::BusWidth;
                        InitStep::BusWidth(BusWidth::OneWire)
                    }
                }
                Err(error) => self.fail(error),
            },
            State::Acmd6Cmd55 => match Self::check_r1(response) {
                Ok(()) => {
                    self.state = State::Acmd6;
                    InitStep::Command(Command::new(6, ACMD6_FOUR_BIT, ResponseKind::Short))
                }
                // A card that refuses the APP_CMD prefix stays on one line.
                Err(_) => self.bus_width_fallback(),
            },
            State::Acmd6 => match Self::check_r1(response) {
                Ok(()) => {
                    self.negotiated_bus_width = BusWidth::FourWire;
                    self.state = State::BusWidth;
                    InitStep::BusWidth(BusWidth::FourWire)
                }
                Err(_) => self.bus_width_fallback(),
            },
            State::Cmd13 => match Self::check_r1(response) {
                Ok(()) => {
                    let info = CardInfo {
                        rca: self.rca,
                        high_capacity: self.high_capacity,
                        blocks: self.blocks,
                        read_block_len: self.read_block_len,
                        bus_width: self.negotiated_bus_width,
                        clock: self.config.clock,
                        cid: self.cid,
                        csd: self.csd,
                    };
                    self.info = Some(info);
                    self.state = State::Complete;
                    InitStep::Complete(info)
                }
                Err(error) => self.fail(error),
            },
            _ => self.fail(SdmcError::NotInitialized),
        }
    }

    /// Feed a command error back into the state machine. Only the `CMD8` probe
    /// and the `ACMD6` bus-width switch are recoverable.
    pub fn on_error(&mut self, error: SdmcError) -> InitStep {
        match self.state {
            // A card that does not answer CMD8 is treated as SD v1.
            State::Cmd8 => {
                self.version = Version::V1;
                self.begin_acmd41()
            }
            // A failed bus-width switch leaves the card on one data line.
            State::Acmd6Cmd55 | State::Acmd6 => self.bus_width_fallback(),
            _ => self.fail(error),
        }
    }

    /// Continue after an [`InitStep::DelayMs`].
    pub fn on_delay_elapsed(&mut self) -> InitStep {
        match self.state {
            State::PowerUpDelay => {
                // CMD0 resets every card on the bus into idle state; it also
                // asks the host to emit the 80-clock initialization sequence.
                self.state = State::Cmd0;
                InitStep::Command(Command::new(0, 0, ResponseKind::None).send_init())
            }
            State::Acmd41Delay => self.begin_acmd41(),
            _ => self.fail(SdmcError::NotInitialized),
        }
    }

    /// Continue after the host applied an [`InitStep::BusWidth`].
    pub fn on_bus_width_applied(&mut self) -> InitStep {
        match self.state {
            State::BusWidth => {
                self.state = State::Clock;
                InitStep::Clock(self.config.clock)
            }
            _ => self.fail(SdmcError::NotInitialized),
        }
    }

    /// Continue after the host applied an [`InitStep::Clock`].
    pub fn on_clock_applied(&mut self) -> InitStep {
        match self.state {
            State::Clock => {
                self.state = State::Cmd13;
                InitStep::Command(Command::new(13, self.rca_argument(), ResponseKind::Short))
            }
            _ => self.fail(SdmcError::NotInitialized),
        }
    }

    /// Run the identification sequence to completion on a blocking host.
    pub fn init_blocking<H, D>(
        &mut self,
        host: &mut H,
        delay: &mut D,
    ) -> Result<CardInfo, SdmcError>
    where
        H: SdHost,
        D: DelayNs,
    {
        let mut step = self.start_init();
        loop {
            step = match step {
                InitStep::Command(command) => match host.execute(command, Data::None) {
                    Ok(response) => self.on_response(response),
                    Err(error) => self.on_error(error),
                },
                InitStep::DelayMs(ms) => {
                    delay.delay_ms(ms);
                    self.on_delay_elapsed()
                }
                InitStep::BusWidth(width) => {
                    host.configure_bus(width);
                    self.on_bus_width_applied()
                }
                InitStep::Clock(clock) => {
                    host.configure_clock(clock);
                    self.on_clock_applied()
                }
                InitStep::Complete(info) => return Ok(info),
                InitStep::Failed(error) => return Err(error),
            };
        }
    }

    /// Run the identification sequence to completion on an interrupt-driven
    /// host.
    ///
    /// The short specification delays between `ACMD41` attempts use the
    /// blocking `delay`; every command itself is awaited asynchronously.
    #[cfg(feature = "interrupts")]
    pub async fn init_async<H, D>(
        &mut self,
        host: &mut H,
        delay: &mut D,
    ) -> Result<CardInfo, SdmcError>
    where
        H: SdHostAsync,
        D: DelayNs,
    {
        let mut step = self.start_init();
        loop {
            step = match step {
                InitStep::Command(command) => match host.execute(command, Data::None).await {
                    Ok(response) => self.on_response(response),
                    Err(error) => self.on_error(error),
                },
                InitStep::DelayMs(ms) => {
                    delay.delay_ms(ms);
                    self.on_delay_elapsed()
                }
                InitStep::BusWidth(width) => {
                    host.configure_bus(width);
                    self.on_bus_width_applied()
                }
                InitStep::Clock(clock) => {
                    host.configure_clock(clock);
                    self.on_clock_applied()
                }
                InitStep::Complete(info) => return Ok(info),
                InitStep::Failed(error) => return Err(error),
            };
        }
    }

    /// Read `buffer.len()` bytes starting at 512-byte block `start`.
    pub fn read_blocks<H>(
        &mut self,
        host: &mut H,
        start: u32,
        buffer: &mut [u8],
    ) -> Result<(), SdmcError>
    where
        H: SdHost,
    {
        if self.transfer_bounds(start, buffer.len())? == 0 {
            return Ok(());
        }

        let chunk_bytes = MAX_BLOCKS_PER_TRANSFER as usize * BLOCK_SIZE;
        let mut offset = 0;
        while offset < buffer.len() {
            let length = core::cmp::min(chunk_bytes, buffer.len() - offset);
            let chunk = &mut buffer[offset..offset + length];
            let block = (start as u64 + (offset / BLOCK_SIZE) as u64) as u32;
            let command = self.read_command(block, (length / BLOCK_SIZE) as u16)?;
            let response = host.execute(command, Data::Read(chunk))?;
            Self::check_r1(response)?;
            offset += length;
        }
        Ok(())
    }

    /// Write `buffer.len()` bytes starting at 512-byte block `start`.
    ///
    /// A write is split into chunks of at most [`MAX_WRITE_BLOCKS_PER_TRANSFER`]
    /// blocks: larger multi-block writes were not reliable on the reference
    /// boards.
    pub fn write_blocks<H>(
        &mut self,
        host: &mut H,
        start: u32,
        buffer: &[u8],
    ) -> Result<(), SdmcError>
    where
        H: SdHost,
    {
        if self.transfer_bounds(start, buffer.len())? == 0 {
            return Ok(());
        }

        let chunk_bytes = MAX_WRITE_BLOCKS_PER_TRANSFER as usize * BLOCK_SIZE;
        let mut offset = 0;
        while offset < buffer.len() {
            let length = core::cmp::min(chunk_bytes, buffer.len() - offset);
            let chunk = &buffer[offset..offset + length];
            let block = (start as u64 + (offset / BLOCK_SIZE) as u64) as u32;
            let command = self.write_command(block, (length / BLOCK_SIZE) as u16)?;
            let response = host.execute(command, Data::Write(chunk))?;
            Self::check_r1(response)?;
            offset += length;
        }
        Ok(())
    }

    /// Asynchronous [`Self::read_blocks`].
    #[cfg(feature = "interrupts")]
    pub async fn read_blocks_async<H>(
        &mut self,
        host: &mut H,
        start: u32,
        buffer: &mut [u8],
    ) -> Result<(), SdmcError>
    where
        H: SdHostAsync,
    {
        if self.transfer_bounds(start, buffer.len())? == 0 {
            return Ok(());
        }

        let chunk_bytes = MAX_BLOCKS_PER_TRANSFER as usize * BLOCK_SIZE;
        let mut offset = 0;
        while offset < buffer.len() {
            let length = core::cmp::min(chunk_bytes, buffer.len() - offset);
            let chunk = &mut buffer[offset..offset + length];
            let block = (start as u64 + (offset / BLOCK_SIZE) as u64) as u32;
            let command = self.read_command(block, (length / BLOCK_SIZE) as u16)?;
            let response = host.execute(command, Data::Read(chunk)).await?;
            Self::check_r1(response)?;
            offset += length;
        }
        Ok(())
    }

    /// Asynchronous [`Self::write_blocks`].
    #[cfg(feature = "interrupts")]
    pub async fn write_blocks_async<H>(
        &mut self,
        host: &mut H,
        start: u32,
        buffer: &[u8],
    ) -> Result<(), SdmcError>
    where
        H: SdHostAsync,
    {
        if self.transfer_bounds(start, buffer.len())? == 0 {
            return Ok(());
        }

        let chunk_bytes = MAX_WRITE_BLOCKS_PER_TRANSFER as usize * BLOCK_SIZE;
        let mut offset = 0;
        while offset < buffer.len() {
            let length = core::cmp::min(chunk_bytes, buffer.len() - offset);
            let chunk = &buffer[offset..offset + length];
            let block = (start as u64 + (offset / BLOCK_SIZE) as u64) as u32;
            let command = self.write_command(block, (length / BLOCK_SIZE) as u16)?;
            let response = host.execute(command, Data::Write(chunk)).await?;
            Self::check_r1(response)?;
            offset += length;
        }
        Ok(())
    }

    /// Build the read command for one chunk of 1..=[`MAX_BLOCKS_PER_TRANSFER`]
    /// blocks.
    pub fn read_command(&self, start: u32, blocks: u16) -> Result<Command, SdmcError> {
        self.transfer_command(start, blocks, true)
    }

    /// Build the write command for one chunk of 1..=[`MAX_WRITE_BLOCKS_PER_TRANSFER`]
    /// blocks.
    pub fn write_command(&self, start: u32, blocks: u16) -> Result<Command, SdmcError> {
        self.transfer_command(start, blocks, false)
    }

    /// Translate a block index into the SD command argument.
    ///
    /// Block-addressed cards (SDHC/SDXC) take the block index directly; byte
    /// addressed cards take `block * 512`.
    pub fn block_address(&self, block: u32) -> Result<u32, SdmcError> {
        let info = self.info.ok_or(SdmcError::NotInitialized)?;
        if info.high_capacity {
            Ok(block)
        } else {
            block.checked_mul(BLOCK_SIZE_U32).ok_or(SdmcError::Overflow)
        }
    }

    /// Validate an `R1` response: return [`SdmcError::ResponseStatus`] if any
    /// error status bit is set.
    ///
    /// A response that is not a short response is rejected with
    /// [`SdmcError::UnsupportedCard`], because the card did not return the
    /// status word the command asks for.
    pub fn check_r1(response: Response) -> Result<(), SdmcError> {
        let status = Self::expect_short(response)?;
        if status & R1_ERROR_MASK != 0 {
            Err(SdmcError::ResponseStatus(status))
        } else {
            Ok(())
        }
    }

    /// Build a read or write command for one chunk.
    fn transfer_command(&self, start: u32, blocks: u16, read: bool) -> Result<Command, SdmcError> {
        let limit = if read {
            MAX_BLOCKS_PER_TRANSFER
        } else {
            MAX_WRITE_BLOCKS_PER_TRANSFER
        };
        if blocks == 0 || blocks > limit {
            return Err(SdmcError::InvalidBlock);
        }
        let info = self.info.ok_or(SdmcError::NotInitialized)?;
        if start as u64 + blocks as u64 > info.blocks as u64 {
            return Err(SdmcError::OutOfRange);
        }
        let argument = self.block_address(start)?;
        let index = match (read, blocks) {
            (true, 1) => 17,
            (true, _) => 18,
            (false, 1) => 24,
            (false, _) => 25,
        };
        Ok(Command::new(index, argument, ResponseKind::Short))
    }

    /// Validate a whole transfer and return its block count.
    fn transfer_bounds(&self, start: u32, length: usize) -> Result<u32, SdmcError> {
        if !length.is_multiple_of(BLOCK_SIZE) {
            return Err(SdmcError::InvalidBlock);
        }
        if length == 0 {
            return Ok(0);
        }
        let info = self.info.ok_or(SdmcError::NotInitialized)?;
        let blocks = length / BLOCK_SIZE;
        if start as u64 + blocks as u64 > info.blocks as u64 {
            return Err(SdmcError::OutOfRange);
        }
        u32::try_from(blocks).map_err(|_| SdmcError::Overflow)
    }

    /// Feed an unrecoverable command error into the state machine.
    fn fail(&mut self, error: SdmcError) -> InitStep {
        self.state = State::Failed(error);
        InitStep::Failed(error)
    }

    /// Continue on one data line after a failed bus-width switch.
    fn bus_width_fallback(&mut self) -> InitStep {
        self.negotiated_bus_width = BusWidth::OneWire;
        self.state = State::BusWidth;
        InitStep::BusWidth(BusWidth::OneWire)
    }

    /// Send the `CMD55` that precedes the next `ACMD41`.
    fn begin_acmd41(&mut self) -> InitStep {
        self.state = State::Acmd41Cmd55;
        InitStep::Command(Command::new(55, self.rca_argument(), ResponseKind::Short))
    }

    /// `ACMD41` asking the card to finish initialization.
    ///
    /// `HCS` tells an SD v2 card that the host supports block addressing; the
    /// response is an `R3` without CRC.
    fn acmd41_command(&self) -> Command {
        let hcs = if self.version == Version::V2 {
            OCR_HCS
        } else {
            0
        };
        Command::new(41, OCR_VOLTAGE_WINDOW | hcs, ResponseKind::Short).without_crc()
    }

    /// RCA field of a command argument.
    const fn rca_argument(&self) -> u32 {
        (self.rca as u32) << RCA_SHIFT
    }

    /// Extract the status word of a short response.
    fn expect_short(response: Response) -> Result<u32, SdmcError> {
        match response {
            Response::Short(word) => Ok(word),
            _ => Err(SdmcError::UnsupportedCard),
        }
    }

    /// Decode the capacity and read block length from a CSD response.
    ///
    /// Returns the capacity in 512-byte blocks and the read block length capped
    /// at [`BLOCK_SIZE`].
    fn decode_csd(csd: &[u32; 4]) -> Result<(u32, usize), SdmcError> {
        // CSD words are most significant first: word3 holds bits 127:96.
        let [word3, word2, word1, _] = *csd;
        let structure = (word3 >> 30) & 0x3;
        match structure {
            // CSD structure 0: byte addressed cards. C_SIZE is bits 73:62,
            // C_SIZE_MULT bits 49:47 and READ_BL_LEN bits 83:80.
            0 => {
                let c_size = (((word2 & 0x3FF) << 2) | (word1 >> 30)) as u64;
                let c_size_mult = (word1 >> 15) & 0x7;
                let read_bl_len = (word2 >> 16) & 0xF;
                let capacity = (c_size + 1) << (c_size_mult + 2 + read_bl_len);
                let blocks = capacity / BLOCK_SIZE as u64;
                if blocks == 0 {
                    return Err(SdmcError::UnsupportedCard);
                }
                if blocks > u32::MAX as u64 {
                    return Err(SdmcError::Overflow);
                }
                let read_block_len = (1usize << read_bl_len).min(BLOCK_SIZE);
                Ok((blocks as u32, read_block_len))
            }
            // CSD structure 1: block addressed cards with 512-byte blocks.
            // C_SIZE is bits 69:48 and the capacity is (c_size + 1) * 512 KiB.
            1 => {
                let c_size = (((word2 & 0x3F) << 16) | (word1 >> 16)) as u64;
                let blocks = (c_size + 1) * (2 * BLOCK_SIZE as u64);
                if blocks == 0 {
                    return Err(SdmcError::UnsupportedCard);
                }
                if blocks > u32::MAX as u64 {
                    return Err(SdmcError::Overflow);
                }
                Ok((blocks as u32, BLOCK_SIZE))
            }
            _ => Err(SdmcError::UnsupportedCard),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const CID: [u32; 4] = [0x0012_3456, 0x789A_BCDE, 0xF012_3456, 0x789A_BCDE];

    fn csd_structure1(c_size: u32) -> [u32; 4] {
        let word3 = 1 << 30;
        let word2 = (c_size >> 16) & 0x3F;
        let word1 = (c_size & 0xFFFF) << 16;
        [word3, word2, word1, 0]
    }

    fn csd_structure0(c_size: u32, c_size_mult: u32, read_bl_len: u32) -> [u32; 4] {
        let word3 = 0;
        let word2 = ((read_bl_len & 0xF) << 16) | ((c_size >> 2) & 0x3FF);
        let word1 = ((c_size & 0x3) << 30) | ((c_size_mult & 0x7) << 15);
        [word3, word2, word1, 0]
    }

    struct MockHost {
        index_counts: [u32; 64],
        arguments: [u32; 64],
        order: [u8; 64],
        order_len: usize,
        read_lengths: [usize; 8],
        read_count: usize,
        write_lengths: [usize; 8],
        write_count: usize,
        cmd8: Result<Response, SdmcError>,
        acmd6: Result<Response, SdmcError>,
        cmd41_ready_after: u32,
        cmd41_count: u32,
        r1_error: Option<u32>,
        csd: [u32; 4],
        bus_width: Option<BusWidth>,
        clock: Option<Hertz>,
    }

    impl MockHost {
        fn new(csd: [u32; 4]) -> Self {
            Self {
                index_counts: [0; 64],
                arguments: [0; 64],
                order: [0; 64],
                order_len: 0,
                read_lengths: [0; 8],
                read_count: 0,
                write_lengths: [0; 8],
                write_count: 0,
                cmd8: Ok(Response::Short(CMD8_CHECK_PATTERN)),
                acmd6: Ok(Response::Short(0)),
                cmd41_ready_after: 0,
                cmd41_count: 0,
                r1_error: None,
                csd,
                bus_width: None,
                clock: None,
            }
        }

        fn count(&self, index: u8) -> u32 {
            self.index_counts[index as usize]
        }

        fn argument(&self, index: u8) -> u32 {
            self.arguments[index as usize]
        }

        fn clear_log(&mut self) {
            self.index_counts = [0; 64];
            self.arguments = [0; 64];
            self.order_len = 0;
            self.read_count = 0;
            self.write_count = 0;
        }

        fn run(&mut self, command: Command, data: Data<'_>) -> Result<Response, SdmcError> {
            let index = command.index();
            self.index_counts[index as usize] += 1;
            self.arguments[index as usize] = command.argument();
            if self.order_len < self.order.len() {
                self.order[self.order_len] = index;
                self.order_len += 1;
            }

            match data {
                Data::Read(buffer) => {
                    assert_eq!(buffer.len() % BLOCK_SIZE, 0);
                    if self.read_count < self.read_lengths.len() {
                        self.read_lengths[self.read_count] = buffer.len();
                        self.read_count += 1;
                    }
                    buffer.fill(command.argument() as u8);
                }
                Data::Write(buffer) => {
                    assert_eq!(buffer.len() % BLOCK_SIZE, 0);
                    if self.write_count < self.write_lengths.len() {
                        self.write_lengths[self.write_count] = buffer.len();
                        self.write_count += 1;
                    }
                    assert!(buffer.iter().all(|&byte| byte == 0xA5));
                }
                Data::None => {}
            }

            match index {
                0 => Ok(Response::None),
                8 => self.cmd8,
                55 => Ok(Response::Short(0x0000_0120)),
                41 => {
                    self.cmd41_count += 1;
                    if self.cmd41_count > self.cmd41_ready_after {
                        Ok(Response::Short(0xC0FF_8000))
                    } else {
                        Ok(Response::Short(0x00FF_8000))
                    }
                }
                2 => Ok(Response::Long(CID)),
                3 => Ok(Response::Short(0x1234_0000)),
                9 => Ok(Response::Long(self.csd)),
                6 => self.acmd6,
                7 | 13 | 16 | 17 | 18 | 24 | 25 => {
                    if let Some(status) = self.r1_error {
                        Ok(Response::Short(status))
                    } else {
                        Ok(Response::Short(0x0000_0900))
                    }
                }
                _ => Err(SdmcError::UnsupportedCard),
            }
        }
    }

    impl SdHost for MockHost {
        fn execute(&mut self, command: Command, data: Data<'_>) -> Result<Response, SdmcError> {
            self.run(command, data)
        }

        fn configure_bus(&mut self, width: BusWidth) {
            self.bus_width = Some(width);
        }

        fn configure_clock(&mut self, clock: Hertz) {
            self.clock = Some(clock);
        }
    }

    struct MockDelay {
        total_ns: u64,
    }

    impl MockDelay {
        const fn new() -> Self {
            Self { total_ns: 0 }
        }
    }

    impl DelayNs for MockDelay {
        fn delay_ns(&mut self, ns: u32) {
            self.total_ns += ns as u64;
        }
    }

    fn init_default(host: &mut MockHost) -> (Card, CardInfo) {
        let mut delay = MockDelay::new();
        let mut card = Card::new(SdmcConfig::default());
        let info = card
            .init_blocking(host, &mut delay)
            .expect("identification");
        (card, info)
    }

    #[test]
    fn identifies_high_capacity_card() {
        let csd = csd_structure1(0x1F_FFFF);
        let mut host = MockHost::new(csd);
        let mut delay = MockDelay::new();
        let config = SdmcConfig::default();
        let mut card = Card::new(config);
        let info = card.init_blocking(&mut host, &mut delay).unwrap();

        assert_eq!(info.rca, 0x1234);
        assert!(info.high_capacity);
        assert_eq!(info.blocks, (0x1F_FFFF + 1) * 1024);
        assert_eq!(info.read_block_len, BLOCK_SIZE);
        assert_eq!(info.bus_width, BusWidth::FourWire);
        assert_eq!(info.clock, config.clock);
        assert_eq!(info.cid, CID);
        assert_eq!(info.csd, csd);
        assert_eq!(card.info(), Some(info));

        assert_eq!(host.argument(8), 0x1AA);
        assert_eq!(host.argument(41), 0x40FF_8000);
        assert_eq!(host.argument(7), 0x1234_0000);
        assert_eq!(host.argument(9), 0x1234_0000);
        assert_eq!(host.argument(16), BLOCK_SIZE_U32);
        assert_eq!(host.argument(6), ACMD6_FOUR_BIT);
        assert_eq!(host.argument(13), 0x1234_0000);
        assert_eq!(host.bus_width, Some(BusWidth::FourWire));
        assert_eq!(host.clock, Some(config.clock));
        assert_eq!(delay.total_ns, 10_000_000);

        assert_eq!(
            host.order[..host.order_len],
            [0, 8, 55, 41, 2, 3, 9, 7, 16, 55, 6, 13]
        );
    }

    #[test]
    fn cmd8_error_falls_back_to_sd_v1() {
        let mut host = MockHost::new(csd_structure0(0x1FF, 7, 9));
        host.cmd8 = Err(SdmcError::ResponseTimeout);
        let (_, info) = init_default(&mut host);

        assert!(!info.high_capacity);
        assert_eq!(info.blocks, 262_144);
        assert_eq!(host.count(8), 1);
        assert_eq!(host.argument(41), 0x00FF_8000);
    }

    #[test]
    fn cmd8_without_echo_is_treated_as_sd_v1() {
        let mut host = MockHost::new(csd_structure0(0x1FF, 7, 9));
        host.cmd8 = Ok(Response::Short(0));
        let (_, info) = init_default(&mut host);

        assert!(!info.high_capacity);
        assert_eq!(host.argument(41), 0x00FF_8000);
    }

    #[test]
    fn acmd41_retries_until_ready() {
        let mut host = MockHost::new(csd_structure1(0xFF));
        host.cmd41_ready_after = 3;
        let mut delay = MockDelay::new();
        let mut card = Card::new(SdmcConfig::default());
        let info = card.init_blocking(&mut host, &mut delay).unwrap();

        assert!(info.high_capacity);
        assert_eq!(host.count(41), 4);
        // Four ACMD41 attempts plus the ACMD6 APP_CMD prefix.
        assert_eq!(host.count(55), 5);
        assert_eq!(delay.total_ns, 13_000_000);
    }

    #[test]
    fn acmd41_timeout_fails_identification() {
        let mut host = MockHost::new(csd_structure1(0xFF));
        host.cmd41_ready_after = u32::MAX;
        let mut delay = MockDelay::new();
        let mut card = Card::new(SdmcConfig::default());

        assert_eq!(
            card.init_blocking(&mut host, &mut delay),
            Err(SdmcError::Timeout)
        );
        assert_eq!(host.count(41), 1000);
        assert_eq!(delay.total_ns, 1_009_000_000);
        assert_eq!(card.info(), None);
    }

    #[test]
    fn acmd6_error_falls_back_to_one_wire() {
        let mut host = MockHost::new(csd_structure1(0xFF));
        host.acmd6 = Err(SdmcError::ResponseCrc);
        let (_, info) = init_default(&mut host);

        assert!(info.high_capacity);
        assert_eq!(info.bus_width, BusWidth::OneWire);
        assert_eq!(host.bus_width, Some(BusWidth::OneWire));
        assert_eq!(host.count(6), 1);
    }

    #[test]
    fn acmd6_status_error_falls_back_to_one_wire() {
        let mut host = MockHost::new(csd_structure1(0xFF));
        host.acmd6 = Ok(Response::Short(0x0008_0000));
        let (_, info) = init_default(&mut host);

        assert_eq!(info.bus_width, BusWidth::OneWire);
        assert_eq!(host.bus_width, Some(BusWidth::OneWire));
    }

    #[test]
    fn one_wire_configuration_skips_acmd6() {
        let mut host = MockHost::new(csd_structure1(0xFF));
        let mut delay = MockDelay::new();
        let config = SdmcConfig {
            bus_width: BusWidth::OneWire,
            ..SdmcConfig::default()
        };
        let mut card = Card::new(config);
        let info = card.init_blocking(&mut host, &mut delay).unwrap();

        assert_eq!(info.bus_width, BusWidth::OneWire);
        assert_eq!(host.count(6), 0);
        assert_eq!(host.count(55), 1);
        assert_eq!(host.bus_width, Some(BusWidth::OneWire));
        assert_eq!(delay.total_ns, 10_000_000);
    }

    #[test]
    fn init_failure_is_reported() {
        let mut host = MockHost::new(csd_structure1(0xFF));
        host.r1_error = Some(1 << 16);
        let mut delay = MockDelay::new();
        let mut card = Card::new(SdmcConfig::default());

        // CMD7 is the first command whose R1 status is checked.
        assert_eq!(
            card.init_blocking(&mut host, &mut delay),
            Err(SdmcError::ResponseStatus(1 << 16))
        );
        assert_eq!(card.info(), None);
    }

    #[test]
    fn decodes_high_capacity_csd() {
        let (blocks, read_block_len) = Card::decode_csd(&csd_structure1(0x1234)).unwrap();
        assert_eq!(blocks, (0x1234 + 1) * 1024);
        assert_eq!(read_block_len, BLOCK_SIZE);
    }

    #[test]
    fn decodes_standard_capacity_csd() {
        let (blocks, read_block_len) = Card::decode_csd(&csd_structure0(0x1FF, 7, 9)).unwrap();
        // (c_size + 1) << (c_size_mult + 2 + read_bl_len) bytes / 512.
        assert_eq!(blocks, 262_144);
        assert_eq!(read_block_len, BLOCK_SIZE);

        // A READ_BL_LEN above 512 is capped at the driver block size.
        let (blocks, read_block_len) = Card::decode_csd(&csd_structure0(0x1FF, 7, 10)).unwrap();
        assert_eq!(blocks, 524_288);
        assert_eq!(read_block_len, BLOCK_SIZE);
    }

    #[test]
    fn rejects_invalid_csd() {
        // CSD structure 2 is not defined for SD memory cards.
        assert_eq!(
            Card::decode_csd(&[0x8000_0000, 0, 0, 0]),
            Err(SdmcError::UnsupportedCard)
        );
        // A zeroed CSD reports no usable capacity.
        assert_eq!(
            Card::decode_csd(&[0, 0, 0, 0]),
            Err(SdmcError::UnsupportedCard)
        );
        // 2 TiB needs more blocks than the u32 block count can hold.
        assert_eq!(
            Card::decode_csd(&csd_structure1(0x3F_FFFF)),
            Err(SdmcError::Overflow)
        );
    }

    #[test]
    fn high_capacity_commands_use_block_addresses() {
        let mut host = MockHost::new(csd_structure1(0x1F_FFFF));
        let (card, info) = init_default(&mut host);

        let single = card.read_command(7, 1).unwrap();
        assert_eq!(single.index(), 17);
        assert_eq!(single.argument(), 7);

        let multi = card.read_command(7, 8).unwrap();
        assert_eq!(multi.index(), 18);
        assert_eq!(multi.argument(), 7);

        let write_single = card.write_command(7, 1).unwrap();
        assert_eq!(write_single.index(), 24);
        let write_multi = card.write_command(7, 4).unwrap();
        assert_eq!(write_multi.index(), 25);

        assert_eq!(card.read_command(7, 0), Err(SdmcError::InvalidBlock));
        assert_eq!(
            card.read_command(7, MAX_BLOCKS_PER_TRANSFER + 1),
            Err(SdmcError::InvalidBlock)
        );
        assert_eq!(
            card.read_command(info.blocks - 1, 2),
            Err(SdmcError::OutOfRange)
        );
        assert_eq!(
            card.write_command(info.blocks, 1),
            Err(SdmcError::OutOfRange)
        );
    }

    #[test]
    fn standard_capacity_cards_use_byte_addresses() {
        let mut host = MockHost::new(csd_structure0(0x1FF, 7, 9));
        host.cmd8 = Err(SdmcError::ResponseTimeout);
        let (card, info) = init_default(&mut host);

        assert!(!info.high_capacity);
        assert_eq!(card.block_address(1).unwrap(), 512);
        assert_eq!(card.block_address(3).unwrap(), 1536);

        let command = card.read_command(3, 1).unwrap();
        assert_eq!(command.index(), 17);
        assert_eq!(command.argument(), 1536);

        let multi = card.write_command(3, 2).unwrap();
        assert_eq!(multi.index(), 25);
        assert_eq!(multi.argument(), 1536);

        // Byte addresses for an SDSC card cannot exceed 32 bits.
        assert_eq!(card.block_address(u32::MAX), Err(SdmcError::Overflow));
    }

    #[test]
    fn check_r1_detects_error_bits() {
        assert_eq!(Card::check_r1(Response::Short(0x0000_0900)), Ok(()));
        assert_eq!(Card::check_r1(Response::Short(0x0000_FFFF)), Ok(()));
        assert_eq!(
            Card::check_r1(Response::Short(0x8000_0000)),
            Err(SdmcError::ResponseStatus(0x8000_0000))
        );
        assert_eq!(
            Card::check_r1(Response::Short(0x0008_0000)),
            Err(SdmcError::ResponseStatus(0x0008_0000))
        );
        assert_eq!(
            Card::check_r1(Response::None),
            Err(SdmcError::UnsupportedCard)
        );
        assert_eq!(
            Card::check_r1(Response::Long([0; 4])),
            Err(SdmcError::UnsupportedCard)
        );
    }

    #[test]
    fn transfers_fail_on_r1_error() {
        let mut host = MockHost::new(csd_structure1(0xFF));
        let (mut card, _) = init_default(&mut host);

        host.r1_error = Some(1 << 31);
        let mut buffer = [0u8; BLOCK_SIZE];
        assert_eq!(
            card.read_blocks(&mut host, 0, &mut buffer),
            Err(SdmcError::ResponseStatus(1 << 31))
        );
        let written = [0xA5u8; BLOCK_SIZE];
        assert_eq!(
            card.write_blocks(&mut host, 0, &written),
            Err(SdmcError::ResponseStatus(1 << 31))
        );
    }

    #[test]
    fn read_blocks_splits_into_dma_sized_chunks() {
        let mut host = MockHost::new(csd_structure1(0x1F_FFFF));
        let (mut card, _) = init_default(&mut host);
        host.clear_log();

        const CHUNK: usize = MAX_BLOCKS_PER_TRANSFER as usize * BLOCK_SIZE;
        let mut buffer = [0u8; 2 * CHUNK];
        card.read_blocks(&mut host, 2, &mut buffer).unwrap();

        assert_eq!(host.read_count, 2);
        assert_eq!(host.read_lengths[0], CHUNK);
        assert_eq!(host.read_lengths[1], CHUNK);
        assert_eq!(host.order[..host.order_len], [18, 18]);
        // The second chunk starts one transfer limit after the first one.
        assert_eq!(host.arguments[18], 2 + MAX_BLOCKS_PER_TRANSFER as u32);
        assert!(buffer[..CHUNK].iter().all(|&byte| byte == 2));
        let second = (2 + MAX_BLOCKS_PER_TRANSFER as u32) as u8;
        assert!(buffer[CHUNK..].iter().all(|&byte| byte == second));
    }

    #[test]
    fn read_blocks_uses_cmd17_for_a_single_block() {
        let mut host = MockHost::new(csd_structure1(0x1F_FFFF));
        let (mut card, _) = init_default(&mut host);
        host.clear_log();

        let mut buffer = [0u8; BLOCK_SIZE];
        card.read_blocks(&mut host, 5, &mut buffer).unwrap();

        assert_eq!(host.order[..host.order_len], [17]);
        assert_eq!(host.arguments[17], 5);
        assert_eq!(host.read_count, 1);
        assert!(buffer.iter().all(|&byte| byte == 5));
    }

    #[test]
    fn read_blocks_uses_multi_block_then_single_block_tail() {
        let mut host = MockHost::new(csd_structure1(0x1F_FFFF));
        let (mut card, _) = init_default(&mut host);
        host.clear_log();

        let mut buffer = [0u8; (MAX_BLOCKS_PER_TRANSFER as usize + 1) * BLOCK_SIZE];
        card.read_blocks(&mut host, 0, &mut buffer).unwrap();

        assert_eq!(host.order[..host.order_len], [18, 17]);
        assert_eq!(host.arguments[18], 0);
        assert_eq!(host.arguments[17], MAX_BLOCKS_PER_TRANSFER as u32);
        assert_eq!(host.read_count, 2);
    }

    #[test]
    fn write_blocks_uses_cmd24_and_cmd25() {
        let mut host = MockHost::new(csd_structure1(0x1F_FFFF));
        let (mut card, _) = init_default(&mut host);
        host.clear_log();

        let single = [0xA5u8; BLOCK_SIZE];
        card.write_blocks(&mut host, 3, &single).unwrap();
        assert_eq!(host.order[..host.order_len], [24]);
        assert_eq!(host.arguments[24], 3);
        assert_eq!(host.write_lengths[0], BLOCK_SIZE);

        host.clear_log();
        let multi = [0xA5u8; 3 * BLOCK_SIZE];
        card.write_blocks(&mut host, 3, &multi).unwrap();
        assert_eq!(host.order[..host.order_len], [25]);
        assert_eq!(host.arguments[25], 3);
        assert_eq!(host.write_lengths[0], 3 * BLOCK_SIZE);
    }

    #[test]
    fn write_blocks_splits_into_reliable_chunks() {
        let mut host = MockHost::new(csd_structure1(0x1F_FFFF));
        let (mut card, _) = init_default(&mut host);
        host.clear_log();

        const BLOCKS: usize = MAX_WRITE_BLOCKS_PER_TRANSFER as usize;
        let buffer = [0xA5u8; BLOCKS * 2 * BLOCK_SIZE + BLOCK_SIZE];
        card.write_blocks(&mut host, 0, &buffer[..]).unwrap();

        // One command per reliable chunk, plus a single-block tail.
        assert_eq!(host.order[..host.order_len], [25, 25, 24]);
        assert_eq!(host.write_count, 3);
        assert_eq!(host.write_lengths[0], BLOCKS * BLOCK_SIZE);
        assert_eq!(host.write_lengths[1], BLOCKS * BLOCK_SIZE);
        assert_eq!(host.write_lengths[2], BLOCK_SIZE);
        assert_eq!(host.arguments[24], 2 * BLOCKS as u32);

        // The command builder enforces the same write limit.
        assert_eq!(
            card.write_command(0, MAX_WRITE_BLOCKS_PER_TRANSFER + 1),
            Err(SdmcError::InvalidBlock)
        );
    }

    #[test]
    fn transfers_reject_invalid_lengths_and_ranges() {
        let mut host = MockHost::new(csd_structure1(0x1F_FFFF));
        let (mut card, info) = init_default(&mut host);
        host.clear_log();

        let mut bad = [0u8; BLOCK_SIZE + 1];
        assert_eq!(
            card.read_blocks(&mut host, 0, &mut bad),
            Err(SdmcError::InvalidBlock)
        );
        assert_eq!(
            card.write_blocks(&mut host, 0, &bad),
            Err(SdmcError::InvalidBlock)
        );

        let mut two_blocks = [0u8; 2 * BLOCK_SIZE];
        assert_eq!(
            card.read_blocks(&mut host, info.blocks - 1, &mut two_blocks),
            Err(SdmcError::OutOfRange)
        );
        assert_eq!(
            card.write_blocks(&mut host, info.blocks, &two_blocks),
            Err(SdmcError::OutOfRange)
        );

        // Empty transfers are no-ops even at an out-of-range start block.
        let mut empty: [u8; 0] = [];
        assert_eq!(card.read_blocks(&mut host, info.blocks, &mut empty), Ok(()));
        assert_eq!(card.write_blocks(&mut host, info.blocks, &empty), Ok(()));
        assert_eq!(host.order_len, 0);
    }

    #[test]
    fn transfers_require_initialization() {
        let mut host = MockHost::new(csd_structure1(0xFF));
        let mut card = Card::new(SdmcConfig::default());
        let mut buffer = [0u8; BLOCK_SIZE];

        assert_eq!(
            card.read_blocks(&mut host, 0, &mut buffer),
            Err(SdmcError::NotInitialized)
        );
        assert_eq!(
            card.write_blocks(&mut host, 0, &buffer),
            Err(SdmcError::NotInitialized)
        );
        assert_eq!(card.block_address(0), Err(SdmcError::NotInitialized));
        assert_eq!(card.read_command(0, 1), Err(SdmcError::NotInitialized));
        assert_eq!(card.write_command(0, 1), Err(SdmcError::NotInitialized));
    }

    #[cfg(feature = "interrupts")]
    fn block_on<F: core::future::Future>(future: F) -> F::Output {
        let waker = core::task::Waker::noop();
        let mut context = core::task::Context::from_waker(waker);
        let mut future = core::pin::pin!(future);
        loop {
            if let core::task::Poll::Ready(value) = future.as_mut().poll(&mut context) {
                return value;
            }
        }
    }

    /// Interrupt-driven mock host built on [`MockHost`].
    #[cfg(feature = "interrupts")]
    struct MockHostAsync {
        inner: MockHost,
    }

    #[cfg(feature = "interrupts")]
    impl SdHostAsync for MockHostAsync {
        async fn execute(
            &mut self,
            command: Command,
            data: Data<'_>,
        ) -> Result<Response, SdmcError> {
            self.inner.run(command, data)
        }

        fn configure_bus(&mut self, width: BusWidth) {
            self.inner.bus_width = Some(width);
        }

        fn configure_clock(&mut self, clock: Hertz) {
            self.inner.clock = Some(clock);
        }
    }

    #[cfg(feature = "interrupts")]
    #[test]
    fn async_identification_and_transfers() {
        let mut host = MockHostAsync {
            inner: MockHost::new(csd_structure1(0xFF)),
        };
        host.inner.cmd41_ready_after = 2;
        let mut delay = MockDelay::new();
        let mut card = Card::new(SdmcConfig::default());

        let info = block_on(card.init_async(&mut host, &mut delay)).unwrap();
        assert_eq!(info.rca, 0x1234);
        assert!(info.high_capacity);
        assert_eq!(info.bus_width, BusWidth::FourWire);
        assert_eq!(host.inner.bus_width, Some(BusWidth::FourWire));
        assert_eq!(delay.total_ns, 12_000_000);

        let mut buffer = [0u8; 2 * BLOCK_SIZE];
        block_on(card.read_blocks_async(&mut host, 3, &mut buffer)).unwrap();
        assert_eq!(host.inner.read_lengths[0], 2 * BLOCK_SIZE);
        assert!(buffer.iter().all(|&byte| byte == 3));

        let written = [0xA5u8; 2 * BLOCK_SIZE];
        block_on(card.write_blocks_async(&mut host, 3, &written)).unwrap();
        assert_eq!(host.inner.write_lengths[0], 2 * BLOCK_SIZE);
    }
}

#[cfg(test)]
mod command_tests {
    use super::*;

    #[test]
    fn command_defaults() {
        let cmd = Command::new(8, 0x1AA, ResponseKind::Short);
        assert_eq!(cmd.index(), 8);
        assert_eq!(cmd.argument(), 0x1AA);
        assert!(cmd.checks_crc());
        assert!(!cmd.is_send_init());
        assert!(!cmd.is_stop());
    }

    #[test]
    fn command_without_crc() {
        let cmd = Command::new(41, 0x40FF_8000, ResponseKind::Short).without_crc();
        assert!(!cmd.checks_crc());
    }

    #[test]
    fn command_send_init_and_stop_flags() {
        let cmd = Command::new(0, 0, ResponseKind::None).send_init();
        assert!(cmd.is_send_init());
        let stop = Command::new(12, 0, ResponseKind::Short).stop();
        assert!(stop.is_stop());
    }

    #[test]
    fn data_lengths() {
        let mut read = [0u8; 1024];
        assert_eq!(Data::Read(&mut read).len(), 1024);
        assert_eq!(Data::Read(&mut read).blocks(), 2);
        assert_eq!(Data::Write(&[0u8; 512]).blocks(), 1);
        assert!(Data::None.is_none());
        assert_eq!(Data::None.len(), 0);
    }

    #[test]
    fn response_variants() {
        assert!(matches!(Response::Short(7), Response::Short(7)));
        assert!(matches!(Response::None, Response::None));
        assert!(matches!(
            Response::Long([1, 2, 3, 4]),
            Response::Long([1, 2, 3, 4])
        ));
    }
}
