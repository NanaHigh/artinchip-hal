//! Blocking SDMC read/write benchmark path (`sync-sd` feature).
//!
//! Same FAT file benchmark as the async path, with [`BlockingSdmc`] and `hadris`'s
//! synchronous block device.

use artinchip_hal::cmu::{ClockTree, Cmu};
use artinchip_hal::gtc::TimerDelay;
use artinchip_hal::prelude::*;
use artinchip_hal::sdmc::{BLOCK_SIZE, BlockingSdmc, Hertz, Sdmc, SdmcConfig, SdmcError, SdmcPads};
use artinchip_hal::types::DmaBuffer;
use artinchip_rt::sync::StaticSlot;
use embedded_hal::delay::DelayNs;
use hadris::ErrorKind;
use hadris::fat::embedded::MountToken;
use hadris::fat::embedded::sync::Fat as SyncFat;
use hadris::fs::OpenOptions;
use hadris::storage::Partition;
use hadris::storage::sync::BlockDevice as SyncBlockDevice;
use log::{error, info};

use crate::common::{
    CARD_CLOCK_HZ, CHUNK, FILE_NAME, ROUNDS, Rate, Sample, WRITE_BYTES, detect_fs, elapsed,
    fat_partition,
};

/// Enable the blocking read-ahead buffer (off by default; flip to benchmark it).
const ENABLE_READ_AHEAD: bool = false;

/// Read-ahead buffer: 32 blocks in two 16-block lines (both FAT copies resident).
const READ_AHEAD_BYTES: usize = 32 * BLOCK_SIZE;
static READ_AHEAD_BUFFER: StaticSlot<DmaBuffer<READ_AHEAD_BYTES>> =
    StaticSlot::new(DmaBuffer::new());

/// Identify the card and run the FAT file benchmark.
pub(crate) fn run<PADS>(
    sdmc: Sdmc<1>,
    pads: PADS,
    delay: TimerDelay<'_>,
    clocks: &ClockTree,
    cmu: &mut Cmu,
    pattern: &[u8],
    buffer: &mut [u8],
) -> Result<(), SdmcError>
where
    PADS: SdmcPads<1>,
{
    let clock = delay;
    let mut card = sdmc.new_blocking(
        pads,
        SdmcConfig {
            clock: Hertz(CARD_CLOCK_HZ),
            ..SdmcConfig::default()
        },
        delay,
        clocks,
        cmu,
    )?;
    info!("Clock {} kHz，sync mode", card.clock().0 / 1000);
    if ENABLE_READ_AHEAD {
        card.load_buffer(READ_AHEAD_BUFFER.take());
    }

    let mut sector = [0u8; BLOCK_SIZE];
    if let Err(err) = card.read_blocks(0, &mut sector) {
        error!("sd: sync cannot read the MBR: {err}");
        return Ok(());
    }
    let Some((offset, len)) = fat_partition(&sector) else {
        error!("fs: no FAT partition in the MBR");
        return Ok(());
    };
    let partition_block = match u32::try_from(offset) {
        Ok(block) => block,
        Err(_) => {
            error!("fs: partition offset does not fit the card address space");
            return Ok(());
        }
    };
    let mut boot = [0u8; BLOCK_SIZE];
    if let Err(err) = card.read_blocks(partition_block, &mut boot) {
        error!("sd: sync cannot read the partition boot sector: {err}");
        return Ok(());
    }
    let fs = match detect_fs(&boot) {
        Ok(fs) => fs,
        Err(reason) => {
            error!("fs: {reason}");
            return Ok(());
        }
    };
    let total = u64::from(card.block_count());
    let gib_x100 = (total * 512 * 100) / (1024 * 1024 * 1024);
    info!(
        "Card size: {}.{:02} GiB, Format: {}, Cluster: {} KiB",
        gib_x100 / 100,
        gib_x100 % 100,
        fs.kind,
        fs.cluster_bytes / 1024
    );

    let mut token = MountToken::new();
    let mut fat = match mount(card, &mut token, offset, len) {
        Ok(fat) => fat,
        Err(err) => {
            error!("fs: sync cannot mount the FAT volume: {err}");
            return Err(SdmcError::UnsupportedCard);
        }
    };

    let (writes, reads, rounds) = benchmark(&mut fat, pattern, buffer, clock);
    if rounds == ROUNDS {
        info!("File write : {}", Rate::average(writes));
        info!("File read  : {}", Rate::average(reads));
        info!("File verify: OK");
    }

    match fat.unmount() {
        Ok(_window) => Ok(()),
        Err(err) => {
            error!("fs: sync cannot unmount the FAT volume: {err}");
            Err(SdmcError::UnsupportedCard)
        }
    }
}

/// Mount the FAT volume, keeping the `SyncFat` temporaries in their own frame.
#[allow(clippy::type_complexity)]
#[inline(never)]
fn mount<'m, PADS, D>(
    card: BlockingSdmc<1, PADS, D>,
    token: &'m mut MountToken,
    offset: u64,
    len: u64,
) -> Result<SyncFat<'m, Partition<BlockingSdmc<1, PADS, D>>, 4>, SdmcError>
where
    PADS: SdmcPads<1>,
    D: DelayNs,
{
    // `Partition::new` takes a byte offset and byte length.
    SyncFat::mount(
        Partition::new(card, offset * BLOCK_SIZE as u64, len * BLOCK_SIZE as u64),
        token,
    )
    .map_err(|_| SdmcError::UnsupportedCard)
}

/// Run [`ROUNDS`] write/read/verify rounds and collect the samples.
#[inline(never)]
fn benchmark<'m, D>(
    fat: &mut SyncFat<'m, D, 4>,
    pattern: &[u8],
    buffer: &mut [u8],
    clock: TimerDelay<'_>,
) -> ([Sample; ROUNDS], [Sample; ROUNDS], usize)
where
    D: SyncBlockDevice,
{
    let root = fat.root();
    let mut writes = [Sample::default(); ROUNDS];
    let mut reads = [Sample::default(); ROUNDS];
    let mut rounds = 0;
    for round in 0..ROUNDS {
        // A crashed earlier run can leave the file with a broken chain.
        let _ = fat.remove_file(root, FILE_NAME);
        match file_benchmark(fat, pattern, buffer, clock) {
            Ok((write, read)) => {
                writes[round] = write;
                reads[round] = read;
                rounds += 1;
            }
            Err(err) => {
                error!("file benchmark failed: {err}");
                break;
            }
        }
    }
    (writes, reads, rounds)
}

/// Write the file, then read it back and verify it against `pattern`.
#[inline(never)]
fn file_benchmark<'m, D>(
    fat: &mut SyncFat<'m, D, 4>,
    pattern: &[u8],
    buffer: &mut [u8],
    clock: TimerDelay<'_>,
) -> Result<(Sample, Sample), hadris::Error<D::Error>>
where
    D: SyncBlockDevice,
{
    let root = fat.root();
    let file = fat.open(
        root,
        FILE_NAME,
        OpenOptions::new().write().create().truncate(),
    )?;
    let start_tick = clock.get_tick();
    let mut written = 0u64;
    while written < WRITE_BYTES {
        let len = CHUNK.min((WRITE_BYTES - written) as usize);
        let n = fat.write(&file, &pattern[..len])?;
        if n == 0 {
            return Err(ErrorKind::Io.into());
        }
        written += n as u64;
    }
    let write = Sample {
        bytes: written,
        ticks: elapsed(start_tick, clock),
    };
    fat.flush(&file)?;
    fat.close(file)?;

    let file = fat.open(root, FILE_NAME, OpenOptions::new().read())?;
    let mut ticks = 0u64;
    let mut read = 0u64;
    loop {
        let start_tick = clock.get_tick();
        let n = fat.read(&file, &mut buffer[..CHUNK])?;
        ticks += elapsed(start_tick, clock);
        if n == 0 {
            break;
        }
        let mut at = (read as usize) % pattern.len();
        let mut done = 0;
        while done < n {
            let take = (pattern.len() - at).min(n - done);
            if buffer[done..done + take] != pattern[at..at + take] {
                fat.close(file)?;
                return Err(ErrorKind::Corrupt.into());
            }
            done += take;
            at = 0;
        }
        read += n as u64;
    }
    fat.close(file)?;
    if let Err(err) = fat.remove_file(root, FILE_NAME) {
        error!("cannot remove {FILE_NAME}: {err}");
    }
    Ok((write, Sample { bytes: read, ticks }))
}
