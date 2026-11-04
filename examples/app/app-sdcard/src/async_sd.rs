//! Async SDMC read/write benchmark path (default unless `sync-sd` is selected).

use artinchip_hal::bind_interrupts;
use artinchip_hal::cmu::{ClockTree, Cmu};
use artinchip_hal::gtc::TimerDelay;
use artinchip_hal::prelude::*;
use artinchip_hal::sdmc::{
    AsyncSdmcHandler, BLOCK_SIZE, Hertz, Sdmc, SdmcConfig, SdmcError, SdmcPads, WRITEBACK_SLOTS,
};
use artinchip_hal::types::DmaBuffer;
use artinchip_rt::sync::StaticSlot;
use hadris::ErrorKind;
use hadris::fat::embedded::MountToken;
use hadris::fat::embedded::async_::Fat as AsyncFat;
use hadris::fs::OpenOptions;
use hadris::storage::Partition;
use hadris::storage::async_::BlockDevice as AsyncBlockDevice;
use log::{error, info};

use crate::common::{
    CARD_CLOCK_HZ, CHUNK, FILE_NAME, ROUNDS, Rate, Sample, WRITE_BYTES, detect_fs, elapsed,
    fat_partition,
};

bind_interrupts!(struct Irqs {
    SDMC1 => AsyncSdmcHandler<1>;
});

/// Four 512-byte write-back slots; `DmaBuffer` already guarantees DMA alignment.
const WRITE_BACK_BYTES: usize = WRITEBACK_SLOTS * BLOCK_SIZE;
static WRITE_BACK_BUFFER: StaticSlot<DmaBuffer<WRITE_BACK_BYTES>> =
    StaticSlot::new(DmaBuffer::new());

/// Identify the card and run the FAT file benchmark.
pub(crate) async fn run<PADS>(
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
    let mut card = sdmc
        .new_async(
            pads,
            SdmcConfig {
                clock: Hertz(CARD_CLOCK_HZ),
                ..SdmcConfig::default()
            },
            delay,
            clocks,
            cmu,
            Irqs,
        )
        .await?;
    info!("Clock {} kHz，async mode", card.info().clock.0 / 1000);
    card.load_buffer(WRITE_BACK_BUFFER.take());

    let mut sector = [0u8; BLOCK_SIZE];
    if let Err(err) = card.read_blocks(0, &mut sector).await {
        error!("sd: async cannot read the MBR: {err}");
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
    if let Err(err) = card.read_blocks(partition_block, &mut boot).await {
        error!("sd: async cannot read the partition boot sector: {err}");
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
    let mut fat = match AsyncFat::mount(
        Partition::new(card, offset * BLOCK_SIZE as u64, len * BLOCK_SIZE as u64),
        &mut token,
    )
    .await
    {
        Ok(fat) => fat,
        Err(err) => {
            error!("fs: async cannot mount the FAT volume: {err}");
            return Err(SdmcError::UnsupportedCard);
        }
    };

    let (writes, reads, rounds) = benchmark(&mut fat, pattern, buffer, clock).await;
    if rounds == ROUNDS {
        info!("File write : {}", Rate::average(writes));
        info!("File read  : {}", Rate::average(reads));
        info!("File verify: OK");
    }

    match fat.unmount().await {
        Ok(window) => {
            let mut card = window.into_inner();
            if let Err(err) = card.flush_writeback().await {
                error!("sd: async cannot flush buffered metadata: {err}");
            }
            Ok(())
        }
        Err(err) => {
            error!("fs: async cannot unmount the FAT volume: {err}");
            Err(SdmcError::UnsupportedCard)
        }
    }
}

/// Run [`ROUNDS`] write/read/verify rounds and collect the samples.
#[inline(never)]
async fn benchmark<'m, D>(
    fat: &mut AsyncFat<'m, D, 4>,
    pattern: &[u8],
    buffer: &mut [u8],
    clock: TimerDelay<'_>,
) -> ([Sample; ROUNDS], [Sample; ROUNDS], usize)
where
    D: AsyncBlockDevice,
{
    let root = fat.root();
    let mut writes = [Sample::default(); ROUNDS];
    let mut reads = [Sample::default(); ROUNDS];
    let mut rounds = 0;
    for round in 0..ROUNDS {
        // A crashed earlier run can leave the file with a broken chain.
        let _ = fat.remove_file(root, FILE_NAME).await;
        match file_benchmark(fat, pattern, buffer, clock).await {
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
async fn file_benchmark<'m, D>(
    fat: &mut AsyncFat<'m, D, 4>,
    pattern: &[u8],
    buffer: &mut [u8],
    clock: TimerDelay<'_>,
) -> Result<(Sample, Sample), hadris::Error<D::Error>>
where
    D: AsyncBlockDevice,
{
    let root = fat.root();
    let file = fat
        .open(
            root,
            FILE_NAME,
            OpenOptions::new().write().create().truncate(),
        )
        .await?;
    let start_tick = clock.get_tick();
    let mut written = 0u64;
    while written < WRITE_BYTES {
        let len = CHUNK.min((WRITE_BYTES - written) as usize);
        let n = fat.write(&file, &pattern[..len]).await?;
        if n == 0 {
            return Err(ErrorKind::Io.into());
        }
        written += n as u64;
    }
    let write = Sample {
        bytes: written,
        ticks: elapsed(start_tick, clock),
    };
    fat.flush(&file).await?;
    fat.close(file).await?;

    let file = fat.open(root, FILE_NAME, OpenOptions::new().read()).await?;
    let mut ticks = 0u64;
    let mut read = 0u64;
    loop {
        let start_tick = clock.get_tick();
        let n = fat.read(&file, &mut buffer[..CHUNK]).await?;
        ticks += elapsed(start_tick, clock);
        if n == 0 {
            break;
        }
        let mut at = (read as usize) % pattern.len();
        let mut done = 0;
        while done < n {
            let take = (pattern.len() - at).min(n - done);
            if buffer[done..done + take] != pattern[at..at + take] {
                fat.close(file).await?;
                return Err(ErrorKind::Corrupt.into());
            }
            done += take;
            at = 0;
        }
        read += n as u64;
    }
    fat.close(file).await?;
    if let Err(err) = fat.remove_file(root, FILE_NAME).await {
        error!("cannot remove {FILE_NAME}: {err}");
    }
    Ok((write, Sample { bytes: read, ticks }))
}
