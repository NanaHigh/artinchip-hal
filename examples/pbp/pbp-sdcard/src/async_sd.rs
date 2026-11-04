//! Async SDMC file benchmark path (feature `async-sd`): each phase runs in its
//! own `block_on` so async temporaries do not accumulate on the stack.

use artinchip_hal::bind_interrupts;
use artinchip_hal::gtc::{CntFreq, TimerDelay};
use artinchip_hal::prelude::*;
use artinchip_hal::sdmc::{AsyncSdmcHandler, Hertz, SdmcConfig, SdmcError};
use artinchip_hal::uart::{UartConfig, uart_logger_init};
use artinchip_rt::Peripherals;
use embassy_futures::block_on;
use hadris::ErrorKind;
use hadris::fat::embedded::MountToken;
use hadris::fat::embedded::async_::Fat as AsyncFat;
use hadris::fs::OpenOptions;
use hadris::storage::Partition;
use hadris::storage::async_::BlockDevice as AsyncBlockDevice;
use log::{error, info};

use crate::common::{
    BLOCK_SIZE, CARD_CLOCK_HZ, FILE_BUFFER, FILE_NAME, PATTERN_BUFFER, ROUNDS, Rate, Sample,
    WRITE_BYTES, WRITE_CHUNK, detect_fs, elapsed, fat_partition, fill_pattern,
};

bind_interrupts!(struct Irqs {
    SDMC1 => AsyncSdmcHandler<1>;
});

/// Create, write, read back and remove `name`, timing only the transfers.
#[inline(never)]
async fn file_benchmark<'m, D>(
    fat: &mut AsyncFat<'m, D, 4>,
    pattern: &[u8],
    buffer: &mut [u8],
    clock: TimerDelay<'_>,
    name: &str,
) -> Result<(Sample, Sample), hadris::Error<D::Error>>
where
    D: AsyncBlockDevice,
{
    let root = fat.root();
    let file = fat
        .open(root, name, OpenOptions::new().write().create().truncate())
        .await?;
    let start_tick = clock.get_tick();
    let mut written = 0u64;
    while written < WRITE_BYTES {
        let len = WRITE_CHUNK.min((WRITE_BYTES - written) as usize);
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

    let file = fat.open(root, name, OpenOptions::new().read()).await?;
    // Only the read calls are timed: the byte comparison is CPU work.
    let mut ticks = 0u64;
    let mut read = 0u64;
    loop {
        let start_tick = clock.get_tick();
        let n = fat.read(&file, &mut buffer[..WRITE_CHUNK]).await?;
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
    if let Err(err) = fat.remove_file(root, name).await {
        error!("cannot remove {name}: {err}");
    }
    Ok((write, Sample { bytes: read, ticks }))
}

/// Run the benchmark rounds and collect the samples.
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
        match file_benchmark(fat, pattern, buffer, clock, FILE_NAME).await {
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

#[inline(never)]
pub(crate) fn run(
    mut p: Peripherals,
    clocks: artinchip_hal::cmu::ClockTree,
) -> Result<(), SdmcError> {
    clocks.configure_plls(&mut p.cmu);
    // D13x CPU PLL boost; the D21x C906 keeps its boot clock (switching it broke boot).
    #[cfg(feature = "d13x")]
    clocks.switch_cpu_to_pll_int0(&mut p.cmu);

    let uart0_tx = p.gpioa.pa0.into_uart0_tx();
    let uart0_rx = p.gpioa.pa1.into_uart0_rx();
    let clock = p.gtc.new_timer_delay(CntFreq::Freq4M, &mut p.cmu);
    let _uart0 = uart_logger_init(
        p.uart0,
        uart0_tx,
        uart0_rx,
        UartConfig::default(),
        &clocks,
        &mut p.cmu,
    )
    .unwrap();
    info!("Welcome to pbp-sdcard by artinchip-hal🦀!");

    // SDMC1 pin mux (function 2) on the reference board TF slot.
    let clk = p.gpioc.pc2.into_sdmc1_clk();
    let cmd = p.gpioc.pc3.into_sdmc1_cmd();
    let dat0 = p.gpioc.pc1.into_sdmc1_dat0();
    let dat1 = p.gpioc.pc0.into_sdmc1_dat1();
    let dat2 = p.gpioc.pc5.into_sdmc1_dat2();
    let dat3 = p.gpioc.pc4.into_sdmc1_dat3();
    let _det = p.gpioc.pc6.into_sdmc1_det();

    // The write pattern is filled once, outside every timed window.
    let pattern = PATTERN_BUFFER.take();
    fill_pattern(pattern.as_mut_slice());
    let file_buffer = FILE_BUFFER.take();

    // Phase 1: create the driver and identify the card. `p` is consumed here.
    let mut card = block_on(async move {
        p.sdmc1
            .new_async(
                (clk, cmd, dat0, dat1, dat2, dat3),
                SdmcConfig {
                    clock: Hertz(CARD_CLOCK_HZ),
                    ..SdmcConfig::default()
                },
                clock,
                &clocks,
                &mut p.cmu,
                Irqs,
            )
            .await
    })?;
    info!("Clock {} kHz，async mode", card.info().clock.0 / 1000);

    // Phase 2: locate the FAT partition window in the MBR.
    let window = block_on(async {
        let mut sector = [0u8; BLOCK_SIZE];
        card.read_blocks(0, &mut sector).await?;
        Ok::<_, SdmcError>(fat_partition(&sector))
    })?;
    let (offset, len) = match window {
        Some(window) => window,
        None => {
            error!("fs: no FAT partition in the MBR");
            return Ok(());
        }
    };
    let partition_block = match u32::try_from(offset) {
        Ok(block) => block,
        Err(_) => {
            error!("fs: partition offset does not fit the card address space");
            return Ok(());
        }
    };
    let boot = block_on(async {
        let mut boot = [0u8; BLOCK_SIZE];
        card.read_blocks(partition_block, &mut boot).await?;
        Ok::<_, SdmcError>(boot)
    })?;
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

    // Phase 3: mount the FAT volume on the partition window.
    let mut token = MountToken::new();
    let mounted = block_on(async {
        AsyncFat::mount(
            Partition::new(card, offset * BLOCK_SIZE as u64, len * BLOCK_SIZE as u64),
            &mut token,
        )
        .await
    });
    let mut fat = match mounted {
        Ok(fat) => fat,
        Err(err) => {
            error!("fs: cannot mount the FAT volume: {err}");
            return Ok(());
        }
    };

    // Phase 4: the benchmark rounds, each in its own `block_on`.
    let (writes, reads, rounds) = block_on(benchmark(
        &mut fat,
        pattern.as_slice(),
        file_buffer.as_mut_slice(),
        clock,
    ));
    if rounds == ROUNDS {
        info!("File write : {}", Rate::average(writes));
        info!("File read  : {}", Rate::average(reads));
        info!("File verify: OK");
    }

    // Phase 5: unmount the filesystem.
    match block_on(async { fat.unmount().await }) {
        Ok(window) => {
            let _card = window.into_inner();
        }
        Err(err) => error!("fs: cannot unmount: {err}"),
    }
    Ok(())
}
