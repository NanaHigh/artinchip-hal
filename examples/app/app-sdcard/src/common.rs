//! Shared test content of the two app-sdcard paths: both run the same 2 MiB /
//! 256 KiB chunk file benchmark, only the block device differs.

use artinchip_hal::gtc::{CntFreq, TimerDelay};
use artinchip_rt::sync::StaticSlot;

pub(crate) use artinchip_hal::sdmc::BLOCK_SIZE;
pub(crate) use artinchip_hal::sdmc::MAX_CLOCK_HZ as CARD_CLOCK_HZ;

pub(crate) const ROUNDS: usize = 3;

pub(crate) const WRITE_BYTES: u64 = 2 * 1024 * 1024;

pub(crate) const CHUNK: usize = 256 * 1024;

pub(crate) const FILE_NAME: &str = "SPEED.TMP";

pub(crate) const NS_PER_TICK: u64 = 1_000_000_000 / CntFreq::Freq4M.hz() as u64;

/// Cache-line aligned read destination for the file benchmark.
#[repr(C, align(64))]
pub(crate) struct FileBuffer([u8; CHUNK]);

impl FileBuffer {
    pub(crate) fn as_mut_slice(&mut self) -> &mut [u8] {
        &mut self.0
    }
}

/// Cache-line aligned write pattern; also the verify source.
#[repr(C, align(64))]
pub(crate) struct PatternBuffer([u8; CHUNK]);

impl PatternBuffer {
    pub(crate) fn as_slice(&self) -> &[u8] {
        &self.0
    }

    pub(crate) fn as_mut_slice(&mut self) -> &mut [u8] {
        &mut self.0
    }
}

pub(crate) static FILE_BUFFER: StaticSlot<FileBuffer> = StaticSlot::new(FileBuffer([0; CHUNK]));
pub(crate) static PATTERN_BUFFER: StaticSlot<PatternBuffer> =
    StaticSlot::new(PatternBuffer([0; CHUNK]));

pub(crate) fn fill_pattern(pattern: &mut [u8]) {
    for (index, byte) in pattern.iter_mut().enumerate() {
        *byte = (index & 0xff) as u8;
    }
}

/// One measured transfer: bytes moved and elapsed GTC ticks.
#[derive(Clone, Copy, Default)]
pub(crate) struct Sample {
    pub(crate) bytes: u64,
    pub(crate) ticks: u64,
}

/// Rate in MiB/s with two decimals.
pub(crate) struct Rate {
    bytes: u64,
    ticks: u64,
}

impl Rate {
    pub(crate) fn average<const N: usize>(samples: [Sample; N]) -> Self {
        Self {
            bytes: samples.iter().map(|sample| sample.bytes).sum(),
            ticks: samples.iter().map(|sample| sample.ticks).sum(),
        }
    }
}

impl core::fmt::Display for Rate {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let numerator = self.bytes.saturating_mul(1_000_000_000);
        let denominator = self
            .ticks
            .max(1)
            .saturating_mul(NS_PER_TICK)
            .saturating_mul(1024 * 1024);
        let hundredths = numerator.saturating_mul(100) / denominator;
        write!(f, "{}.{:02} MiB/s", hundredths / 100, hundredths % 100)
    }
}

pub(crate) fn elapsed(start: u64, clock: TimerDelay<'_>) -> u64 {
    clock.get_tick().wrapping_sub(start)
}

/// Start block and length of the first FAT partition in an MBR, if there is one.
pub(crate) fn fat_partition(sector: &[u8; BLOCK_SIZE]) -> Option<(u64, u64)> {
    for slot in 0..4 {
        let entry = &sector[446 + slot * 16..446 + slot * 16 + 16];
        if matches!(
            entry[4],
            0x01 | 0x04 | 0x06 | 0x07 | 0x0B | 0x0C | 0x0E | 0x14 | 0x16 | 0x1B | 0x1C | 0x1E
        ) {
            let start = u32::from_le_bytes([entry[8], entry[9], entry[10], entry[11]]) as u64;
            let blocks = u32::from_le_bytes([entry[12], entry[13], entry[14], entry[15]]) as u64;
            if start != 0 && blocks != 0 {
                return Some((start, blocks));
            }
        }
    }
    None
}

/// Filesystem information parsed from the partition boot sector.
pub(crate) struct FsInfo {
    pub(crate) kind: &'static str,
    pub(crate) cluster_bytes: u32,
}

/// Classify the FAT variant from the boot sector (the MBR type byte is a hint).
pub(crate) fn detect_fs(boot: &[u8; BLOCK_SIZE]) -> Result<FsInfo, &'static str> {
    if boot[0] != 0xEB && boot[0] != 0xE9 {
        return Err("partition does not start with a FAT boot sector");
    }
    if &boot[3..11] == b"EXFAT   " {
        return Err("the partition is exFAT, format it as FAT32 first");
    }
    if &boot[3..11] == b"NTFS    " {
        return Err("the partition is NTFS, format it as FAT32 first");
    }
    let bytes_per_sector = u16::from_le_bytes([boot[11], boot[12]]) as u32;
    if !matches!(bytes_per_sector, 512 | 1024 | 2048 | 4096) {
        return Err("invalid BPB sector size");
    }
    let sectors_per_cluster = boot[13] as u32;
    if sectors_per_cluster == 0 || !sectors_per_cluster.is_power_of_two() {
        return Err("invalid BPB cluster size");
    }
    let cluster_bytes = bytes_per_sector * sectors_per_cluster;
    let root_entries = u16::from_le_bytes([boot[17], boot[18]]) as u32;
    let fat16 = u16::from_le_bytes([boot[22], boot[23]]) as u32;
    if &boot[82..90] == b"FAT32   " || (fat16 == 0 && root_entries == 0) {
        return Ok(FsInfo {
            kind: "FAT32",
            cluster_bytes,
        });
    }
    if fat16 == 0 {
        return Err("the partition is not a FAT filesystem");
    }
    let reserved = u16::from_le_bytes([boot[14], boot[15]]) as u32;
    let fat_count = boot[16] as u32;
    let total16 = u16::from_le_bytes([boot[19], boot[20]]) as u32;
    let total32 = u32::from_le_bytes([boot[32], boot[33], boot[34], boot[35]]);
    let total = if total16 != 0 { total16 } else { total32 };
    let root_sectors = (root_entries * 32).div_ceil(bytes_per_sector);
    let data_sectors = total.saturating_sub(reserved + fat_count * fat16 + root_sectors);
    let clusters = data_sectors / sectors_per_cluster;
    let kind = if clusters < 4085 {
        "FAT12"
    } else if clusters < 65525 {
        "FAT16"
    } else {
        "FAT32"
    };
    Ok(FsInfo {
        kind,
        cluster_bytes,
    })
}
