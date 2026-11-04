# PBP SD Card — minimal FAT file benchmark

First-stage (PBP) example that brings up the SDMC controller, identifies the TF
card, mounts the allocator-free **`hadris` 3.0 embedded FAT** driver and measures
file write / read / verify speed. It runs from the BootROM like the other PBP
examples; PSRAM and DRAM are not used.

Two interchangeable driver paths are provided:

| feature | driver | stack (measured) | notes |
|:--|:--|:--|:--|
| `async-sd` | interrupt-driven `AsyncSdmc` | ~15 KiB | each phase runs in its own `block_on` |
| `sync-sd` | blocking `BlockingSdmc` | ~9 KiB | split into `#[inline(never)]` phases |

Both paths use a **16 KiB** PBP stack (`artinchip-rt` default) and load **no
caller-provided driver buffer**: the HAL keeps only the fixed 512 B async
bounce, so metadata writes go straight to the card. The higher-level
`app-sdcard` application uses `load_buffer` (write-back cache), 256 KiB chunks
and a larger stack for maximum throughput.

## Wiring

The example uses **SDMC1** on both SoCs (all pins in mux function 2):

| Signal | Pin | Function |
|:--|:--|:--|
| CLK | PC2 | SDC1_CLK (2) |
| CMD | PC3 | SDC1_CMD (2) |
| DAT0 | PC1 | SDC1_D0 (2) |
| DAT1 | PC0 | SDC1_D1 (2) |
| DAT2 | PC5 | SDC1_D2 (2) |
| DAT3 | PC4 | SDC1_D3 (2) |
| DET | PC6 | SDC1_DET (2) |

Console: **UART0** (PA0 TX / PA1 RX), 115200 8N1.

D13x SDC0 (PB6–PB11) and D21x SDC0 (PB0–PB11, 8-bit capable including
D4–D7/DS/RST) are also defined in the pin mux tables, but this example drives
SDC1. The driver currently uses a 4-bit bus.

## Build

Requirements: nightly Rust with both bare-metal targets installed:

```bash
rustup target add riscv32imafc-unknown-none-elf riscv64gc-unknown-none-elf
```

Each build is three commands: compile, flatten the ELF, pack the raw boot image.
Run them from the repository root. The image must fit the 128 KiB boot
partition; for the async path the default `release` profile is too large, so
both paths use the size-first `pbp-release` profile.

### D13x (RV32, SPI-NOR) — async

```bash
cargo build -p pbp-sdcard --target riscv32imafc-unknown-none-elf --profile pbp-release --no-default-features --features d13x,async-sd
rust-objcopy -O binary target/riscv32imafc-unknown-none-elf/pbp-release/pbp-sdcard target/riscv32imafc-unknown-none-elf/pbp-release/pbp-sdcard-d13x-async-sd.bin
cargo run -p aicfwc -- target/riscv32imafc-unknown-none-elf/pbp-release/pbp-sdcard-d13x-async-sd.bin --raw-img --spi-nor
```

### D13x (RV32, SPI-NOR) — sync

```bash
cargo build -p pbp-sdcard --target riscv32imafc-unknown-none-elf --profile pbp-release --no-default-features --features d13x,sync-sd
rust-objcopy -O binary target/riscv32imafc-unknown-none-elf/pbp-release/pbp-sdcard target/riscv32imafc-unknown-none-elf/pbp-release/pbp-sdcard-d13x-sync-sd.bin
cargo run -p aicfwc -- target/riscv32imafc-unknown-none-elf/pbp-release/pbp-sdcard-d13x-sync-sd.bin --raw-img --spi-nor
```

### D21x (RV64, SPI-NAND) — sync only

The D21x PBP SRAM window is only 96 KiB (`0x00100000`-`0x00117FFF`, PBP loaded at
`0x00103000`), while D13x has 1 MiB. The async image (~103 KiB code + async
state) cannot fit, so `async-sd` is rejected at compile time for `d21x`. The
sync build uses 2 KiB test chunks and a 12 KiB stack so code + bss end below
`0x00118000`.

### D21x (RV64, SPI-NAND) — sync

```bash
cargo build -p pbp-sdcard --target riscv64gc-unknown-none-elf --profile pbp-release --no-default-features --features d21x,sync-sd
rust-objcopy -O binary target/riscv64gc-unknown-none-elf/pbp-release/pbp-sdcard target/riscv64gc-unknown-none-elf/pbp-release/pbp-sdcard-d21x-sync-sd.bin
cargo run -p aicfwc -- target/riscv64gc-unknown-none-elf/pbp-release/pbp-sdcard-d21x-sync-sd.bin --raw-img --spi-nand
```

`aicfwc` writes two files under
`target/<target>/pbp-release/pbp-sdcard-<chip>-<path>-out/`:

* `pbp-sdcard-<chip>-<path>.pbp` — checksum-repaired PBP file;
* `pbp-sdcard-<chip>-<path>.img` — **raw media image, this is the one to burn**.

Without `--raw-img` the converter only validates and repairs the PBP checksum:

```bash
cargo run -p aicfwc -- target/riscv32imafc-unknown-none-elf/pbp-release/pbp-sdcard-d13x-async-sd.bin
```

## Flash

| SoC | Media | Burn offset | Programmer settings |
|:--|:--|:--|:--|
| D13x | SPI-NOR (`W25Q128JV`) | `0x0` (128 KiB boot partition) | standard 3-byte address |
| D21x | SPI-NAND (`W25N01GV`) | `0x0` | page 2048 B, block 128 KiB |

Burn the `.img`, not the `.pbp` file. The D21x image already contains the `AICP`
page table that the BootROM expects for SPI-NAND (`aicfwc --raw-img --spi-nand`
adds it); page 0 is that table and the PBP starts on page 1.

## Expected output

```text
[INFO] Welcome to pbp-sdcard by artinchip-hal🦀!
[INFO] Clock 50000 kHz，async mode      (or "sync mode")
[INFO] Card size: 29.72 GiB, Format: FAT32, Cluster: 32 KiB
[INFO] File write : 6.xx MiB/s
[INFO] File read  : 20.xx MiB/s
[INFO] File verify: OK
[INFO] Done
```

The test writes a 2 MiB file in 64 KiB chunks (3 rounds, average reported), reads
it back into a separate 64 KiB buffer and compares every chunk with the pattern.
At 50 MHz SDR 4-bit (23.84 MiB/s theoretical) a healthy FAT32 card gives
roughly 6–7 MiB/s file write and ~20 MiB/s file read without driver buffers.
D21x sync uses 2 KiB chunks to fit its 96 KiB SRAM, so it is much slower
(~1 MiB/s write / ~4 MiB/s read). Loading a caller buffer (`load_buffer`) in the
`app-sdcard` application pushes file write to ~13–15 MiB/s and file read to
~21 MiB/s.

## Files

| File | Purpose |
|:--|:--|
| `src/main.rs` | entry point and startup (buffers/parsing live in `common.rs`) |
| `src/common.rs` | MBR/BPB parsing, benchmark samples, static test buffers |
| `src/async_sd.rs` | interrupt-driven path (`async-sd`) |
| `src/sync_sd.rs` | blocking path (`sync-sd`) |

`common.rs` parses the MBR locally (FAT partition start/length) instead of
linking `hadris-part`.

## Troubleshooting

The `Format:` label is derived from the partition boot sector (BPB), not from
the MBR type byte: exFAT and NTFS partitions are rejected explicitly with a
message instead of being mislabelled as `FAT`.

* No UART output at all: check the console pins (PA0/PA1, function 5) and that
  the image was burned at media offset `0x0` with the right media type
  (NOR vs NAND page size).
* `Card size: ... Format: FAT32` appears but the file benchmark fails: the card
  must have an MBR partition table with a FAT32/FAT16 partition (a superfloppy
  without MBR is not supported).
* `no SD card detected in the slot`: SDMC1 has a hardware card-detect unit. The
  driver enables the debounce timer, waits for it after power-on and fails fast
  instead of timing out. Check the TF card seat and the DET pin (PC6).
* Missing `Clock ...` line or HardFault: the PBP stack is 16 KiB for both paths;
  the async path was measured at ~15 KiB, so large additions can overflow it.
* D21x: the D13x CPU PLL boost is compiled out on D21x (the C906 keeps its boot
  clock); if the board still does not start, verify the TF slot is really wired
  to SDC1, or switch the example to SDC0 pins and `Sdmc::<0>`/`SDMC0` handler.
