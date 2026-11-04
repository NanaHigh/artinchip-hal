# app-sdcard — SD card FAT file benchmark (async / blocking driver)

Second-stage application booted by `app-bootloader`. It identifies the TF card,
mounts the allocator-free **`hadris` 3.0 embedded FAT** driver and measures file
write / read / verify speed.

Two driver paths share the same test content and are selected at build time:

| build | driver | library features |
|:--|:--|:--|
| default | interrupt-driven `AsyncSdmc` (with a write-back cache) | chip feature only |
| `sync-sd` | blocking `BlockingSdmc` (mirrors the PBP sync path) | `<chip>,sync-sd` |

Each image runs one path from a clean boot, so the two measurements never share
a controller. (Running both in one boot would mean handing the controller over
with `AsyncSdmc::free()`; that handover did leave the card busy on the reference
board, so it is not a supported path today.)

The app is not limited to the 96 KiB PBP SRAM: it runs from DDR3 (D21x) or PSRAM
(D13x), which leaves room for full-size test buffers, a caller-provided buffer
for the write-back cache and 256 KiB filesystem chunks.

| chip feature | load address | notes |
|:--|:--|:--|
| `d13x` (default) | `0x40700000` | PSRAM application region, SPI-NOR board |
| `d21x` | `0x42000000` | DDR3 application region, SPI-NAND board |

Data path: a 2 MiB file is written in 256 KiB chunks (3 rounds, average
reported), read back into a second 256 KiB buffer and compared chunk by chunk
with the pattern buffer. The async path loads four 512-byte write-back cache
slots, so FAT metadata writes of one block are merged instead of hitting the card
one by one; the blocking path mirrors the PBP example and uses no caller buffer
by default (`ENABLE_READ_AHEAD` in `sync_sd.rs` enables a 32-block read-ahead
buffer for A/B comparison).

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

Console: **UART0** (PA0 TX / PA1 RX), 115200 8N1. The driver currently uses a
4-bit bus; D13x SDC0 and D21x SDC0 (8-bit capable) are available in the pin mux
tables but not used by this example.

## Build

Requirements: nightly Rust with both bare-metal targets installed:

```bash
rustup target add riscv32imafc-unknown-none-elf riscv64gc-unknown-none-elf
```

The application is packed together with the boot chain by `aicfwc`; the board
preset supplies the partition layout and the load address used by `build.rs`.

### Async path (default)

```bash
# D13x (RV32, SPI-NOR)
cargo run -p aicfwc -- --board d13x-nor --app app-sdcard --build

# D21x (RV64, SPI-NAND)
cargo run -p aicfwc -- --board d21x-nand --app app-sdcard --build
```

### Blocking path (`sync-sd`)

The board preset selects the default (async) features, so the blocking image is
built by hand and packed from the already-built application:

```bash
# D13x: build the sync application, then pack it into its own output directory
cargo build -p app-sdcard --target riscv32imafc-unknown-none-elf --release --no-default-features --features d13x,sync-sd
cargo run -p aicfwc -- --board d13x-nor --app app-sdcard --out-dir target/riscv32imafc-unknown-none-elf/release/app-sdcard-sync-out

# D21x
cargo build -p app-sdcard --target riscv64gc-unknown-none-elf --release --no-default-features --features d21x,sync-sd
cargo run -p aicfwc -- --board d21x-nand --app app-sdcard --out-dir target/riscv64gc-unknown-none-elf/release/app-sdcard-sync-out
```

Run the default async packaging once first so the boot-chain pieces
(`bootloader.aic`, `usbupg-*.aic`, ...) exist; the sync pack command reuses them.
Re-running the async `--build` command afterwards restores the default
`app-sdcard-out/` images.

Artifacts:

| Directory | Content |
|:--|:--|
| `target/<target>/release/app-sdcard-out/` | `app-sdcard.img` (`AIC.FW` for AiBurn), `app-sdcard.aic`, `app-sdcard-raw-*.img`, boot-chain pieces |
| `target/<target>/release/app-sdcard-sync-out/` | the same set for the blocking-driver image |

## Flash

* **AiBurn (USB burn mode)**: write `app-sdcard.img` (`AIC.FW`); it contains the
  application record (and the boot-chain records when they changed). This is
  the validated path.
* **External programmer**: the `app-sdcard-raw-*.img` images are laid out by
  flash partition and can be written at media offset `0x0`. On the tested D13x
  board the raw-boot path has not been validated yet (it traps before the app
  starts while AiBurn works), so prefer the AiBurn image for now:

| SoC | Media | Offset | Programmer settings |
|:--|:--|:--|:--|
| D13x | SPI-NOR (`W25Q128JV`) | `0x0` | standard 3-byte address |
| D21x | SPI-NAND (`W25N01GV`) | `0x0` | page 2048 B, block 128 KiB |

## Expected output

Async image:

```text
[INFO] Welcome to app-sdcard by artinchip-hal🦀!
[INFO] Clock 50000 kHz，async mode
[INFO] Card size: 29.72 GiB, Format: FAT32, Cluster: 32 KiB
[INFO] File write : 13.xx MiB/s
[INFO] File read  : 21.xx MiB/s
[INFO] File verify: OK
[INFO] Done
```

Blocking image (`sync-sd`) prints the same sequence with `sync mode`. Read
bandwidth is close to the async path (both drivers use the internal DMA with the
full descriptor chain); **file write is lower** because the blocking path has no
write-back cache and writes every FAT metadata block through the card.

Measured async baseline at 50 MHz SDR 4-bit (23.84 MiB/s theoretical) with a
32 KiB-cluster FAT32 card:

| board | file write | file read | note |
|:--|--:|--:|:--|
| D13x | 13.26 MiB/s | 21.53 MiB/s | PSRAM application region |
| D21x | 15.04 MiB/s | 21.90 MiB/s | DDR3 application region |

The write path is split into 64 KiB IDMA commands by the driver; reads use the
full 256 KiB descriptor chain. The application stays alive after the benchmark
(60 s timer) so the console remains usable.

## Files

| File | Purpose |
|:--|:--|
| `src/main.rs` | entry, UART/logging, pad/clock setup, driver-path selection, keep-alive |
| `src/common.rs` | test constants, static pattern/read buffers, MBR/BPB parsing, rate samples |
| `src/async_sd.rs` | interrupt-driven path: `new_async`, write-back cache, FAT benchmark |
| `src/sync_sd.rs` | blocking path (`sync-sd`): `new_blocking`, optional read-ahead buffer, FAT benchmark |
| `build.rs` | links the application at the board-preset load address |
| `Cargo.toml` | chip features, driver-path feature and dependencies |

## Design notes

* The filesystem seam is the `hadris` block-device trait (`async_::BlockDevice`
  for the async path, `sync::BlockDevice` for the blocking path); the HAL keeps
  no hidden data buffers, only the fixed 512 B async bounce. The write-back cache
  buffer is owned by this example and loaded with `load_buffer`.
* The blocking path has an optional 32-block caller buffer for the read-ahead
  cache. It is disabled by default (`ENABLE_READ_AHEAD` in `sync_sd.rs`) so the
  first run matches the PBP sync validation; flip the constant to benchmark the
  read-ahead cache.
* SDMC1 has a hardware card-detect unit (`SDMC_CDET`): the driver programs a
  ~10 ms debounce, waits for it after power-on and reports
  `no SD card detected in the slot` instead of timing out when the slot is
  empty.
* The `Format:` label comes from the partition boot sector (BPB), not from the
  MBR type byte: FAT12/FAT16/FAT32 are distinguished by cluster count, while
  exFAT and NTFS are rejected explicitly.
* The example requires an **MBR** partition table (no GPT, no superfloppy), a
  FAT partition starting at a u32 LBA, and 512-byte logical sectors.

## Troubleshooting

* `no SD card detected in the slot`: check the TF card seat and the DET pin
  (PC6, internal pull-up expected on the card socket).
* `the partition is exFAT, format it as FAT32 first`: the embedded FAT stack
  does not parse exFAT; reformat the card as FAT32.
* `no FAT partition in the MBR`: create an MBR partition table with a FAT32
  primary partition (a superfloppy without MBR is not supported).
* `sd: benchmark failed: SDMC card busy`: the card is still finishing a write;
  only seen with the same-controller handover that is not supported, so flash
  one path per image.
* `Clock` much lower than 50 MHz: the driver selects the fastest achievable
  card clock that does not exceed the request; on some PLL configurations only
  a few dividers are available.
* No UART output at all: check the console pins (PA0/PA1) and that the image was
  written to the right partition (AiBurn `app-sdcard.img`, or the raw image at
  media offset `0x0` with the correct NOR/NAND media type).
