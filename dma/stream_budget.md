# DMA stream budget on STM32

Status: **idea**, not started. Written 2026-09-22 while reviewing [PX4/PX4-Autopilot#28080](https://github.com/PX4/PX4-Autopilot/pull/28080) (concurrent BDShot capture), which is blocked on this. Verified against PX4 `main` at `3160543cd10` and the NuttX submodule it pins.

## Problem

Nothing records which DMA streams a board has committed. Each `board_dma_map.h` carries a hand-counted comment per controller ("Using at most 8 Channels on DMA1"), but no build step checks it against the defconfig or `timer_config.cpp`, and the comments drift. On fmu-v6c the DMA1 list names TIM5 UP while `timer_config.cpp` uses TIM1, and UART7 is labelled DMA1 while `DMAMAP_DMA12_UART7RX_1` puts it on DMA2. Oversubscription shows up only at run time: whichever driver asks last fails, so the loser depends on start order.

## How streams are allocated

**STM32H7** (`arch/arm/src/stm32h7/stm32_dma.c`, `stm32_dmachannel()`): the DMAMAP value selects only the controller (DMA1, DMA2, BDMA, MDMA) and the DMAMUX request. The allocator returns the first unused stream on that controller, or NULL when none is free, so every consumer on a controller competes with every other. Each stream has a `used` flag and no record of its owner.

**STM32F4/F7**: the DMAMAP value fixes the stream, and `stm32_dmachannel()` waits on that stream's semaphore until the holder frees it. A conflict is two consumers mapped to the same stream.

DMA consumers on H7 boards:

| Consumer | Enabled by | Allocates in |
|---|---|---|
| NuttX serial | `CONFIG_<port>_RXDMA` / `_TXDMA` and `DMAMAP_<port>_RX` / `_TX` | `up_dma_setup()`, on first open; freed on last close |
| NuttX SPI | `CONFIG_STM32H7_SPIn_DMA` | `spi_bus_initialize()`, at boot |
| NuttX ADC | `CONFIG_STM32H7_ADCn_DMA` | `adc_setup()` |
| PX4IO serial | `PX4IO_SERIAL_TX_DMAMAP` / `_RX_DMAMAP` | `ArchPX4IOSerial::init()` |
| Serial RGB LED | `S_RGB_LED_DMA` | `neopixel_init()` |
| DShot | `DMA{}` argument to `initIOTimer()` | `init_timers_dma_up()`, one stream per timer; BDShot frees and re-allocates it twice per cycle |

## Failure modes

- The H7 serial driver ignores a failed allocation. `up_dma_setup()` passes the result of `stm32_dmachannel()` straight to `stm32_dmasetup()` (`stm32_serial.c:2231`, `:2245`), under a comment that says it "should always succeed".
- The H7 SPI driver only `DEBUGASSERT`s the handle. Its comment says `stm32_dmachannel()` blocks until the stream is free, which is F7 behavior.
- When `dshot_start_timer_burst()` can't get the UP stream, it logs and skips that timer's output for the cycle. BDShot frees the stream every cycle, so on a full controller a driver that opens in that window takes it, and the timer's motors get no commands until it is released.
- [#24573](https://github.com/PX4/PX4-Autopilot/pull/24573): CRSF RC input stopped on ARK FPV once BDShot was enabled. [#24610](https://github.com/PX4/PX4-Autopilot/pull/24610) fixed it by cutting BDShot capture to one round-robin stream per timer.

## Idea

Make the budget a compile-time check per board. For each controller, sum the peak number of streams every enabled consumer can hold at once, and `static_assert` the sum against the controller's size (8 streams each on DMA1 and DMA2, 8 channels on BDMA). If it fits, first-free allocation cannot fail in any start order, because no consumer holds more than it was counted for.

Everything the check needs is visible to the preprocessor or `constexpr`:

- `DMAMAP_CONTROLLER(m)` (`hardware/stm32_dmamux.h`) is a plain macro, so every `DMAMAP_*` in `board_dma_map.h` resolves to a controller at compile time.
- Serial, SPI and ADC DMA enables are `CONFIG_*` symbols in `nuttx/config.h`.
- `io_timers[]` is `constexpr` in each board's `timer_config.cpp`. Count one stream per timer with a `DMA{}`, plus four capture streams per timer that opts into concurrent capture.

Count a serial port whose DMA is enabled even if nothing opens it at boot. Whether it opens depends on parameters (`MAV_n_CONFIG`, `GPS_n_CONFIG`, ...), so a check that counted only boot-time users would still let a parameter change oversubscribe the controller.

For F4/F7 the equivalent check is that no two enabled consumers map to the same stream.

With the check in place, the per-controller comment in `board_dma_map.h` can go, and a board that opts into concurrent capture either fits or fails to build.

## Open questions

- Where the check lives. It needs `io_timers[]`, which each board defines in `timer_config.cpp`; a shared header included at the end of that file is one option.
- Whether to patch NuttX's H7 serial driver to fail the open, or fall back to interrupt-driven I/O, on a NULL handle. That goes to apache/nuttx first.
- Whether to add an nsh command that dumps per-controller stream usage. NuttX keeps only the `used` flag, so naming the owner needs extra bookkeeping.
- Whether BDShot should hold its UP stream permanently when capture has streams of its own.
- How many of the 57 H7 boards are already over budget. The first build with the check answers this.
