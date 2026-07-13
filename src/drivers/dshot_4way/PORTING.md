# dshot_4way — BLHeli 4way passthrough for PX4

Goal: expose the BLHeli/SiLabs bootloader on the DShot output pins over a USB
serial port, so `esc-configurator` / BLHeliSuite can read & write ESC settings
(specifically **Beacon Delay = Infinite** to stop the idle beep) while the ESCs
stay soldered to the FC.

Target board: **Matek H743-Slim (STM32H7)**, PX4 **v1.15.4**.
ESC: JHEMCU EM40A = BLHeli_S (SiLabs EFM8) → uses the "BLHeli bootloader"
protocol implemented in `serial_4way_avrootloader.c` (imSIL_BLB).

## Design

Port Betaflight's `serial_4way` 4way-interface to PX4. The protocol code is
kept verbatim; only the platform layer is swapped via a single compat header.

```
esc-configurator (webserial)
        │  4way MSP-like protocol over USB CDC (/dev/ttyACM0)
        ▼
dshot_4way_main.cpp        ── nsh command: stop dshot, open CDC, run loop
        ▼
serial_4way.c              ── esc4wayProcess(): verbatim BF protocol state machine
        ▼
serial_4way_avrootloader.c ── BL_* bootloader cmds + 52us/bit soft-UART (verbatim)
        ▼
bf_compat.h + px4_serial_4way_io.c  ── PX4 glue:
    - IO_t/IORead/IOHi/IOLo/IOConfigGPIO  -> px4_arch_gpio* on dshot channel pins
    - micros()/millis()                    -> hrt_absolute_time()
    - serialRead/Write/RxWaiting/TxFree    -> read()/write() on the CDC fd
    - motorGetIo/Enable/Disable            -> io_timer_channel_get_gpio_output + dshot stop
```

## Key facts (verified in-tree)

- `io_timer_channel_get_gpio_output(channel)` (`px4_arch/io_timer.h`) returns the
  GPIO output pinset for a dshot channel — basis for all 6 ESC pin primitives.
- `px4_arch_configgpio/gpioread/gpiowrite` = `stm32_configgpio/gpioread/gpiowrite`.
- Soft-UART: **52 µs/bit (~19200), one-wire, idle-high**, busy-wait on micros().
- BF already defines the 6 primitives (isEscHi/Lo, setEscHi/Lo, setEscInput/Output)
  in serial_4way.c via IORead/IOHi/IOLo/IOConfigGPIO → we only supply those macros.

## Open risks / tuning knobs

1. **Soft-UART jitter.** BF runs the bit-bang with IRQs enabled; long ISRs can
   corrupt a byte. If reads/writes are flaky, gate the 10-bit sample window in
   `suart_getc_/putc_` with `px4_enter_critical_section()` (leave the start-bit
   wait outside the critical section). First cut mirrors BF (IRQs on).
2. **Channel→motor mapping.** First cut maps `selected_esc` 0..N directly to
   io_timer channel index 0..N. Verify order matches Motor1..4 for this board.
3. **Input pin config.** setEscInput = input + pull-up; setEscOutput = push-pull.
   Derived from the output pinset by masking MODE/PUPD bits (STM32H7).
4. On exit we re-run dshot init to restore the timer/AF on the pins.

## Status — WORKING (dump + beacon write verified on hardware)

- [x] branch `blheli-passthrough`, BF reference in ../reference_betaflight
- [x] BF files copied into module
- [x] bf_compat.h + px4 IO glue (px4_serial_4way_io.c)
- [x] includes adapted in BF files
- [x] nsh command (dshot_4way_main.cpp)
- [x] board enable (CONFIG_DRIVERS_DSHOT_4WAY=y) + compile + link OK
- [x] hardware test step 1: `dshot_4way dump` — bit-bang validated, full 112-byte
      settings read cleanly. ESCs identified as **Bluejay** on SiLabs EFM8BB21 (sig 0xE8B2).
- [x] hardware test step 2: `dshot_4way beacon <esc> 5` — Beacon Delay written to
      Infinite (0x05) and read-back-verified on all 4 ESCs.

## What was broken and how it was fixed (the "not quite working" part)

1. **Bootloader entry.** A continuously-powered BLHeli_S app only jumps to the
   bootloader (0x1C00) from `init_no_signal`: after it loses the throttle signal it
   needs a *clean, long* idle-HIGH window (>15ms, and enough time to declare signal
   loss). The old code held the line high ~1ms then blasted the wake string, so the
   app never handed off. Fix: `BL_ConnectBootloader()` drives the line push-pull HIGH
   for 80ms before each wake attempt (25 tries). Entry is ~2/3 per attempt, so the
   on-FC commands should be retried (the harness loops until connect).
2. **RX framing.** The soft-UART disabled IRQs *per byte* only; an ISR firing in an
   inter-byte gap corrupted a byte, and a single bad byte fails the whole read. Fix:
   hold IRQs off across the *entire* handshake/read (nested enter/leave in suart keep
   it disabled). Reads are chunked (8B read / 16B write) to bound each IRQ-off window
   to the ~6-13ms regime proven safe by the connect handshake.
3. **SiLabs read framing.** `BL_ReadBuf` picks data+CRC+ACK vs data+ACK from
   `isMcuConnected()` (global `DeviceInfo`). We bypass the MSP `Connect()`, so we set
   `DeviceInfo` ourselves after connecting (and clear it before, since the connect
   handshake itself must see it false). Without this the CRC bytes were read as data.

## Beacon Delay (the goal): offset 0x1D in the 0x1A00 page

Values: 1=1m 2=2m 3=5m 4=10m **5=Infinite**. Confirmed against dump (0x1B=BeepStr=0x28,
0x1C=BeaconStr=0x50, 0x1D=BeaconDelay). Write = read 512B page, set byte, `BL_PageErase`
(512B page), `BL_WriteFlash` page back, verify. Erasing this page can at worst reset ESC
settings to Bluejay defaults (firmware @0x0000, bootloader @0x1C00 untouched) — not a brick.

## Topology note (user's rig)
FC USB -> Raspberry Pi (runs sverk-ros2 docker, forwards MAVLink). No direct PC-USB
to FC, and ESC only reachable via the 8-wire ribbon (motor signals on channels 0-3;
`dshot status` shows func 101-104 = Motor1-4 on ch 0-3, but scrambled:
ch0=M3, ch1=M2, ch2=M1, ch3=M4). esc-configurator can't reach the FC serial here AND
our port speaks raw 4way (no MSP), so passthrough+esc-configurator is impractical.
=> Pivoted to self-contained on-FC commands (dump / beacon write) run from QGC console.

## dump (read-only, current step)
`dshot stop; dshot_4way dump; dshot start`. Reads BLHeli_S settings at flash 0x1A00,
112 bytes, hexdumped per ESC. Beacon Delay offset TBD from dump vs known layout
(network was down when fetching authoritative layout; confirm empirically first).

Build fixes applied: `/*`-in-comment, `ioMem` shadow (removed global decl),
inner `i` shadow -> `j`, serialBeginWrite/EndWrite no-op shims.
Firmware: build/matek_h743-slim_default/matek_h743-slim_default.px4

## Build / test

```
# enable in boards/matek/h743-slim/default.px4board:  CONFIG_DRIVERS_DSHOT_4WAY=y
cd ~/Documents/Applications/px4/PX4-Autopilot
make matek_h743-slim_default
# flash, then on nsh / QGC MAVLink console (REMOVE PROPS):
dshot stop
dshot_4way dump 4          # read-only: hexdump each ESC's 0x1A00 settings
dshot_4way beacon 0 5      # set ESC 0 Beacon Delay = Infinite (retry if "no bootloader response")
dshot_4way beacon 1 5
dshot_4way beacon 2 5
dshot_4way beacon 3 5
dshot start                # restore normal DShot output; ESCs re-read settings on app restart
```
Tip: entry is flaky (~2/3), so retry each command until it prints "connected". Capturing
long output to `/fs/microsd/x.txt` then `cat` is more reliable than the console (the
bit-bang starves USB CDC). The raw `start` (esc-configurator passthrough) path is left in
but is impractical on this rig — see topology note.
