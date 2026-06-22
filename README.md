# SLIMrev

Firmware for the SLIM (Structure for Lossless Ion Manipulation) Reverser module.
The SLIM Reverser accepts eight Twave electrode input signals and provides eight switched outputs, allowing the direction of ion travel through the Twave to be reversed under software or hardware control. This was developed to support any Twave system not just SLIM.

Developed by **Gordon Anderson — GAA Custom Electronics, LLC**

---

## Hardware

| Item | Detail |
|------|--------|
| Microcontroller | Adafruit Trinket M0 (SAMD21E18A) |
| Clock | 48 MHz |
| RAM | 32 KB |
| Flash | 256 KB |
| Switch IC | MAX14802 16-channel SPI relay |
| Host interface | USB CDC Serial (115200 baud) |

### Pin Assignments

| Pin | Function |
|-----|----------|
| 0 | MAX14802 latch strobe output (LTCH) |
| 1 | External opto-isolated trigger input (TRIG) |
| 13 | Heartbeat status LED |
| SPI bus | MAX14802 data/clock |
| Internal | Adafruit DotStar RGB LED |

---

## Features

- **Three switch states** — Forward, Reverse, and Open (all relays open), each with an independently configurable 16-bit relay bit pattern.
- **External trigger input** — An opto-isolated digital input can be mapped to any active/inactive direction state pair, with configurable active-high or active-low polarity.
- **FLASH-backed configuration** — All settings are persisted to on-chip Flash and restored on power-up.  A signature check falls back to factory defaults if the stored config is invalid.
- **Dual-bank firmware** — Supports a lower/upper Flash bank arrangement for field firmware updates; `LoadAltRev` checks pin A31 on boot and can vector to an alternate firmware at `0x20000`.
- **ASCII serial command protocol** — Human-readable commands with ACK/NAK responses; optional echo mode for scripted host control.
- **Thread scheduler** — Uses ArduinoThread/ThreadController to run a 40 Hz background update task without blocking the main serial loop.

---

## Build

The project uses [PlatformIO](https://platformio.org/).

```bash
# Build
pio run

# Upload
pio run --target upload

# Serial monitor (115200 baud)
pio device monitor --baud 115200
```

**`platformio.ini` target:** `adafruit_trinket_m0`

### Dependencies

The following libraries must be available under the path configured in `platformio.ini` (`lib_extra_dirs`):

| Library | Purpose |
|---------|---------|
| ArduinoThread / ThreadController | Cooperative thread scheduler |
| Adafruit DotStar | On-board RGB LED driver |
| FlashStorage / FlashAsEEPROM | FLASH-backed persistent storage |
| SerialBuffer | Ring buffer serial support |
| SPI | MAX14802 SPI relay driver |
| Wire | I²C bus (available for expansion) |

---

## Serial Command Protocol

Commands are ASCII strings sent at **115200 baud, 8-N-1** over the USB CDC port.
Each command is terminated with `\n` or `;`.  Arguments are comma-separated.
The device responds with:

- **ACK** (`0x06`) — command accepted
- **NAK** (`0x15 ?`) — command rejected; retrieve the error code with `GERR`

### General Commands

| Command | Args | Description |
|---------|------|-------------|
| `GVER` | — | Report firmware version string |
| `GERR` | — | Report last error code |
| `GCMDS` | — | List all supported command tokens |
| `RESET` | — | Software reboot |
| `SAVE` | — | Persist current configuration to Flash |
| `RESTORE` | — | Reload configuration from Flash |
| `FORMAT` | — | Reset Flash configuration to factory defaults |
| `MUTE` | `ON\|OFF` | Suppress / restore all serial responses |
| `ECHO` | `TRUE\|FALSE` | Enable / disable command echo mode |
| `DELAY` | `ms` | Blocking delay (milliseconds) |
| `DEBUG` | `n` | Ad-hoc diagnostic hook |
| `THREADS` | — | List threads: name, ID, interval, state, runtime |
| `STHRDENA` | `name, TRUE\|FALSE` | Enable or disable a named thread |
| `WHERE` | — | Report current firmware load address (hex) |
| `GOTO` | `addr` | Jump to hex address (e.g. `0x20000`) |
| `ERASEU` | — | Erase upper Flash bank (`0x20000`+) |
| `M0PGM` | `addr, size` | Receive and burn a file to Flash over serial |
| `ARBPGM` | `addr, size` | Alternate path for the same Flash programming protocol |

### SLIM Reverser Commands

| Command | Args | Description |
|---------|------|-------------|
| `GSTATE` | — | Report current switch state (`FWD`, `REV`, or `OPEN`) |
| `SSTATE` | `FWD\|REV\|OPEN` | Set switch state |
| `GFWD` | — | Report forward flag (`TRUE` / `FALSE`) |
| `SFWD` | `TRUE\|FALSE` | Set direction: `TRUE` = forward, `FALSE` = reverse |
| `GENAEXT` | — | Report external trigger enable state |
| `SENAEXT` | `TRUE\|FALSE` | Enable / disable external trigger input |
| `GAHIGH` | — | Report trigger polarity |
| `SAHIGH` | `TRUE\|FALSE` | Set trigger polarity: `TRUE` = active-high |
| `GASTATE` | — | Report direction applied when trigger is active |
| `SASTATE` | `FWD\|REV\|OPEN` | Set direction for active trigger state |
| `GISTATE` | — | Report direction applied when trigger is inactive |
| `SISTATE` | `FWD\|REV\|OPEN` | Set direction for inactive trigger state |
| `GFWDP` | — | Report forward relay bit pattern (4-digit hex) |
| `SFWDP` | `xxxx` | Set forward relay bit pattern (16-bit hex) |
| `GREVP` | — | Report reverse relay bit pattern |
| `SREVP` | `xxxx` | Set reverse relay bit pattern (16-bit hex) |
| `GOPENP` | — | Report open-state relay bit pattern |
| `SOPENP` | `xxxx` | Set open-state relay bit pattern (16-bit hex) |

### Example Session

```
GVER
  → SLIMrev version 1.1, Nov 5, 2023
SSTATE,REV
  → ACK
GSTATE
  → FWD   (external trigger overrides if SENAEXT,TRUE)
SFWDP,00FF
  → ACK
SAVE
  → ACK
```

---

## Factory Default Configuration

| Setting | Default |
|---------|---------|
| External trigger enabled | `TRUE` |
| Trigger polarity | Active-high |
| Direction | Forward |
| Active trigger state | Forward |
| Inactive trigger state | Reverse |
| Forward pattern | `0x00FF` |
| Reverse pattern | `0xFF00` |
| Open pattern | `0x0000` |

---

## Flash Programming Protocol (`M0PGM` / `ARBPGM`)

Used to burn a binary file into on-chip Flash over the USB serial connection:

1. Host sends: `M0PGM,<hex address>,<decimal byte count>\n`
2. Device replies **ACK** if the address and size are valid.
3. Host sends each byte as two ASCII hex characters (no delimiter).
4. Host sends `\n`, then the CRC-8 value (decimal), then `\n`.
5. Device verifies the CRC and replies **ACK** on success or **NAK** on failure.

Data is written in 256-byte blocks with a read-back verify after each block.

---

## Source Files

| File | Description |
|------|-------------|
| `src/SLIMrev.cpp` | Arduino `setup()`, `loop()`, direction control, host command handlers |
| `src/SLIMrev.h` | `SLIMREVdata` config struct, `Direction` enum, function prototypes |
| `src/Serial.cpp` | Ring buffer, tokeniser, command dispatch table and state machine |
| `src/Serial.h` | Ring buffer type, command table types, all serial function prototypes |
| `src/Hardware.cpp` | ADC/DAC calibration, TC5 scan timer, CRC-8, Flash programmer, MAX14802 driver |
| `src/Hardware.h` | `ADCchan` / `DACchan` structs, hardware function prototypes |
| `src/Errors.h` | Error code definitions and `SetErrorCode` / `BADARG` macros |
| `src/Hooks.c` | SysTick hook forwarding to `mySysTickHook` in application code |
| `src/AtomicBlock.h` | Portable interrupt-safe atomic block template (third-party, C. Andrews) |

---

## Revision History

| Version | Date | Notes |
|---------|------|-------|
| 1.0 | Jan 23, 2022 | Initial release |
| 1.1 | Nov 5, 2023 | Added OPEN state; editable relay bit patterns; configurable trigger active/inactive states |

---

## License

Copyright © GAA Custom Electronics, LLC.  All rights reserved.
