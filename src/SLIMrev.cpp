// SLIMrev.cpp — SLIM reverser main firmware
//
// Controls the MAX14802 16-channel relay switch to reverse (or open) a set of
// eight Twave electrode signals.  Direction is commanded over USB serial or
// driven automatically by an opto-isolated external input.
//
// Hardware: Adafruit Trinket M0 (SAMD21E18A)
//   pin 0 — MAX14802 latch output
//   pin 1 — external opto-isolated trigger input
//
// Revision history:
//   1.0  Jan 23 2022 — initial release
//   1.1  Nov  5 2023 — added OPEN state, editable bit patterns, configurable
//                      trigger active/inactive states
//
// Gordon Anderson — GAA Custom Electronics, LLC
// gaa@owt.com

#include <Arduino.h>
#include <variant.h>
#include <wiring_private.h>
#include "SERCOM.h"
#include <Thread.h>
#include <ThreadController.h>
#include <Adafruit_DotStar.h>
#include <SPI.h>
#include <FlashStorage.h>
#include <FlashAsEEPROM.h>
#include <SerialBuffer.h>

#include "Hardware.h"
#include "SLIMrev.h"
#include "Errors.h"
#include "Serial.h"

// ── Version string (stored in FLASH to save RAM) ─────────────────────────────
const char Version[] PROGMEM = "SLIMrev version 1.1, Nov 5, 2023";

// ── Global state ──────────────────────────────────────────────────────────────
SLIMREVdata srdata; // Active configuration; mirrored to FLASH on SAVE command

// On-board DotStar RGB LED (single pixel, BGR colour order)
Adafruit_DotStar strip = Adafruit_DotStar(1, INTERNAL_DS_DATA, INTERNAL_DS_CLK, DOTSTAR_BGR);

SerialBuffer sb;

// FLASH-backed storage region for srdata
FlashStorage(flash_srdata, SLIMREVdata);

// ── Thread scheduler ──────────────────────────────────────────────────────────
ThreadController control      = ThreadController();
Thread           SystemThread = Thread();

// ── Factory default configuration ────────────────────────────────────────────
SLIMREVdata Rev_1_srdata = {
  sizeof(SLIMREVdata), "SLIMrev", 1,
  /*Ext*/        true,
  /*ActiveHigh*/ true,
  /*Fwd*/        true,
  /*dir*/        FWD_DIR,
  /*activeState*/   FWD_DIR,
  /*inactiveState*/ REV_DIR,
  /*fwdPattern*/  0x00FF,
  /*revPattern*/  0xFF00,
  /*openPattern*/ 0x0000,
  SIGNATURE
};

// ── SysTick hook ──────────────────────────────────────────────────────────────
// The Arduino SAMD framework calls mySysTickHook() (if non-NULL) from its
// SysTick ISR via Hooks.c.  The pointer is defined here and set to the stub
// below, providing a convenient place to add millisecond-granularity tasks
// without modifying framework files.
void msTimerIntercept(void);
void (*mySysTickHook)(void) = msTimerIntercept;

void msTimerIntercept(void)
{
  // Placeholder — add any 1 ms periodic work here
}

// ── Forward declaration ───────────────────────────────────────────────────────
void Update(void);

// ── Arduino entry points ──────────────────────────────────────────────────────

void setup()
{
  // Check for alternate firmware in upper FLASH bank and jump to it if present
  LoadAltRev();

  pinMode(0,  OUTPUT);
  pinMode(13, OUTPUT);

  // Turn off the on-board DotStar LED
  strip.begin();
  strip.setPixelColor(0, 0, 0, 0);
  strip.show();

  // Load config from FLASH; fall back to factory defaults if signature is bad
  srdata = flash_srdata.read();
  if (srdata.Signature != SIGNATURE) srdata = Rev_1_srdata;

  SerialInit();
  analogReadResolution(12);
  analogWriteResolution(12);

  // Register the 40 Hz background update thread
  SystemThread.setName((char *)"Update");
  SystemThread.onRun(Update);
  SystemThread.setInterval(25); // 25 ms → 40 Hz
  control.add(&SystemThread);

  MAX14802(0x0000); // Let the main loop apply the correct pattern on first run
  pinMode(TRIG, INPUT);
}

// Background update thread — runs at 40 Hz via ThreadController.
// Toggles the status LED at 0.5 Hz (every 40 ticks × 25 ms = 1 s half-period).
void Update(void)
{
  static int i = 40;

  if (i-- == 0)
  {
    i = 40;
    digitalWrite(13, !digitalRead(13)); // toggle heartbeat LED
  }
}

void loop()
{
  static Direction curDir = NA_DIR; // tracks the last direction applied to the switch

  ProcessSerial();
  control.run();

  // Read the external trigger and map it to activeState / inactiveState
  if (srdata.Ext)
  {
    bool trigHigh = (digitalRead(TRIG) == HIGH);
    srdata.dir = (trigHigh == srdata.ActiveHigh) ? srdata.activeState
                                                 : srdata.inactiveState;
  }

  // Only update the switch hardware when the direction actually changes
  if (srdata.dir != curDir)
  {
    curDir     = srdata.dir;
    srdata.Fwd = (srdata.dir == FWD_DIR);

    if      (srdata.dir == FWD_DIR)  MAX14802(srdata.fwdPattern);
    else if (srdata.dir == REV_DIR)  MAX14802(srdata.revPattern);
    else if (srdata.dir == OPEN_DIR) MAX14802(srdata.openPattern);
  }
}

// ── Serial I/O pump ───────────────────────────────────────────────────────────

// Transfer any available USB serial bytes into the ring buffer, then (if scan
// is true) dispatch all complete commands.  Pass scan=false when called from
// inside a blocking operation (e.g. ProgramFLASH) to keep the buffer drained
// without re-entering the command processor.
void ProcessSerial(bool scan)
{
  if (Serial.available() > 0) PutCh(Serial.read());
  if (!scan) return;
  if (RB_Commands(&RB) > 0) while (ProcessCommand() == 0);
}

// ── Settings persistence ──────────────────────────────────────────────────────

void SaveSettings(void)
{
  srdata.Signature = SIGNATURE;
  flash_srdata.write(srdata);
  SendACK;
}

void RestoreSettings(void)
{
  SLIMREVdata srsd = flash_srdata.read();
  if (srsd.Signature == SIGNATURE)
  {
    srdata = srsd;
  }
  else
  {
    SetErrorCode(ERR_EEPROMWRITE);
    SendNAK;
    return;
  }
  SendACK;
}

void Software_Reset(void)
{
  NVIC_SystemReset();
}

void FormatFLASH(void)
{
  flash_srdata.write(Rev_1_srdata);
  SendACK;
}

// ── Direction helper functions ────────────────────────────────────────────────

// Parse a direction token ("FWD", "REV", or "OPEN") and return the
// corresponding enum value.  On an unrecognised token, sends NAK and
// returns NA_DIR so callers can early-exit.
Direction checkState(char *val)
{
  if      (strcmp(val, "FWD")  == 0) return FWD_DIR;
  else if (strcmp(val, "REV")  == 0) return REV_DIR;
  else if (strcmp(val, "OPEN") == 0) return OPEN_DIR;

  SetErrorCode(ERR_BADARG);
  SendNAK;
  return NA_DIR;
}

// Send ACK followed by the direction name as a string.
void reportState(Direction dir)
{
  SendACKonly;
  if (SerialMute) return;
  if      (dir == FWD_DIR)  serial->println("FWD");
  else if (dir == REV_DIR)  serial->println("REV");
  else if (dir == OPEN_DIR) serial->println("OPEN");
  else                      serial->println("NA");
}

// ── Host command implementations ──────────────────────────────────────────────

// SFWD — set direction via TRUE (forward) / FALSE (reverse) boolean string
void setFWD(char *val)
{
  if      (strcmp(val, "TRUE")  == 0) { srdata.dir = FWD_DIR; srdata.Fwd = true;  }
  else if (strcmp(val, "FALSE") == 0) { srdata.dir = REV_DIR; srdata.Fwd = false; }
  else { BADARG; }
  SendACK;
}

// SSTATE / GSTATE — set or report the current switch direction
void setState(char *val)
{
  Direction dir = checkState(val);
  if (dir == NA_DIR) return;
  SendACK;
  srdata.dir = dir;
  srdata.Fwd = (srdata.dir == FWD_DIR);
}

void getState(void) { reportState(srdata.dir); }

// SASTATE/GASTATE — direction applied when the trigger is in its active state
void setActiveState(char *val)   { if (checkState(val) == NA_DIR) return; SendACK; srdata.activeState   = checkState(val); }
void getActiveState(void)        { reportState(srdata.activeState); }

// SISTATE/GISTATE — direction applied when the trigger is in its inactive state
void setInactiveState(char *val) { if (checkState(val) == NA_DIR) return; SendACK; srdata.inactiveState = checkState(val); }
void getInactiveState(void)      { reportState(srdata.inactiveState); }

// Parse a hex string into a 16-bit relay bit pattern and store it.
void writePattern(char *val, uint16_t *pattern)
{
  int i;
  if (sscanf(val, "%x", &i) == 1) { SendACK; *pattern = i & 0xFFFF; }
  else BADARG;
}

// Send ACK then the pattern as a 4-digit hex string.
void reportPattern(uint16_t pattern)
{
  SendACKonly;
  if (SerialMute) return;
  serial->printf("%4X\n\r", pattern);
}

// SFWDP/GFWDP — forward relay bit pattern
void setFWDpattern(char *val)  { writePattern(val, &srdata.fwdPattern); }
void getFWDpattern(void)       { reportPattern(srdata.fwdPattern); }

// SREVP/GREVP — reverse relay bit pattern
void setREVpattern(char *val)  { writePattern(val, &srdata.revPattern); }
void getREVpattern(void)       { reportPattern(srdata.revPattern); }

// SOPENP/GOPENP — open-state relay bit pattern
void setOPENpattern(char *val) { writePattern(val, &srdata.openPattern); }
void getOPENpattern(void)      { reportPattern(srdata.openPattern); }

// DEBUG — hook for ad-hoc diagnostic code during development
void Debug(int i)
{
  (void)i; // parameter reserved for future use
}
