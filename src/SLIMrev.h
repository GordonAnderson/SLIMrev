// SLIMrev.h — SLIM reverser firmware declarations
//
// Defines the persistent configuration structure (SLIMREVdata), direction
// enum, and prototypes for the host command functions implemented in SLIMrev.cpp.
//
// Gordon Anderson — GAA Custom Electronics, LLC

#ifndef SLIMrev_h
#define SLIMrev_h

#include "Hardware.h"

// ── Constants ─────────────────────────────────────────────────────────────────
#define FILTER     0.1          // IIR low-pass filter coefficient (not currently used)
#define SIGNATURE  0xAA55A5A5  // Magic value that marks valid FLASH-stored config

#define ESC  27  // ASCII Escape
#define ENQ   5  // ASCII Enquiry

// ── Switch direction ──────────────────────────────────────────────────────────
// Represents the three states the MAX14802 relay switch can be placed in,
// plus NA_DIR used as a sentinel / "unknown" value.
enum Direction
{
  FWD_DIR,   // Forward — apply fwdPattern
  REV_DIR,   // Reverse — apply revPattern
  OPEN_DIR,  // Open    — apply openPattern (all relays open)
  NA_DIR     // Not applicable / parse error sentinel
};

// ── Persistent configuration ──────────────────────────────────────────────────
// Stored in FLASH via FlashStorage.  The Signature field is checked on boot;
// if it does not match SIGNATURE the struct is reset to factory defaults.
typedef struct
{
  int16_t   Size;          // Size of this struct in bytes (for version checking)
  char      Name[20];      // Board name string, e.g. "SLIMrev"
  int8_t    Rev;           // Firmware/board revision number

  // External trigger settings
  bool      Ext;           // true → honour the external opto-isolated input
  bool      ActiveHigh;    // true → trigger pin HIGH maps to activeState
  bool      Fwd;           // true → switch is currently in the forward direction

  // Run-time switch state
  Direction dir;           // Current switch direction
  Direction activeState;   // Switch state when trigger is active
  Direction inactiveState; // Switch state when trigger is inactive

  // Relay bit patterns written to the MAX14802 for each direction
  uint16_t  fwdPattern;
  uint16_t  revPattern;
  uint16_t  openPattern;

  uint      Signature;     // Must equal SIGNATURE for the struct to be considered valid
} SLIMREVdata;

// Exposed so Serial.cpp can include srdata in the command table without a
// separate extern in every file that uses it.
extern bool MonitorFlag;

// ── Function prototypes ───────────────────────────────────────────────────────

// Main serial command dispatcher; scan=false skips command processing (used
// while waiting inside blocking operations such as ProgramFLASH).
void ProcessSerial(bool scan = true);

// Host commands — direction and pattern control
void setFWD(char *val);
void setState(char *val);
void getState(void);
void setActiveState(char *val);
void getActiveState(void);
void setInactiveState(char *val);
void getInactiveState(void);
void setFWDpattern(char *val);
void getFWDpattern(void);
void setREVpattern(char *val);
void getREVpattern(void);
void setOPENpattern(char *val);
void getOPENpattern(void);

#endif // SLIMrev_h
