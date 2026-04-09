// Serial.h — USB serial command processor declarations
//
// Declares the ring-buffer type, the command-dispatch table types, and all
// functions used by the serial command processor implemented in Serial.cpp.
//
// Gordon Anderson — GAA Custom Electronics, LLC

#ifndef SERIAL_H_
#define SERIAL_H_

#include <Arduino.h>
#include "SLIMrev.h"
#include <ThreadController.h>
#include <FlashStorage.h>
#include <FlashAsEEPROM.h>

// Active serial stream — defaults to the hardware USB CDC Serial port.
// Can be redirected to another Stream for debugging over a different interface.
extern Stream *serial;

// When true, all ACK/NAK/data responses are suppressed.
extern bool SerialMute;

// The working copy of the persistent device configuration (defined in SLIMrev.cpp).
extern SLIMREVdata srdata;

// ── Ring buffer ───────────────────────────────────────────────────────────────
#define RB_BUF_SIZE  4096  // Receive ring buffer capacity in bytes

// ── Response macros ───────────────────────────────────────────────────────────
// These are macros so they can be used as single-statement expressions inside
// command handler functions.  All output is gated on !SerialMute.
extern char *SelectedACKonlyString;

#define SendNAK     { if (!SerialMute) serial->write("\x15?\n\r"); }
#define SendACK     { if (!SerialMute) serial->write("\x06\n\r"); }
#define SendACKonly { if (!SerialMute) serial->write(SelectedACKonlyString); }
#define SendERR     { if (!SerialMute) serial->write("\x15?\n\r"); }
#define SendBSY     { if (!SerialMute) serial->write("\x15?\n\r"); }

// ── Flow-control / protocol bytes ─────────────────────────────────────────────
#define XON  0x11  // Resume transmission
#define XOFF 0x13  // Pause transmission
#define ACK  0x06  // Positive acknowledgement
#define NAK  0x15  // Negative acknowledgement

// ── Receive ring buffer ───────────────────────────────────────────────────────
typedef struct
{
  char  Buffer[RB_BUF_SIZE];
  int   Tail;      // write index
  int   Head;      // read index
  int   Count;     // bytes currently in buffer
  int   Commands;  // number of complete commands waiting (delimited by \n, \r, ;)
} Ring_Buffer;

// ── Command table types ───────────────────────────────────────────────────────

// Describes how the command handler interprets the pointers union below
enum CmdTypes
{
  CMDstr,          // Read/write a char* string
  CMDint,          // Read/write an int
  CMDfloat,        // Read/write a float
  CMDbool,         // Read/write a bool as "TRUE" / "FALSE"
  CMDfunction,     // Call a void function with 0, 1, or 2 int args
  CMDfunctionStr,  // Call a void function with 1 or 2 char* args
  CMDfunctionLine, // Call a void function that reads its own tokens from the ring buffer
  CMDfun2int1flt,  // Call a void function(int, int, float)
  CMDlongStr,      // Read a variable-length string directly into a char* buffer
  CMDna
};

// Parser state machine states
enum PCstates
{
  PCcmd,      // Waiting for a command token
  PCarg1,     // Waiting for first argument
  PCarg2,     // Waiting for second argument
  PCarg3,     // Waiting for third argument
  PCargStr,   // Waiting for a string argument
  PCargLine,  // Waiting for a full line (no tokenisation)
  PCend,      // Waiting for terminator (\n or ;) before dispatching
  PCna
};

// Polymorphic pointer used in the command table
union functions
{
  char  *charPtr;
  int   *intPtr;
  float *floatPtr;
  bool  *boolPtr;
  void  (*funcVoid)();
  void  (*func1int)(int);
  void  (*func2int)(int, int);
  void  (*func1str)(char *);
  void  (*func2str)(char *, char *);
  void  (*func2int1flt)(int, int, float);
};

// One entry in the command dispatch table
typedef struct
{
  const char      *Cmd;       // ASCII command token (e.g. "GVER")
  enum  CmdTypes   Type;      // How to interpret NumArgs and pointers
  int              NumArgs;   // Number of arguments expected
  union functions  pointers;  // Data pointer or function pointer
} Commands;

// ── Globals ───────────────────────────────────────────────────────────────────
extern Ring_Buffer  RB;        // Main receive ring buffer
extern const char   Version[]; // Firmware version string (PROGMEM)

// ── System command prototypes (implemented in SLIMrev.cpp) ───────────────────
void SaveSettings(void);
void RestoreSettings(void);
void Software_Reset(void);
void FormatFLASH(void);
void Debug(int i);

// ── Serial subsystem prototypes ───────────────────────────────────────────────
void   SerialInit(void);
char  *GetToken(bool ReturnComma);
int    ProcessCommand(void);
void   RB_Init(Ring_Buffer *);
int    RB_Size(Ring_Buffer *);
char   RB_Put(Ring_Buffer *, char);
char   RB_Get(Ring_Buffer *);
char   RB_Next(Ring_Buffer *);
int    RB_Commands(Ring_Buffer *);
void   PutCh(char ch);
void   Mute(char *cmd);
void   GetCommands(void);
void   DelayCommand(int dtime);
void   SetThreadEnable(char *, char *);
void   ListThreads(void);
void   ProgramGOTO(char *location);
void   LoadAltRev(void);
void   WhereAmI(void);
void   EraseUpper(void);

#endif // SERIAL_H_
