// Serial.cpp — USB serial command processor
//
// Implements the ring buffer, a non-blocking token parser, and a state-machine
// command dispatcher.  Commands are comma-delimited ASCII strings terminated
// by '\n' or ';'.  The dispatch table (CmdArray) maps command tokens to data
// pointers or function pointers using the types defined in Serial.h.
//
// Gordon Anderson — GAA Custom Electronics, LLC

#include "Arduino.h"
#include "string.h"
#include "Serial.h"
#include "Errors.h"
#include <FlashStorage.h>
#include <Wire.h>
#include <SPI.h>
#include "reset.h"

extern ThreadController control;

// ── Module-level state ────────────────────────────────────────────────────────

Stream *serial     = &Serial; // Default output stream; reassign to redirect I/O
bool    SerialMute = false;

#define MaxToken 20

// Working token buffer shared by GetToken() and ProcessCommand()
char Token[MaxToken];
char Sarg1[MaxToken];
char Sarg2[MaxToken];
unsigned char Tptr; // write index into Token[]

int ErrorCode = 0; // Last communication error (read via GERR command)

Ring_Buffer RB;    // Main receive ring buffer

// ACK-only string: plain ACK, or ACK with a leading comma in echo mode
char *ACKonlyString1       = (char *)"\x06";
char *ACKonlyString2       = (char *)",\x06";
char *SelectedACKonlyString = ACKonlyString1;

bool echoMode = false; // When true, each command token is echoed back to the host

// ── Command dispatch table ────────────────────────────────────────────────────
// Each entry maps an ASCII command token to either a data pointer (for simple
// read/write commands) or a function pointer (for action commands).
// The cast to (char *) is required because the union's charPtr is used as a
// universal storage type for all pointer variants.
Commands CmdArray[] = {
// ── General system commands ───────────────────────────────────────────────────
  {"GVER",    CMDstr,         0, (char *)Version},              // Report firmware version string
  {"GERR",    CMDint,         0, (char *)&ErrorCode},           // Report last error code
  {"MUTE",    CMDfunctionStr, 1, (char *)Mute},                 // Suppress/restore serial responses: ON | OFF
  {"ECHO",    CMDbool,        1, (char *)&echoMode},            // Echo command tokens back to host: TRUE | FALSE
  {"DELAY",   CMDfunction,    1, (char *)DelayCommand},         // Blocking delay in milliseconds
  {"GCMDS",   CMDfunction,    0, (char *)GetCommands},          // List all command tokens
  {"RESET",   CMDfunction,    0, (char *)Software_Reset},       // Reboot the microcontroller
  {"SAVE",    CMDfunction,    0, (char *)SaveSettings},         // Persist current config to FLASH
  {"RESTORE", CMDfunction,    0, (char *)RestoreSettings},      // Reload config from FLASH
  {"FORMAT",  CMDfunction,    0, (char *)FormatFLASH},          // Reset FLASH config to factory defaults
  {"DEBUG",   CMDfunction,    1, (char *)Debug},                // Ad-hoc diagnostic hook (arg varies)
  {"THREADS", CMDfunction,    0, (char *)ListThreads},          // List threads: name, ID, interval, state, runtime
  {"STHRDENA",CMDfunctionStr, 2, (char *)SetThreadEnable},      // Enable/disable a named thread: name, TRUE|FALSE
  {"ARBPGM",  CMDfunctionStr, 2, (char *)ProgramFLASH},         // Program FLASH from host file (ARB path)
  {"M0PGM",   CMDfunctionStr, 2, (char *)ProgramFLASH},         // Program FLASH from host file (M0 path)
  {"GOTO",    CMDfunctionStr, 1, (char *)ProgramGOTO},          // Jump to hex address (e.g. "0x20000")
  {"WHERE",   CMDfunctionStr, 0, (char *)WhereAmI},             // Report current function address in hex
  {"ERASEU",  CMDfunctionStr, 0, (char *)EraseUpper},           // Erase upper FLASH bank (0x20000+)
// ── SLIM reverser commands ────────────────────────────────────────────────────
  {"SENAEXT", CMDbool,        1, (char *)&srdata.Ext},          // Enable external trigger input: TRUE | FALSE
  {"GENAEXT", CMDbool,        0, (char *)&srdata.Ext},          // Query external trigger enable state
  {"SAHIGH",  CMDbool,        1, (char *)&srdata.ActiveHigh},   // Set trigger polarity: TRUE = active-high
  {"GAHIGH",  CMDbool,        0, (char *)&srdata.ActiveHigh},   // Query trigger polarity
  {"SFWD",    CMDfunctionStr, 1, (char *)setFWD},               // Force direction: TRUE = FWD, FALSE = REV
  {"GFWD",    CMDbool,        0, (char *)&srdata.Fwd},          // Query forward flag
  // Rev 1.1 additions
  {"SSTATE",  CMDfunctionStr, 1, (char *)setState},             // Set switch state: FWD | REV | OPEN
  {"GSTATE",  CMDfunction,    0, (char *)getState},             // Report switch state
  {"SASTATE", CMDfunctionStr, 1, (char *)setActiveState},       // Set state when trigger is active
  {"GASTATE", CMDfunction,    0, (char *)getActiveState},       // Report state when trigger is active
  {"SISTATE", CMDfunctionStr, 1, (char *)setInactiveState},     // Set state when trigger is inactive
  {"GISTATE", CMDfunction,    0, (char *)getInactiveState},     // Report state when trigger is inactive
  {"SFWDP",   CMDfunctionStr, 1, (char *)setFWDpattern},        // Set forward relay bit pattern (16-bit hex)
  {"GFWDP",   CMDfunction,    0, (char *)getFWDpattern},        // Report forward relay bit pattern
  {"SREVP",   CMDfunctionStr, 1, (char *)setREVpattern},        // Set reverse relay bit pattern (16-bit hex)
  {"GREVP",   CMDfunction,    0, (char *)getREVpattern},        // Report reverse relay bit pattern
  {"SOPENP",  CMDfunctionStr, 1, (char *)setOPENpattern},       // Set open-state relay bit pattern (16-bit hex)
  {"GOPENP",  CMDfunction,    0, (char *)getOPENpattern},       // Report open-state relay bit pattern
// ── End of table marker ───────────────────────────────────────────────────────
  {0},
};

// ── Utility / system commands ─────────────────────────────────────────────────

// Report the load address of this function as a proxy for the firmware bank
// currently executing (used to distinguish lower vs. upper FLASH banks).
void WhereAmI(void)
{
  uint32_t addr = (uint32_t)&WhereAmI;
  serial->println(addr, 16);
}

// Erase the upper FLASH bank (starting at 0x20000).
// If this firmware is already running from the upper bank, a full peripheral
// shutdown is performed first so the erase does not corrupt live state.
void EraseUpper(void)
{
  uint32_t addr = (uint32_t)&EraseUpper;
  if (addr < 0x20000)
  {
    // Running from lower bank — safe to erase upper bank directly
    FlashClass fc((void *)0x20000, 8);
    fc.erase((void *)0x20000, 8);
    return;
  }

  // Running from upper bank — shut down all peripherals before erasing
  Serial.end();
  USB->DEVICE.CTRLA.bit.SWRST = 1;
  while (USB->DEVICE.CTRLA.bit.SWRST == 1);

  SPI.end();
  Wire.end();

  // Stop all timer/counter peripherals
  TCC0->CTRLA.bit.SWRST  = 1;
  TCC1->CTRLA.bit.SWRST  = 1;
  TCC2->CTRLA.bit.SWRST  = 1;
  TC3->COUNT16.CTRLA.reg = 1;
  TC4->COUNT16.CTRLA.reg = 1;
  TC5->COUNT16.CTRLA.reg = 1;

  // Stop the SysTick timer
  SysTick->LOAD = 0;
  SysTick->VAL  = 0;
  SysTick->CTRL = 0;
  NVIC_DisableIRQ(SysTick_IRQn);
  NVIC_ClearPendingIRQ(SysTick_IRQn);

  FlashClass fc((void *)0x20000, 8);
  fc.erase((void *)0x20000, 8);

  // Jump back to the lower bank reset vector
  volatile uint32_t *Start_Address = (uint32_t *)0x2000;
  __set_MSP(Start_Address[0]);
  SCB->VTOR = (0x2000 & SCB_VTOR_TBLOFF_Msk);
  asm("bx %0" :: "r"(Start_Address[1]));
}

// If pin A31 is pulled low AND this firmware is in the lower bank, jump to
// the alternate firmware at 0x20000 (if it appears programmed).
// Called at the very top of setup() to allow dual-bank firmware updates.
void LoadAltRev(void)
{
  // Configure PA31 as input with internal pull-up
  PORT->Group[PORTA].PINCFG[31].reg = (uint8_t)(PORT_PINCFG_INEN | PORT_PINCFG_PULLEN);
  PORT->Group[PORTA].DIRCLR.reg     = (uint32_t)(1 << 31);
  PORT->Group[PORTA].OUTSET.reg     = (uint32_t)(1 << 31);

  // Sample 100 times; if the pin is ever high the condition is not met → stay here
  for (int i = 0; i < 100; i++)
    if ((PORT->Group[PORTA].IN.reg & (1ul << 31)) != 0) return;

  // Only vector away if we are currently in the lower bank
  uint32_t addr = (uint32_t)&LoadAltRev;
  if (addr > 0x20000) return;

  ProgramGOTO((char *)"0x20000");
}

// Vector to the application at <location> (a hex address string) if it looks
// programmed (i.e. the reset vector is not 0xFFFFFFFF).
// Performs a full peripheral shutdown before jumping so no DMA or timer
// continues to run in the new firmware context.
void ProgramGOTO(char *location)
{
  uint32_t             addr          = strtol(location, NULL, 16);
  volatile uint32_t   *Start_Address = (uint32_t *)addr;

  if (Start_Address[1] == 0xFFFFFFFF) return; // unprogrammed — do nothing

  Serial.end();
  USB->DEVICE.CTRLA.bit.SWRST = 1;
  while (USB->DEVICE.CTRLA.bit.SWRST == 1);

  SPI.end();
  Wire.end();

  TCC0->CTRLA.bit.SWRST  = 1;
  TCC1->CTRLA.bit.SWRST  = 1;
  TCC2->CTRLA.bit.SWRST  = 1;
  TC3->COUNT16.CTRLA.reg = 1;
  TC4->COUNT16.CTRLA.reg = 1;
  TC5->COUNT16.CTRLA.reg = 1;

  SysTick->LOAD = 0;
  SysTick->VAL  = 0;
  SysTick->CTRL = 0;
  NVIC_DisableIRQ(SysTick_IRQn);
  NVIC_ClearPendingIRQ(SysTick_IRQn);

  __set_MSP(Start_Address[0]);
  SCB->VTOR = (addr & SCB_VTOR_TBLOFF_Msk);
  asm("bx %0" :: "r"(Start_Address[1]));
}

// ── Serial command dispatcher helpers ────────────────────────────────────────

// Send the full list of supported command tokens, one per line.
void GetCommands(void)
{
  SendACKonly;
  for (int i = 0; CmdArray[i].Cmd != 0; i++)
    serial->println((char *)CmdArray[i].Cmd);
}

void DelayCommand(int dtime)
{
  delay(dtime);
  SendACK;
}

// Enable or disable serial response suppression.  "ON" → mute, "OFF" → unmute.
void Mute(char *cmd)
{
  if (strcmp(cmd, "ON") == 0)       { SerialMute = true;  SendACK; return; }
  else if (strcmp(cmd, "OFF") == 0) { SerialMute = false; SendACK; return; }
  SetErrorCode(ERR_BADARG);
  SendNAK;
}

void SerialInit(void)
{
  Serial.begin(115200);
  RB_Init(&RB);
}

// ── Tokeniser ─────────────────────────────────────────────────────────────────

// Append one character to the shared Token[] buffer, clamping at MaxToken−1.
void Char2Token(char ch)
{
  Token[Tptr++] = ch;
  if (Tptr >= MaxToken) Tptr = MaxToken - 1;
}

// Arduino String overload — extract the Nth comma-delimited token from a String.
// TokenNum is 1-based; returns an empty String when the index is out of range.
String GetToken(String cmd, int TokenNum)
{
  int    i, j, k;
  String Token;

  cmd.trim();
  if (TokenNum <= 1)
  {
    if ((i = cmd.indexOf(',')) == -1) return cmd;
    return cmd.substring(0, i);
  }

  k = 0;
  for (i = 2; i <= TokenNum; i++)
  {
    if ((j = cmd.indexOf(',', k)) == -1) return "";
    k = j + 1;
  }
  Token = cmd.substring(k);
  Token.trim();
  if ((j = Token.indexOf(',')) == -1) return Token;
  Token = Token.substring(0, j);
  Token.trim();
  return Token;
}

// Non-blocking ring-buffer tokeniser.
// Reads characters from RB until a delimiter is found, then returns a pointer
// to the null-terminated Token[] buffer.  Returns NULL if no complete token is
// available yet.  If ReturnComma is false, lone comma tokens are suppressed.
char *GetToken(bool ReturnComma)
{
  unsigned char ch;

  while (1)
  {
    ch = RB_Next(&RB);
    if (ch == 0xFF) return NULL; // buffer empty — nothing to return yet

    if (Tptr >= MaxToken) Tptr = MaxToken - 1;

    if ((ch == '\n') || (ch == ';') || (ch == ':') ||
        (ch == ',')  || (ch == ']') || (ch == '['))
    {
      // Delimiter: if we have accumulated characters, terminate the token;
      // otherwise consume the delimiter itself as the token.
      if (Tptr != 0) ch = 0;
      else { Char2Token(RB_Get(&RB)); ch = 0; }
    }
    else
    {
      RB_Get(&RB); // advance the ring-buffer read pointer
    }

    Char2Token(ch);

    if (ch == 0)
    {
      Tptr = 0;
      if ((Token[0] == ',') && !ReturnComma) return NULL;
      return Token;
    }
  }
}

// ── Command executor ──────────────────────────────────────────────────────────

// Dispatch a fully-parsed command to its handler, passing the appropriate
// pre-converted arguments.  In echo mode the ACK-only string gains a leading
// comma so the host can parse the echoed command and its response together.
void ExecuteCommand(Commands *cmd, int arg1, int arg2,
                    char *args1, char *args2, float farg1)
{
  SelectedACKonlyString = echoMode ? ACKonlyString2 : ACKonlyString1;

  switch (cmd->Type)
  {
    case CMDbool:
      if (cmd->NumArgs == 0) // read
      {
        SendACKonly;
        if (!SerialMute)
          serial->println(*(cmd->pointers.boolPtr) ? "TRUE" : "FALSE");
      }
      if (cmd->NumArgs == 1) // write
      {
        if ((strcmp(args1, "TRUE") == 0) || (strcmp(args1, "FALSE") == 0))
        {
          *(cmd->pointers.boolPtr) = (strcmp(args1, "TRUE") == 0);
          SendACK;
          break;
        }
        SetErrorCode(ERR_BADARG);
        SendNAK;
      }
      break;

    case CMDstr:
      if (cmd->NumArgs == 0) { SendACKonly; if (!SerialMute) serial->println(cmd->pointers.charPtr); }
      if (cmd->NumArgs == 1) { strcpy(cmd->pointers.charPtr, args1); SendACK; }
      break;

    case CMDint:
      if (cmd->NumArgs == 0) { SendACKonly; if (!SerialMute) serial->println(*(cmd->pointers.intPtr)); break; }
      if (cmd->NumArgs == 1) { *(cmd->pointers.intPtr) = arg1; SendACK; break; }
      /* fall through to CMDfloat if neither branch matched */

    case CMDfloat:
      if (cmd->NumArgs == 0) { SendACKonly; if (!SerialMute) serial->println(*(cmd->pointers.floatPtr)); }
      if (cmd->NumArgs == 1) { *(cmd->pointers.floatPtr) = farg1; SendACK; }
      break;

    case CMDfunction:
      if (cmd->NumArgs == 0) cmd->pointers.funcVoid();
      if (cmd->NumArgs == 1) cmd->pointers.func1int(arg1);
      if (cmd->NumArgs == 2) cmd->pointers.func2int(arg1, arg2);
      break;

    case CMDfunctionStr:
      if (cmd->NumArgs == 0) cmd->pointers.funcVoid();
      if (cmd->NumArgs == 1) cmd->pointers.func1str(args1);
      if (cmd->NumArgs == 2) cmd->pointers.func2str(args1, args2);
      break;

    case CMDfun2int1flt:
      if (cmd->NumArgs == 3) cmd->pointers.func2int1flt(arg1, arg2, farg1);
      break;

    default:
      SendNAK;
      break;
  }
}

// ── Main command state machine ────────────────────────────────────────────────
//
// Non-blocking: each call advances the state machine by one token.
// Returns 0 if progress was made, -1 if there was nothing to do.
// The caller should loop calling ProcessCommand() while it returns 0.
int ProcessCommand(void)
{
  String sToken;
  char  *Token, ch;
  int    i;
  static int        arg1, arg2;
  static float      farg1;
  static PCstates   state;
  static int        CmdNum;
  static char       delimiter = 0;
  // Long-string mode: reads characters directly into a caller-provided buffer
  static char *lstrptr   = NULL;
  static int   lstrindex;
  static bool  lstrmode  = false;
  static int   lstrmax;

  // ── Special modes ─────────────────────────────────────────────────────────

  // CMDfunctionLine: wait for a complete line, then call the function with no args
  if (state == PCargLine)
  {
    if (RB.Commands <= 0) return -1;
    CmdArray[CmdNum].pointers.funcVoid();
    state = PCcmd;
    return 0;
  }

  // CMDlongStr: stream characters from the ring buffer directly into a buffer
  if (lstrmode)
  {
    ch = RB_Get(&RB);
    if (ch == 0xFF)  return -1;
    if (ch == ',')   return  0; // skip comma separators
    if (ch == '\r')  return  0; // skip bare CR
    if (ch == '\n')  { lstrptr[lstrindex++] = 0; lstrmode = false; return 0; }
    lstrptr[lstrindex++] = ch;
    return 0;
  }

  // ── Normal token processing ───────────────────────────────────────────────

  Token = GetToken(false);
  if (Token == NULL || Token[0] == 0) return -1;

  // Echo mode: print each non-newline token back to the host
  if (echoMode && !SerialMute)
  {
    if (strcmp(Token, "\n") != 0)
    {
      if (delimiter != 0) serial->write(delimiter);
      serial->print(Token);
    }
    delimiter = (strcmp(Token, "\n") == 0) ? 0 : ',';
  }

  switch (state)
  {
    case PCcmd:
      if (strcmp(Token, ";") == 0 || strcmp(Token, "\n") == 0) break;

      // Look up the token in the command table
      CmdNum = -1;
      for (i = 0; CmdArray[i].Cmd != 0; i++)
      {
        if (strcmp(Token, CmdArray[i].Cmd) == 0) { CmdNum = i; break; }
      }
      if (CmdNum == -1) { SetErrorCode(ERR_BADCMD); SendNAK; break; }

      if (CmdArray[i].Type == CMDfunctionLine) { state = PCargLine; break; }

      if (CmdArray[i].Type == CMDlongStr)
      {
        lstrptr    = CmdArray[i].pointers.charPtr;
        lstrindex  = 0;
        lstrmax    = CmdArray[i].NumArgs;
        lstrmode   = true;
        break;
      }

      state = (CmdArray[i].NumArgs > 0) ? PCarg1 : PCend;
      break;

    case PCarg1:
      Sarg1[0] = 0;
      sToken   = Token;
      sToken.trim();
      arg1     = sToken.toInt();
      farg1    = sToken.toFloat();
      strcpy(Sarg1, sToken.c_str());
      state = (CmdArray[CmdNum].NumArgs > 1) ? PCarg2 : PCend;
      break;

    case PCarg2:
      Sarg2[0] = 0;
      sToken   = Token;
      sToken.trim();
      arg2     = sToken.toInt();
      strcpy(Sarg2, sToken.c_str());
      state = (CmdArray[CmdNum].NumArgs > 2) ? PCarg3 : PCend;
      break;

    case PCarg3:
      sToken = Token;
      sToken.trim();
      farg1  = sToken.toFloat();
      state  = PCend;
      break;

    case PCend:
      // Require a clean terminator; anything else is a protocol error
      if (strcmp(Token, "\n") != 0 && strcmp(Token, ";") != 0)
      {
        state = PCcmd;
        SendNAK;
        break;
      }
      i      = CmdNum;
      CmdNum = -1;
      state  = PCcmd;
      ExecuteCommand(&CmdArray[i], arg1, arg2, Sarg1, Sarg2, farg1);
      break;

    default:
      state = PCcmd;
      break;
  }

  return 0;
}

// ── Ring buffer implementation ────────────────────────────────────────────────

void RB_Init(Ring_Buffer *rb)
{
  rb->Head     = 0;
  rb->Tail     = 0;
  rb->Count    = 0;
  rb->Commands = 0;
}

int RB_Size(Ring_Buffer *rb)     { return rb->Count; }
int RB_Commands(Ring_Buffer *rb) { return rb->Commands; }

// Insert a character into the ring buffer.
// Returns 0xFF if the buffer is full, 0 on success.
// Increments Commands for each line/command terminator received.
char RB_Put(Ring_Buffer *rb, char ch)
{
  if (rb->Count >= RB_BUF_SIZE) return 0xFF;

  rb->Buffer[rb->Tail] = ch;
  if (rb->Tail++ >= RB_BUF_SIZE - 1) rb->Tail = 0;
  rb->Count++;

  if (ch == ';' || ch == '\r' || ch == '\n') rb->Commands++;
  return 0;
}

// Remove and return the next character from the ring buffer.
// Maps '\r' → '\n' for consistent line-ending handling.
// Returns 0xFF if the buffer is empty.
char RB_Get(Ring_Buffer *rb)
{
  if (rb->Count == 0) { rb->Commands = 0; return 0xFF; }

  char ch = rb->Buffer[rb->Head];
  if (rb->Head++ >= RB_BUF_SIZE - 1) rb->Head = 0;
  rb->Count--;

  if (ch == '\r') ch = '\n';
  if (ch == ';' || ch == '\n') { rb->Commands--; if (rb->Commands < 0) rb->Commands = 0; }

  return ch;
}

// Peek at the next character without consuming it.
// Maps '\r' → '\n'.  Returns 0xFF if the buffer is empty.
char RB_Next(Ring_Buffer *rb)
{
  if (rb->Count == 0) return 0xFF;
  char ch = rb->Buffer[rb->Head];
  return (ch == '\r') ? '\n' : ch;
}

// Push one received character into the ring buffer.
void PutCh(char ch) { RB_Put(&RB, ch); }

// ── Thread management commands ────────────────────────────────────────────────

// List all registered threads: name, ID, interval (ms), enabled state, last run time (ms).
void ListThreads(void)
{
  SendACKonly;
  serial->println("Thread name,ID,Interval,Enabled,Run time");

  int     i = 0;
  Thread *t;
  while ((t = control.get(i++)) != NULL)
  {
    serial->print(t->getName());   serial->print(", ");
    serial->print(t->getID());     serial->print(", ");
    serial->print(t->getInterval()); serial->print(", ");
    serial->print(t->enabled ? "Enabled," : "Disabled,");
    serial->println(t->runTimeMs());
  }
}

// Enable or disable a thread by name.  state must be "TRUE" or "FALSE".
void SetThreadEnable(char *name, char *state)
{
  if (strcmp(state, "TRUE") != 0 && strcmp(state, "FALSE") != 0)
  {
    SetErrorCode(ERR_BADARG);
    SendNAK;
    return;
  }

  Thread *t = control.get(name);
  if (t == NULL) { SetErrorCode(ERR_BADARG); SendNAK; return; }

  SendACKonly;
  t->enabled = (strcmp(state, "TRUE") == 0);
}
