// Hardware.cpp — ADC/DAC conversions, TC5 scan timer, MAX14802 SPI driver,
//                and FLASH programming over USB serial.
//
// Gordon Anderson — GAA Custom Electronics, LLC

#include "Hardware.h"
#include "AtomicBlock.h"
#include <Arduino.h>
#include "SPI.h"
#include <wiring_private.h>
#include <assert.h>
#include <FlashStorage.h>
#include "Serial.h"

// ── Calibration conversions ───────────────────────────────────────────────────

// Convert raw ADC/DAC counts to an engineering-unit value using the stored
// slope (m) and offset (b):  value = (counts − b) / m
float Counts2Value(int Counts, DACchan *DC)
{
  return (Counts - DC->b) / DC->m;
}

float Counts2Value(int Counts, ADCchan *ad)
{
  return (Counts - ad->b) / ad->m;
}

// Convert an engineering-unit value to raw DAC/ADC counts, clamped to [0, 65535]
int Value2Counts(float Value, DACchan *DC)
{
  int counts = (Value * DC->m) + DC->b;
  if (counts < 0)     counts = 0;
  if (counts > 65535) counts = 65535;
  return counts;
}

int Value2Counts(float Value, ADCchan *ac)
{
  int counts = (Value * ac->m) + ac->b;
  if (counts < 0)     counts = 0;
  if (counts > 65535) counts = 65535;
  return counts;
}

// ── TC5 scan-timer helpers ────────────────────────────────────────────────────
// Helpers are defined before tcConfigure so no forward declarations are needed.
// Adapted from: https://gist.github.com/nonsintetic/ad13e70f164801325f5f552f84306d6f

// Returns true while TC5 is still synchronising its register writes to the
// peripheral clock domain (must be polled before enabling or resetting).
bool tcIsSyncing()
{
  return TC5->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY;
}

// Reset TC5 via software reset and wait for the reset to complete.
void tcReset()
{
  TC5->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
  while (tcIsSyncing());
  while (TC5->COUNT16.CTRLA.bit.SWRST); // wait until hardware clears SWRST
}

// Enable TC5 and wait until the enable has synchronised.
void tcStartCounter()
{
  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;
  while (tcIsSyncing());
}

// Disable TC5 and wait until the change has synchronised.
void tcDisable()
{
  TC5->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;
  while (tcIsSyncing());
}

// ── TC5 interrupt service routine ────────────────────────────────────────────

// Pointer to the user-supplied scan callback; NULL disables the callback.
void (*callback_func)(void) = NULL;

// TC5 ISR — called at the configured frequency.  Clears the match-capture
// interrupt flag (MC0) after invoking the callback.
void TC5_Handler(void)
{
  if (callback_func != NULL) callback_func();
  TC5->COUNT16.INTFLAG.bit.MC0 = 1; // required flag clear; do not remove
}

// ── TC5 configuration ─────────────────────────────────────────────────────────

// Configure TC5 to fire at <freq> Hz and call <callback> from the ISR.
// The function selects the coarsest prescaler whose resulting count fits in
// the 16-bit compare register, then arms the NVIC at priority 8.
// Returns the actual frequency achieved (may differ slightly from requested).
int tcConfigure(int freq, void (*callback)(void))
{
  callback_func = callback;

  // Feed GCLK0 (48 MHz system clock) into TC4/TC5
  GCLK->CLKCTRL.reg = (uint16_t)(GCLK_CLKCTRL_CLKEN |
                                  GCLK_CLKCTRL_GEN_GCLK0 |
                                  GCLK_CLKCTRL_ID(GCM_TC4_TC5));
  while (GCLK->STATUS.bit.SYNCBUSY);

  tcReset();

  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16;   // 16-bit mode
  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;   // match-frequency waveform

  // Walk through available prescalers (1, 2, 4, 8, 16, 64, 256, 1024),
  // stopping at the first one whose count fits in 16 bits.
  int targetCount = VARIANT_MCK / freq;
  if      ((targetCount /= 1)    <= 65535) TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1    | TC_CTRLA_ENABLE;
  else if ((targetCount /= 2)    <= 65535) TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV2    | TC_CTRLA_ENABLE;
  else if ((targetCount /= 2)    <= 65535) TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV4    | TC_CTRLA_ENABLE;
  else if ((targetCount /= 2)    <= 65535) TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV8    | TC_CTRLA_ENABLE;
  else if ((targetCount /= 2)    <= 65535) TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV16   | TC_CTRLA_ENABLE;
  else if ((targetCount /= 4)    <= 65535) TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV64   | TC_CTRLA_ENABLE;
  else if ((targetCount /= 4)    <= 65535) TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV256  | TC_CTRLA_ENABLE;
  else if ((targetCount /= 4)    <= 65535) TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1024 | TC_CTRLA_ENABLE;

  TC5->COUNT16.CC[0].reg = targetCount;

  // Configure NVIC for TC5 at priority 8 (below highest-priority ISRs)
  NVIC_DisableIRQ(TC5_IRQn);
  NVIC_ClearPendingIRQ(TC5_IRQn);
  NVIC_SetPriority(TC5_IRQn, 8);
  NVIC_EnableIRQ(TC5_IRQn);

  TC5->COUNT16.INTENSET.bit.MC0 = 1; // enable match-capture interrupt
  while (tcIsSyncing());

  return VARIANT_MCK / targetCount;
}

// ── CRC-8 helpers ─────────────────────────────────────────────────────────────
// Polynomial 0x1D (implicit leading 1 → 0x11D = CRC-8/MAXIM).

// Update a running CRC byte with one new data byte.
void ComputeCRCbyte(byte *crc, byte by)
{
  byte generator = 0x1D;

  *crc ^= by;
  for (int j = 0; j < 8; j++)
  {
    if ((*crc & 0x80) != 0) *crc = ((*crc << 1) ^ generator);
    else                    *crc <<= 1;
  }
}

// Compute the CRC-8 of an arbitrary byte buffer.
byte ComputeCRC(byte *buf, int bsize)
{
  byte generator = 0x1D;
  byte crc       = 0;

  for (int i = 0; i < bsize; i++)
  {
    crc ^= buf[i];
    for (int j = 0; j < 8; j++)
    {
      if ((crc & 0x80) != 0) crc = ((crc << 1) ^ generator);
      else                   crc <<= 1;
    }
  }
  return crc;
}

// ── FLASH programming over USB serial ────────────────────────────────────────
//
// Protocol (host → device):
//   1. Host sends: <hex address>,<decimal byte count>\n
//   2. Device replies ACK if address/size are acceptable, else NAK.
//   3. Host sends each byte as two ASCII hex digits (no delimiter).
//   4. Host sends '\n' then the CRC (decimal) followed by '\n'.
//   5. Device verifies the CRC and replies ACK on success, NAK on failure.
//
// Data is written in 256-byte blocks; a read-back verify is performed after
// each block write.  fbuf/vbuf are static to avoid consuming stack space on
// the 32 KB SAMD21.
void ProgramFLASH(char *Faddress, char *Fsize)
{
  // Static buffers avoid 512-byte stack allocation on the 32 KB SAMD21
  static byte fbuf[256]; // write staging buffer
  static byte vbuf[256]; // read-back verification buffer

  // These are reset on every call and do not need to retain state between calls
  uint32_t FlashAddress = strtol(Faddress, 0, 16);
  String   sToken       = Fsize;
  int      numBytes     = sToken.toInt();
  int      fi           = 0;    // index into fbuf for the current block
  int      val, tcrc;
  char     c, buf[3], *Token;
  byte     crc          = 0;
  uint32_t start;

  SendACK;
  FlashClass fc((void *)FlashAddress, numBytes);

  for (int i = 0; i < numBytes; i++)
  {
    start = millis();

    // Read two ASCII hex digits from the ring buffer, waiting up to 10 s each
    while ((c = RB_Get(&RB)) == 0xFF) { ProcessSerial(false); if (millis() > start + 10000) goto TimeoutExit; }
    buf[0] = c;
    while ((c = RB_Get(&RB)) == 0xFF) { ProcessSerial(false); if (millis() > start + 10000) goto TimeoutExit; }
    buf[1] = c;
    buf[2] = 0;

    sscanf(buf, "%x", &val);
    fbuf[fi++] = val;
    ComputeCRCbyte(&crc, val);

    if (fi == 256)
    {
      // Full 256-byte block: erase, write, then verify
      fi = 0;
      noInterrupts();
      fc.erase((void *)FlashAddress, 256);
      fc.write((void *)FlashAddress, fbuf, 256);
      fc.read ((void *)FlashAddress, vbuf, 256);
      for (int j = 0; j < 256; j++)
      {
        if (fbuf[j] != vbuf[j])
        {
          interrupts();
          serial->println("FLASH data write error!");
          SendNAK;
          return;
        }
      }
      interrupts();
      FlashAddress += 256;
      serial->println("Next");
    }
  }

  // Write any remaining partial block
  if (fi > 0)
  {
    noInterrupts();
    fc.erase((void *)FlashAddress, fi);
    fc.write((void *)FlashAddress, fbuf, fi);
    fc.read ((void *)FlashAddress, vbuf, fi);
    for (int j = 0; j < fi; j++)
    {
      if (fbuf[j] != vbuf[j])
      {
        interrupts();
        serial->println("FLASH data write error!");
        SendNAK;
        return;
      }
    }
    interrupts();
  }

  // Expect '\n', then CRC token, then '\n'
  start = millis();
  while ((c = RB_Get(&RB)) == 0xFF) { ProcessSerial(false); if (millis() > start + 10000) goto TimeoutExit; }
  if (c == '\n')
  {
    while ((Token = GetToken(true)) == NULL) { ProcessSerial(false); if (millis() > start + 10000) goto TimeoutExit; }
    sscanf(Token, "%d", &tcrc);
    while ((Token = GetToken(true)) == NULL) { ProcessSerial(false); if (millis() > start + 10000) goto TimeoutExit; }
    if ((Token[0] == '\n') && (crc == tcrc))
    {
      serial->println("File received from host and written to FLASH.");
      SendACK;
      return;
    }
  }

  serial->println("\nError during file receive from host!");
  SendNAK;
  return;

TimeoutExit:
  serial->println("\nFile receive from host timedout!");
  SendNAK;
}

// ── MAX14802 16-channel SPI relay driver ─────────────────────────────────────

// Pulse the latch strobe to transfer the shift register contents to the relay
// output latches.
void MAX14802_Latch(void)
{
  digitalWrite(LTCH, LOW);
  digitalWrite(LTCH, HIGH);
}

// Shift a 16-bit relay pattern into the MAX14802 over SPI.
// On the first call the SPI bus and control pins are initialised.
// If Latch is true the outputs are committed immediately.
void MAX14802(int val, bool Latch)
{
  static bool inited = false;

  if (!inited)
  {
    SPI.begin();
    SPI.setClockDivider(0x20); // ~1.5 MHz SPI clock
    pinMode(0, OUTPUT);
    digitalWrite(0, HIGH);    // latch idle high
    pinMode(2, OUTPUT);
    digitalWrite(2, LOW);     // clear active-low, hold inactive
    inited = true;
  }

  SPI.transfer16(val);
  if (Latch) MAX14802_Latch();
}
