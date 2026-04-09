// Hardware.h — ADC/DAC channel descriptors and hardware utility declarations
//
// Defines the ADCchan and DACchan calibration structs used throughout the
// firmware to convert between raw counts and engineering-unit values.
// Also declares the TC5-based scan timer, the MAX14802 SPI relay driver,
// and the FLASH programming routine.
//
// Gordon Anderson — GAA Custom Electronics, LLC

#ifndef Hardware_h
#define Hardware_h

#include <stdint.h>

// ── Digital I/O pin assignments ──────────────────────────────────────────────
#define TRIG  1   // External opto-isolated trigger input
#define LTCH  0   // MAX14802 latch strobe output

// ── Calibration channel descriptors ─────────────────────────────────────────

// ADC channel: maps raw counts → engineering-unit value via  value = (counts − b) / m
typedef struct
{
  int8_t  Chan;   // ADC channel number (0..max); MSB set → native M0 ADC channel
  float   m;      // Slope:  ADCcounts = m * value + b
  float   b;      // Offset: value    = (ADCcounts − b) / m
} ADCchan;

// DAC channel: maps engineering-unit value → raw counts via  counts = m * value + b
typedef struct
{
  int8_t  Chan;   // DAC channel number (0..max channels for chip)
  float   m;      // Slope:  DACcounts = m * value + b
  float   b;      // Offset: value    = (DACcounts − b) / m
} DACchan;

// ── Calibration conversion functions ─────────────────────────────────────────
float Counts2Value(int Counts, DACchan *DC);
float Counts2Value(int Counts, ADCchan *ad);
int   Value2Counts(float Value, DACchan *DC);
int   Value2Counts(float Value, ADCchan *ac);

// ── TC5 scan-timer control ────────────────────────────────────────────────────
// Configures TC5 to fire an interrupt at the requested frequency and calls
// the supplied callback from within the ISR.  Returns the actual frequency set.
int  tcConfigure(int freq, void (*callback)(void));
void tcStartCounter(void);
void tcDisable(void);
void tcReset(void);
bool tcIsSyncing(void);

// ── FLASH programming over serial ────────────────────────────────────────────
// Receives a file from the USB host in hex and burns it to the specified
// FLASH address.  Protocol: address (hex), size (decimal), then hex data bytes
// followed by an 8-bit CRC.
void ProgramFLASH(char *Faddress, char *Fsize);

// ── MAX14802 16-channel SPI relay driver ─────────────────────────────────────
// Shifts a 16-bit pattern into the MAX14802 and optionally latches the outputs.
void MAX14802(int val, bool Latch = true);

#endif // Hardware_h
