// Hooks.c — Arduino SAMD framework hook overrides
//
// The Arduino SAMD core calls sysTickHook() from its SysTick ISR each
// millisecond.  By implementing it here we intercept that call and forward
// it to the function pointer mySysTickHook, which is defined and set in
// SLIMrev.cpp.  This allows application code to run at 1 ms granularity
// without modifying the framework.

#include <Arduino.h>

extern void (*mySysTickHook)(void);

int sysTickHook(void)
{
  if (mySysTickHook != NULL) mySysTickHook();
  return (false); // returning false tells the framework to continue normally
}
