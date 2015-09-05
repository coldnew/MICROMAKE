#include "LiquidCrystalRus.h"

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <avr/pgmspace.h>

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

// it is a Russian alphabet translation
// except 0401 --> 0xa2 = 鈺