#pragma once

#include <inttypes.h>
#include <avr/pgmspace.h>

#define BOOT_START_ADDR 0x7000

extern const uint8_t newBootloader[4096] PROGMEM;
extern const uint8_t oldBootloader[4096] PROGMEM;
