#pragma once
#include <inttypes.h>

__attribute__((section(".apitable_spm_func"))) __attribute__((noinline)) void BootloaderAPI_ErasePageWrite(uint16_t Address);
__attribute__((section(".apitable_spm_func"))) __attribute__((noinline)) void BootloaderAPI_FillWord(uint16_t Address, const uint16_t word);
