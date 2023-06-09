#include "BootloaderAPI.h"
#include <avr/boot.h>
#include <avr/interrupt.h>
#include "fastio.h"

// WARNING:
// Avoid changes to this region at all costs. Any mistake here could render the bootloader inoperable
// and further updates could be impossible.
// The code in this section must fit into a single page and must be relocatable.


/// @brief Method that takes an already filled page buffer and commits it to a page in flash
///        by performing page erase and page write in sequence.
/// @param Address the address of the page to be committed in bytes. 
///        It must be aligned to the page boundry and must not point to the BootloaderAPI page.
void BootloaderAPI_ErasePageWrite(uint16_t Address) {
	// If the address is not aligned to a page boundary, abort.
	if (Address & (SPM_PAGESIZE - 1))
		return;
	
	// If the address points to the page containing this code, abort.
	// It is not possible for the BootloaderAPI to update itself directly.

	// get the program counter of the code here.
	uint16_t PC;
	asm volatile (
		"rcall ." "\n\t" // push the PC to the stack by calling the next instruction
		"pop %B0" "\n\t" // pop the upper 8bit PC from the stack
		"pop %A0" "\n\t" // pop the lower 8bit PC from the stack
		: "=r" (PC)
	);
	PC <<= 1; // AVR words are 2 bytes long, so multiply the PC by 2 to get the byte address
	PC &= ~(SPM_PAGESIZE - 1); // get the page aligned address of the PC
	if (Address == PC) // check if we are overwriting the current page
		return;
	
	// These operations should always be done inside a critical section since the SPM (self-program) operation has tight timing requirements
	CRITICAL_SECTION_START;
	// Erase the target page
	boot_page_erase(Address);
	boot_spm_busy_wait();
	// Commit the page buffer to the target page
	boot_page_write(Address);
	boot_spm_busy_wait();
	// Enable the RWW section (the App part of the flash) after the SPM operations are completed.
	// This is needed in case the BootloaderAPI was called from the App section.
	boot_rww_enable();
	CRITICAL_SECTION_END;
}

/// @brief Method that fills the page buffer word by word. Must be followed by BootloaderAPI_ErasePageWrite() after the buffer is filled.
/// @param Address the address of the page to be committed in bytes
/// @param word the 16bit data to be added to the page buffer at Address
void BootloaderAPI_FillWord(uint16_t Address, const uint16_t word) {
	// Page filling should be done inside a critical section since the SPM operation has tight timing requirements
	CRITICAL_SECTION_START;
	boot_page_fill(Address, word);
	CRITICAL_SECTION_END;
}