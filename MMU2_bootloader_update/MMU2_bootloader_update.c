/**
 * @brief Small tool for updating the Original Prusa MMU2 bootloader via the firmware update mechanism and some exploits.
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include <stdio.h>
#include <stdbool.h>
#include <util/atomic.h>

#include "binFiles.h"

// Setting this to false allows the user to step through the code one instruction at a time as soon as the tracer is enabled.
// Used for debugging and for the development of the tracer.
#define TRACE_RUN_DEFAULT true

// Signatures constants used by the bootloader; must be maintained in the updated bootloader
#define CDC_SIGNATURE 0xDF00
#define LUFA_SIGNATURE 0xDCFB

// list of addresses where the tracer will halt execution and wait for user input to continue during debugging
static const uint16_t breakpoint_list[] PROGMEM = {
	// 0x7528,
	// 0x75dc,
	// 0x7ff4,
	// 0x7ff6,
};

// global variable used by tracer to feed the page data to the old bootloader
static const uint8_t * volatile pageData = NULL;

// symbol to a global variable used by the old bootloader; needed for the page write operation
extern uint32_t CurrAddress;

// symbols for the new bootloader api methods
__attribute__((noinline)) extern void BootloaderAPI_ErasePageWrite(uint16_t Address);
__attribute__((noinline)) extern void BootloaderAPI_FillWord(uint16_t Address, const uint16_t word);

// current MiniBootloader page symbols
extern uint8_t bootloader_version_major;
extern uint8_t bootloader_version_minor;
extern uint8_t bootloader_hwversion_major;
extern uint8_t bootloader_hwversion_minor;
extern uint32_t boot_start_addr;
extern uint16_t cdc_signature;
extern uint16_t lufa_signature;

// UART related methods
#define UART_BAUD_SELECT(baudRate,xtalCpu) (((float)(xtalCpu))/(((float)(baudRate))*8.0)-1.0+0.5)
static int uart1_putchar(char c, __attribute__((unused)) FILE *stream);
static char uart1_getchar(void);
// static void uart1_flushTX(void);

// Atomic varianta of the serial printing methods to avoid the tracer interrupt from interfering with the UART
#define aprintf_P(...) \
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { \
		printf_P(__VA_ARGS__); \
	} \

#define aputs_P(...) \
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { \
		puts_P(__VA_ARGS__); \
	} \

// program context used by the tracer
typedef struct __attribute__((packed)) {
	uint8_t sreg;
	uint8_t reg[32];
	uint8_t PCH;
	uint8_t PCL;
} TracerFrame_t;

// The main tracer handler method
__attribute__((noinline,used)) static void handle_ISR(TracerFrame_t *frame) {
	// get PC from stack frame
	// PC is the address of the next instruction that will be executed after this ISR
	uint16_t PC = ((uint16_t)frame->PCH << 8) | frame->PCL;
	PC <<= 1; // AVR words are 2 bytes long, so multiply the PC by 2 to get the byte address
	
	static bool tracer_continuous = TRACE_RUN_DEFAULT;
	
	// printf_P(PSTR("PC=%04X\n"), PC);
	uint16_t newPC = PC;
	
	// Handle the tracer events
	switch (PC) {
		case 0x0000: {
			// The reset vector of the App section was reached. This should never happen.
			// Announce the event and stop the tracer.
			puts_P(PSTR("RESET_vect reached"));
			tracer_continuous = false;
			EIMSK &= ~_BV(INT0);
		} break;
		
		case 0x7526: // end of old page erase
		case 0x762A: { // end of page write
			newPC = 0x7e9e; // jump to ret instruction
		} break;
		
		case 0x75e0: { // skip page validation which prevents the bootloader from writing to itself
			// This is what is skipped:
			// 75e0:	d7 01       	movw	r26, r14
			// 75e2:	c6 01       	movw	r24, r12
			// 75e4:	8f 77       	andi	r24, 0x7F	; 127
			// 75e6:	99 27       	eor	r25, r25
			// 75e8:	aa 27       	eor	r26, r26
			// 75ea:	bb 27       	eor	r27, r27
			// 75ec:	c1 14       	cp	r12, r1
			// 75ee:	f0 e7       	ldi	r31, 0x70	; 112
			// 75f0:	df 06       	cpc	r13, r31
			// 75f2:	e1 04       	cpc	r14, r1
			// 75f4:	f1 04       	cpc	r15, r1
			// 75f6:	c0 f4       	brcc	.+48     	; 0x7628 <_binary_bootloader_orig_bin_start+0x7628>
			// 75f8:	89 2b       	or	r24, r25
			// 75fa:	8a 2b       	or	r24, r26
			// 75fc:	8b 2b       	or	r24, r27
			// 75fe:	a1 f4       	brne	.+40     	; 0x7628 <_binary_bootloader_orig_bin_start+0x7628>
			newPC = 0x7600;
		} break;

		case 0x7564: {
			// Entering stupid non-atomic zone. Must disable the tracer for the timed SPMCSR+SPM operation.
			// We rely on the TIMER0_COMPA_vect interrupt to enable the tracer again after a one-shot operation.

			// These two instructions should have been in a critical section, but they weren't in the original bootloader.
			// 7564:	a0 92 57 00 	sts	0x0057, r10	; 0x800057 <_binary_bootloader_orig_bin_end+0x7f8057>
			// 7568:	e8 95       	spm

			// puts_P(PSTR("boot_page_fill_safe() reached. Tracer disabled"));
			// tracer_continuous = false;
			PORTD |= _BV(PIND0); // pause the tracer interrupt by setting PD0 to be HIGH
			OCR0A = 95; // this many clock cycles is the exact time needed for the non-atomic section to complete execution
			TCCR0A = _BV(WGM01); // ctc mode for timer to run until OCR0A is reached
			EIFR |= _BV(INTF0); // clear pending tracer interrupt flag
		} break;

		case 0x7134: { // end of FetchNextCommandByte()
			// Feed the page data by hijacking the FetchNextCommandByte() return value in the old bootloader.
			// Data is fed byte by byte. From the perspective of the old bootloader code, it's as if the data is coming from the USB RX endpoint.
			uint8_t data = pgm_read_byte(pageData++);
			frame->reg[24] = data; // r24 is the return byte of FetchNextCommandByte()
			// printf_P(PSTR("FetchNextCommandByte()->%02hX, remaining=%u\n"), data, frame->reg[28] | ((uint16_t)(frame->reg[29]) << 8));
		} break;
	}
	
	if (PC != newPC) {
		// printf_P(PSTR("Jump to %04X\n"), newPC);
		PC = newPC;
	}
	
	// Pause the tracer if a breakpoint is reached and wait for user input
	for (uint8_t i = 0; i != sizeof(breakpoint_list) / sizeof(breakpoint_list[0]); i++) {
		if (PC == pgm_read_word(breakpoint_list + i)) {
			puts_P(PSTR("Breakpoint hit"));
			tracer_continuous = false;
		}
	}
	
	// Pause the tracer if a break event happened and wait for user input
	if (!tracer_continuous) {
		// Fetch a character from the UART
		switch (uart1_getchar()) {
		case 'c': // Continue
			tracer_continuous = true;
			break;
		case 'n': // Execute the Next instruction
		default:
			break;
		}
	}
	
	// update PC in stack frame
	// This is the address of the instruction that will execute once this ISR returns.
	PC >>= 1; // AVR words are 2 bytes long, so divide the PC by 2 to get the word address
	frame->PCH = PC >> 8;
	frame->PCL = PC & 0xFF;
	// uart1_flushTX(); //flust the UART TX buffer
	
	// Start the one-shot timer as late as possible and try to keep the timing as consistent as possible
	if (TCCR0A & _BV(WGM01)) {
		TCNT0 = 0; // reset timer counter
		TIFR0 |= _BV(OCF0A); // clear pending interrupt flag
		TIMSK0 |= _BV(OCIE0A); // enable interrupt
		TCCR0B = _BV(CS00); // set prescaler to 1 so the timer is incremented every clock cycle
	}
}

// Tracer ISR vector
// This ISR is executed by hardware when the INT0 interrupt is triggered.
ISR(INT0_vect, ISR_NAKED) {
	// This asm code pushes the program context to the stack so that the handle_ISR can read and write to it.
	// r24:25 will hold a pointer to the stack frame of TracerFrame_t type.
	asm volatile (
		// Since the ISR was executed by hardware, the PC is already pushed to the stack.
		// Now push all registers in reverse order since the stack grows downwards.
		"push r31" "\n\t"
		"push r30" "\n\t"
		"push r29" "\n\t"
		"push r28" "\n\t"
		"push r27" "\n\t"
		"push r26" "\n\t"
		"push r25" "\n\t"
		"push r24" "\n\t"
		"push r23" "\n\t"
		"push r22" "\n\t"
		"push r21" "\n\t"
		"push r20" "\n\t"
		"push r19" "\n\t"
		"push r18" "\n\t"
		"push r17" "\n\t"
		"push r16" "\n\t"
		"push r15" "\n\t"
		"push r14" "\n\t"
		"push r13" "\n\t"
		"push r12" "\n\t"
		"push r11" "\n\t"
		"push r10" "\n\t"
		"push r9" "\n\t"
		"push r8" "\n\t"
		"push r7" "\n\t"
		"push r6" "\n\t"
		"push r5" "\n\t"
		"push r4" "\n\t"
		"push r3" "\n\t"
		"push r2" "\n\t"
		"push r1" "\n\t"
		"push r0" "\n\t"
		
		// Now that all registers have been pushed to stack, also save the SREG.
		"in r1, %2" "\n\t" // Load SREG from IO register
		"push r1" "\n\t" // Push SREG to stack
		"clr r1" "\n\t" // Make sure r1 stays cleared (__zero_reg__)
		
		// Load stack frame address
		"in r25, %0" "\n\t" // Load upper stack pointer bytes
		"in r24, %1" "\n\t" // Load lower stack pointer bytes
		"adiw r24, 1" "\n\t" // Increment the stack pointer by 1 because by default it points to the next free address on AVR
		
		// Call the tracer handler and pass the context pointer as a parameter (r24:r25)
		"call handle_ISR" "\n\t"
		
		"pop r1" "\n\t" // Pop SREG from stack
		"out %2, r1" "\n\t" // Store SREG in IO register

		// Pop the rest of the registers in the correct order
		"pop r0" "\n\t"
		"pop r1" "\n\t"
		"pop r2" "\n\t"
		"pop r3" "\n\t"
		"pop r4" "\n\t"
		"pop r5" "\n\t"
		"pop r6" "\n\t"
		"pop r7" "\n\t"
		"pop r8" "\n\t"
		"pop r9" "\n\t"
		"pop r10" "\n\t"
		"pop r11" "\n\t"
		"pop r12" "\n\t"
		"pop r13" "\n\t"
		"pop r14" "\n\t"
		"pop r15" "\n\t"
		"pop r16" "\n\t"
		"pop r17" "\n\t"
		"pop r18" "\n\t"
		"pop r19" "\n\t"
		"pop r20" "\n\t"
		"pop r21" "\n\t"
		"pop r22" "\n\t"
		"pop r23" "\n\t"
		"pop r24" "\n\t"
		"pop r25" "\n\t"
		"pop r26" "\n\t"
		"pop r27" "\n\t"
		"pop r28" "\n\t"
		"pop r29" "\n\t"
		"pop r30" "\n\t"
		"pop r31" "\n\t"

		// The context has been restored. Now return from the ISR.
		"reti" "\n\t"
		:
		: "I" (_SFR_IO_ADDR(SPH)),
		  "I" (_SFR_IO_ADDR(SPL)),
		  "I" (_SFR_IO_ADDR(SREG))
	);
}

// ISR vector of TIMER0. It is used for enabling the tracer again after a set number of clock cycles.
// This timer is needed because of a bug in the original code which left a critical section with interrupts enabled.
// The tracer would interfere with the bugged timed SPM sequence, so the tracer is disabled till after that section of code is executed.
ISR(TIMER0_COMPA_vect) {
	// Enable the tracer by setting the PD0 pin to be LOW
	PORTD &= ~_BV(PIND0);
	
	// Reset the timer registers to the default state
	TIMSK0 = 0;
	TCCR0A = 0;
	TCCR0B = 0;
}


static FILE _uart1io = {0};

static int uart1_putchar(char c, __attribute__((unused)) FILE *stream) {
	
	while (!(UCSR1A & _BV(UDRE1))); // Wait for the UART TX buffer to not be full
	UDR1 = c; // Push char to the UART TX buffer
	return 0;
}

static char uart1_getchar(void) {
	while (!(UCSR1A & _BV(RXC1))); // Wait for uart char to be received
	return UDR1; // Read the received char from the UART RX buffer
}

/* unused
static void uart1_flushTX(void) {
	while (!(UCSR1A & (_BV(TXC1) | _BV(UDRE1)))); //wait for TX complete
}
*/

/// @brief Erase a page using the old bootloader code.
/// @param page Address in bytes of the page to be erased. Must be aligned to the page boundary.
__attribute__((noinline)) static void oldBoot_erase_page(uint32_t page) {
	aprintf_P(PSTR("oldBoot_erase_page(%08lX)..."), page);
	asm volatile (
		// Save the registers which are modified by the old bootloader code
		"push r31" "\n\t"
		"push r30" "\n\t"
		"push r29" "\n\t"
		"push r28" "\n\t"
		"push r25" "\n\t"
		"push r24" "\n\t"
		"push r15" "\n\t"
		"push r14" "\n\t"
		"push r13" "\n\t"
		"push r12" "\n\t"
		"push r0" "\n\t"

		// Set PageStartAddress in the appropriate registers
		"mov r12, %A0" "\n\t"
		"mov r13, %B0" "\n\t"
		"mov r14, %C0" "\n\t"
		"mov r15, %D0" "\n\t"

		// Call convenient entry point in the old bootloader which contains the page erase code.
		// The tracer will catch the end of the page erase and jump to the ret instruction.
		"call 0x74fc" "\n\t"

		// Restore the registers saved earlier
		"pop r0" "\n\t"
		"pop r12" "\n\t"
		"pop r13" "\n\t"
		"pop r14" "\n\t"
		"pop r15" "\n\t"
		"pop r24" "\n\t"
		"pop r25" "\n\t"
		"pop r28" "\n\t"
		"pop r29" "\n\t"
		"pop r30" "\n\t"
		"pop r31" "\n\t"
		:
		: "r" (page)
	);
	aputs_P(PSTR("OK"));
}

/// @brief Fill and commit a page using the old bootloader code.
/// @param page Address in bytes of the page to be programmed. Must be aligned to the page boundary.
/// @param data PROGMEM reference to the array containing the bootloader code to be programmed. Must point to the beginning of the bootloader.
__attribute__((noinline)) static void oldBoot_fill_write_page(uint32_t page, const uint8_t *data) {
	aprintf_P(PSTR("oldBoot_fill_write_page(%08lX)..."), page);

	pageData = data + page - BOOT_START_ADDR;
	CurrAddress = page;

	asm volatile (
		// Save the registers which are modified by the old bootloader code
		"push r31" "\n\t"
		"push r30" "\n\t"
		"push r29" "\n\t"
		"push r28" "\n\t"
		"push r25" "\n\t"
		"push r24" "\n\t"
		"push r23" "\n\t"
		"push r22" "\n\t"
		"push r21" "\n\t"
		"push r20" "\n\t"
		"push r19" "\n\t"
		"push r18" "\n\t"
		"push r17" "\n\t"
		"push r16" "\n\t"
		"push r15" "\n\t"
		"push r14" "\n\t"
		"push r13" "\n\t"
		"push r12" "\n\t"
		"push r11" "\n\t"
		"push r10" "\n\t"
		"push r7" "\n\t"
		"push r6" "\n\t"
		"push r5" "\n\t"
		"push r4" "\n\t"
		"push r0" "\n\t"

		// Set PageStartAddress in the appropriate registers
		"mov r12, %A0" "\n\t"
		"mov r13, %B0" "\n\t"
		"mov r14, %C0" "\n\t"
		"mov r15, %D0" "\n\t"

		// Set MemoryType to 'F' so that the code old bootloader targets the flash instead of eeprom
		"ldi r17, 'F'" "\n\t"

		// Set BlockSize to 0x0080 (one full page worth of data)
		"ldi r28, 0x80" "\n\t"
		"ldi r29, 0x00" "\n\t"

		// Set LowByte to 0 to maintain consistency with old bootloader code:
		// This is normally set in the old bootloader code at 0x7526, but due to the tracer
		// catching the execution at this address for the page erase, we have to set it here
		// manually and then call the page write code one instruction later.
		"mov r11, r1" "\n\t"

		// Call convenient entry point in the old bootloader which contains the page fill and write code.
		// The tracer will catch the end of this operation and jump to the ret instruction.
		"call 0x7528" "\n\t"

		// Restore the registers saved earlier
		"pop r0" "\n\t"
		"pop r4" "\n\t"
		"pop r5" "\n\t"
		"pop r6" "\n\t"
		"pop r7" "\n\t"
		"pop r10" "\n\t"
		"pop r11" "\n\t"
		"pop r12" "\n\t"
		"pop r13" "\n\t"
		"pop r14" "\n\t"
		"pop r15" "\n\t"
		"pop r16" "\n\t"
		"pop r17" "\n\t"
		"pop r18" "\n\t"
		"pop r19" "\n\t"
		"pop r20" "\n\t"
		"pop r21" "\n\t"
		"pop r22" "\n\t"
		"pop r23" "\n\t"
		"pop r24" "\n\t"
		"pop r25" "\n\t"
		"pop r28" "\n\t"
		"pop r29" "\n\t"
		"pop r30" "\n\t"
		"pop r31" "\n\t"
		:
		: "r" (page)
	);

	aputs_P(PSTR("OK"));
}

/// @brief Exit method for the bootloader updater. It is called after all operations have completed.
///        It also erases the first page from the App section before resetting the board.
__attribute__((section(".exit_page_func"),noinline)) static void resetToBootloader(void) {
	puts_P(PSTR("resetToBootloader()"));

	// Disable all interrupts
	cli();

	// Erase the first page. It makes the bootloader think there is no program loaded in flash, so it doesn't time out and try to execute it.
	BootloaderAPI_ErasePageWrite(0);

	// Tell the boootloader that the watchdog reset was intentional and that it must not start executing the bootloader code.
	*(uint16_t *)(RAMEND - 1) = 0x7777;

	// Enable the watchdog timer with a short timeout.
	wdt_enable(WDTO_15MS);

	// Wait for the watchdog to timeout and reset the CPU.
	for (;;);
}


/// @brief Compare two PROGMEM regions. Identical behavior to memcmp.
static int memcmp_PP(const void *str1, const void *str2, size_t count) {
	register const unsigned char *s1 = (const unsigned char*)str1;
	register const unsigned char *s2 = (const unsigned char*)str2;
	
	while (count-- > 0) {
		if (pgm_read_byte(s1++) != pgm_read_byte(s2++))
			return s1[-1] < s2[-1] ? -1 : 1;
	}

	return 0;
}

/// @brief Update the bootloader using the MiniBootloader
static void newBoot_update(void) {
	// Loop through every page of the new bootloader and commit it to flash (except the MiniBootloader page which is already committed)
	for (uint32_t addr = 0; addr < sizeof(newBootloader) - SPM_PAGESIZE; addr += SPM_PAGESIZE) {
		aprintf_P(PSTR("newBoot(%08lX)..."), BOOT_START_ADDR + addr);

		pageData = newBootloader + addr;
		for (uint8_t i = 0; i < SPM_PAGESIZE; i += 2) {
			uint16_t wd = pgm_read_byte(pageData++);
			wd |= (uint16_t)(pgm_read_byte(pageData++)) << 8;
			// aprintf_P(PSTR("BootloaderAPI_FillWord(%08lX, %04X)\n"), BOOT_START_ADDR + addr + i, wd);
			BootloaderAPI_FillWord(BOOT_START_ADDR + addr + i, wd);
		}
		BootloaderAPI_ErasePageWrite(BOOT_START_ADDR + addr);

		aputs_P(PSTR("OK"));
	}
}

/// @brief Print the version information from the currently loaded MiniBootloader section
static void printCurrentVersion(void) {
	printf_P(PSTR("Current bootloader version: boot:%hu.%hu, hw:%hu.%hu\n"),
		pgm_read_byte(&bootloader_version_major), pgm_read_byte(&bootloader_version_minor),
		pgm_read_byte(&bootloader_hwversion_major), pgm_read_byte(&bootloader_hwversion_minor));
}

/// @brief Replace the MiniBootloader using the old MiniBootloader code
static void MiniBootloaderUpdate(void) {
	// Use the last-1 page to update the last page.
	printf_P(PSTR("MiniBootloaderUpdate()..."));
	uint16_t targetPage = FLASHEND - SPM_PAGESIZE * 2 + 1; // compute the address of the temporary MiniBootloader page
	pageData = newBootloader + sizeof(newBootloader) - SPM_PAGESIZE;
	
	// Commit the temporary MiniBootloader page to flash
	for (uint8_t i = 0; i < SPM_PAGESIZE; i += 2) {
		uint16_t wd = pgm_read_byte(pageData++);
		wd |= (uint16_t)(pgm_read_byte(pageData++)) << 8;
		BootloaderAPI_FillWord(targetPage + i, wd);
	}
	BootloaderAPI_ErasePageWrite(targetPage);

	// Function pointers to the API of the temporary page
	void (*_eraseWrite)(uint16_t addr) = &BootloaderAPI_ErasePageWrite - SPM_PAGESIZE;
	void (*_fill)(uint16_t addr, uint16_t data) = &BootloaderAPI_FillWord - SPM_PAGESIZE;
	
	// Now commit the MiniBootloader page using the code in the temporary page.
	targetPage = FLASHEND - SPM_PAGESIZE + 1;
	pageData = newBootloader + sizeof(newBootloader) - SPM_PAGESIZE;
	for (uint8_t i = 0; i < SPM_PAGESIZE; i += 2) {
		uint16_t wd = pgm_read_byte(pageData++);
		wd |= (uint16_t)(pgm_read_byte(pageData++)) << 8;
		(*_fill)(targetPage + i, wd);
	}
	(*_eraseWrite)(targetPage);
	
	aputs_P(PSTR("OK"));	
}

int main(void) {
	cli();
	
	// TIMER0 reset to default state to handle if the bootloader was interrupted while running
	TIMSK0 = 0; // disable all timer interrupts
	TCCR0A = 0; // reset timer control register A 
	TCCR0B = 0; // reset timer control register B 
	
	// Init UART1 for logging purposes and command entry
	UCSR1A |= _BV(U2X1); // baudrate multiplier
	UBRR1 = UART_BAUD_SELECT(115200, F_CPU); // select baudrate
	UCSR1B = _BV(RXEN1) | _BV(TXEN1); // enable receiver and transmitter
	fdev_setup_stream(&_uart1io, uart1_putchar, NULL, _FDEV_SETUP_WRITE); // setup uart1 i/o stream
	stdout = &_uart1io;
	puts_P(PSTR("\nstart"));
	
	// Check if the currently loaded bootloader is the old bootloader
	if (memcmp_PP((uint8_t*)BOOT_START_ADDR, oldBootloader, 4096) == 0) {
		// The current bootloader is the old bootloader. Load the MiniBootloader using the tracer

		GPIOR0 = 0; //Set register to DEVICE_STATE_Unattached so old bootloader LUFA code doesn't actually wait for data on USB
		
		// Setup the tracer:
		// The tracer is triggered by the INT0 interrupt. It interrupts the execution of the old bootloader code after every instruction
		// and allows the updater to manipulate the registers and memory of the old bootloader to trigger unintended operations.
		PORTD &= ~_BV(PIND0); // Set pin PD0 state LOW
		DDRD |= _BV(PIND0); // Set pin PD0 as output
		EICRA &= ~(_BV(ISC01) | _BV(ISC00)); // configure the INT0 interrupt in the "low level interrupt" mode (always triggers while the PD0 pin is LOW)
		EIMSK |= _BV(INT0); // Enable the INT0 interrupt
		
		// The following sequence is equivalent to MiniBootloaderUpdate(), but for the old bootloader
		sei(); // Start the tracer by enabling all interrupts
		oldBoot_erase_page(FLASHEND - SPM_PAGESIZE + 1); // Erase the last page. This page will be used to store the MiniBootloader
		oldBoot_fill_write_page(FLASHEND - SPM_PAGESIZE + 1, newBootloader); // Write the MiniBootloader
		cli(); // Stop the tracer by disabling all interrupts
		
		// Now flash the rest of the new bootloader using the new MiniBootloader
		newBoot_update();
	} else { // If it's not the old bootloader that's loaded, check if the current bootloader is valid and can be updated
		if (pgm_read_dword(&boot_start_addr) != BOOT_START_ADDR || pgm_read_word(&cdc_signature) != CDC_SIGNATURE || pgm_read_word(&lufa_signature) != LUFA_SIGNATURE) {
			puts_P(PSTR("Current bootloader not valid"));
			for (;;);
		}
		printCurrentVersion();
		MiniBootloaderUpdate();
		newBoot_update();
	}
	
	printCurrentVersion();
	puts_P(PSTR("done"));
	resetToBootloader();
}
