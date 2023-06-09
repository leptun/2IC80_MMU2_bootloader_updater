/*
             LUFA Library
     Copyright (C) Dean Camera, 2011.

  dean [at] fourwalledcubicle [dot] com
           www.lufa-lib.org
*/

/*
  Copyright 2011  Dean Camera (dean [at] fourwalledcubicle [dot] com)

  Permission to use, copy, modify, distribute, and sell this
  software and its documentation for any purpose is hereby granted
  without fee, provided that the above copyright notice appear in
  all copies and that both that the copyright notice and this
  permission notice and warranty disclaimer appear in supporting
  documentation, and that the name of the author not be used in
  advertising or publicity pertaining to distribution of the
  software without specific, written prior permission.

  The author disclaim all warranties with regard to this
  software, including all implied warranties of merchantability
  and fitness.  In no event shall the author be liable for any
  special, indirect or consequential damages or any damages
  whatsoever resulting from loss of use, data or profits, whether
  in an action of contract, negligence or other tortious action,
  arising out of or in connection with the use or performance of
  this software.
*/

/** \file
 *
 *  Main source file for the CDC class bootloader. This file contains the complete bootloader logic.
 */

#define  INCLUDE_FROM_CATERINA_C
#include "Caterina.h"
#include "BootloaderAPI.h"
#include <util/delay.h>

//UART1 bootloader support
#ifdef UART1_BOOT
#define UART_BAUD_SELECT(baudRate,xtalCpu) (((float)(xtalCpu))/(((float)(baudRate))*8.0)-1.0+0.5)
volatile uint8_t uart1_buff[256];//input round buffer
volatile uint8_t uart1_wpos = 0; //buffer write index
volatile uint8_t uart1_rpos = 0; //buffer read index
volatile uint8_t uart1_size = 0; //number of chars in buffer
#endif //UART1_BOOT

/** Contains the current baud rate and other settings of the first virtual serial port. This must be retained as some
 *  operating systems will not open the port unless the settings can be set successfully.
 */
static CDC_LineEncoding_t LineEncoding = { .BaudRateBPS = 0,
                                           .CharFormat  = CDC_LINEENCODING_OneStopBit,
                                           .ParityType  = CDC_PARITY_None,
                                           .DataBits    = 8                            };

/** Current address counter. This stores the current address of the FLASH or EEPROM as set by the host,
 *  and is used when reading or writing to the AVRs memory (either FLASH or EEPROM depending on the issued
 *  command.)
 */
static uint16_t CurrAddress;

#if (FLASHEND > 0xFFFF)
#error Max flash size is 64KB
#endif

#define BOOT_KEY_MAGIC 0x7777U
#define BOOT_KEY (*(volatile uint16_t *)(RAMEND - 1)) // new location

enum {
	InterfaceUnknown = 0,
	InterfaceUSB,
#ifdef UART1_BOOT
	InterfaceUART1,
#endif
} volatile commInterface = InterfaceUnknown;

bool programmingMode = false;


/* Bootloader timeout timer */
uint16_t Timeout = 0;

#ifdef SHR16_DS_PIN
inline void shr16_init(void) {
	WRITE(SHR16_DS_PIN, 0);
	WRITE(SHR16_STCP_PIN, 0);
	WRITE(SHR16_SHCP_PIN, 0);
	SET_OUTPUT(SHR16_DS_PIN);
	SET_OUTPUT(SHR16_STCP_PIN);
	SET_OUTPUT(SHR16_SHCP_PIN);
}

void shr16_write(uint16_t v) {
	for (uint8_t k = 0; k < 16; k++) {
		WRITE(SHR16_SHCP_PIN, 0);
		WRITE(SHR16_DS_PIN, (v & 0x01));
		// _delay_us(1);
		WRITE(SHR16_SHCP_PIN, 1);
		// _delay_us(1);
		v >>= 1;
	}
	WRITE(SHR16_SHCP_PIN, 0);
	WRITE(SHR16_STCP_PIN, 1);
	_delay_us(15);
	WRITE(SHR16_STCP_PIN, 0);
	_delay_us(15);
}
#endif

/*	Breathing animation on L LED indicates bootloader is running */
uint16_t LLEDPulse;
void LEDPulse(void)
{
#ifdef SHR16_DS_PIN
	LLEDPulse += 3;
#else
	LLEDPulse += 1;
#endif
	uint8_t p = LLEDPulse >> 8;
	if (p > 127)
		p = 255-p;
	p += p;
	if (((uint8_t)LLEDPulse) > p) {
#ifdef L_LED_PIN
		WRITE(L_LED_PIN, !L_LED_POL);
#endif
#ifdef SHR16_DS_PIN
		shr16_write(0x5400);
#endif
	} else {
#ifdef L_LED_PIN
		WRITE(L_LED_PIN, L_LED_POL);
#endif
#ifdef SHR16_DS_PIN
		shr16_write(0x56AA);
#endif
	}
}

void StartSketch(void)
{
	cli();

	/* Relocate the interrupt vector table to the application section */
	MCUCR = (1 << IVCE);
	MCUCR = 0;

	/* jump to beginning of application space */
	__asm__ volatile("jmp 0x0000");
	
	// No return
	for (;;);
}

/** Main program entry point. This routine configures the hardware required by the bootloader, then continuously
 *  runs the bootloader processing routine until it times out or is instructed to exit.
 */
int main(void)
{
	/* Check the reason for the reset so we can act accordingly */
	uint8_t mcusr_backup = MCUSR;		// store the initial state of the Status register
#ifdef PRESERVE_MCUSR_FLAGS
	MCUSR &= ~(1 << PORF);				// clear PORF flag only, other flags we want keep for application
#else
	MCUSR = 0;							// clear all reset flags
#endif

	uint16_t boot_key_backup = BOOT_KEY;
	BOOT_KEY = mcusr_backup;

	/* Watchdog may be configured with a 15 ms period so must disable it before going any further */
	wdt_disable();

	uint16_t appWord = pgm_read_word(0);

	if (mcusr_backup & (1<<EXTRF)) {
		// External reset -  we should continue to self-programming mode.
	} else if ((mcusr_backup & (1<<PORF)) && (appWord != 0xFFFF)) {
		// After a power-on reset skip the bootloader and jump straight to sketch
		// if one exists.
#ifdef PRESERVE_MCUSR_FLAGS
		//clear brown out, watchdog and external reset flag, because we want ignore it in case of power on
		MCUSR &= ~((1 << BORF) | (1 << WDRF) | (1 << EXTRF)); 
#endif
		StartSketch();
	}
	else if((mcusr_backup & (1<<WDRF)) && (boot_key_backup != BOOT_KEY_MAGIC) && (appWord != 0xFFFF)) {
		// If it looks like an "accidental" watchdog reset then start the sketch.
		StartSketch();
	}
	
	/* Setup hardware required for the bootloader */
	SetupHardware();

	/* Enable global interrupts so that the USB stack can function */
	sei();
	
	Timeout = 0;
	
	for (;;)
	{
		CDC_Task();
		USB_USBTask();
		/* Time out and start the sketch if one is present */
		if (Timeout > TIMEOUT_PERIOD)
			break;

		LEDPulse();
		
	}

	/* Disconnect from the host - USB interface will be reset later along with the AVR */
	USB_Detach();

	/* Watchdog reset to start the application. */
	wdt_enable(WDTO_120MS);
	for(;;);
}

/** Configures all hardware required for the bootloader. */
void SetupHardware(void)
{
#ifdef PRESERVE_MCUSR_FLAGS
	/* Disable watchdog if enabled by bootloader/fuses */
	MCUSR &= ~(1 << WDRF);
#endif
	wdt_disable();

	/* Disable clock division */
	clock_prescale_set(clock_div_1);

	/* Relocate the interrupt vector table to the bootloader section */
	MCUCR = (1 << IVCE);
	MCUCR = (1 << IVSEL);

//TCM2130 CS signals to output/high
#ifdef TMC1_CS_PIN
	WRITE(TMC1_CS_PIN, 1);
	SET_OUTPUT(TMC1_CS_PIN);
#endif
#ifdef TMC2_CS_PIN
	WRITE(TMC2_CS_PIN, 1);
	SET_OUTPUT(TMC2_CS_PIN);
#endif
#ifdef TMC3_CS_PIN
	WRITE(TMC3_CS_PIN, 1);
	SET_OUTPUT(TMC3_CS_PIN);
#endif

#ifdef RD_LED_PIN
	WRITE(RD_LED_PIN, !RD_LED_POL);
	SET_OUTPUT(RD_LED_PIN);
#endif
#ifdef WR_LED_PIN
	WRITE(WR_LED_PIN, !WR_LED_POL);
	SET_OUTPUT(WR_LED_PIN);
#endif
#ifdef L_LED_PIN
	WRITE(L_LED_PIN, !L_LED_POL);
	SET_OUTPUT(L_LED_PIN);
#endif

#ifdef SHR16_DS_PIN
	shr16_init();
	shr16_write(0x5400); //disable steppers and LEDs
#endif

	/* Initialize TIMER1 to handle bootloader timeout and LED tasks.  
	 * With 16 MHz clock and 1/64 prescaler, timer 1 is clocked at 250 kHz
	 * Our chosen compare match generates an interrupt every 1 ms.
	 */ 
	OCR1A = F_CPU/ 64000;
	TIMSK1 = (1 << OCIE1A);									// enable timer 1 output compare A match interrupt
	TCCR1B = ((1 << WGM12) | (1 << CS11) | (1 << CS10));	// 1/64 prescaler on timer 1 input, CTC mode

	/* Initialize USB Subsystem */
	USB_Init();

#ifdef UART1_BOOT
	//init UART1
	UCSR1A |= (1 << U2X1); // baudrate multiplier
	UBRR1 = UART_BAUD_SELECT(UART1_BAUD, F_CPU); // select baudrate
	UCSR1B = (1 << RXEN1) | (1 << TXEN1); // enable receiver and transmitter
	UCSR1B |= (1 << RXCIE1); // enable rx interrupt
#endif //UART1_BOOT

}

#ifdef UART1_BOOT
ISR(USART1_RX_vect)
{
	if (commInterface == InterfaceUnknown)
		commInterface = InterfaceUART1;

	uart1_buff[uart1_wpos++] = UDR1;
	uart1_size++;
}
#endif //UART1_BOOT


ISR(TIMER1_COMPA_vect, ISR_BLOCK)
{
	if (pgm_read_word(0) != 0xFFFF)
		Timeout++;
}

/** Event handler for the USB_ConfigurationChanged event. This configures the device's endpoints ready
 *  to relay data to and from the attached USB host.
 */
void EVENT_USB_Device_ConfigurationChanged(void)
{
	/* Setup CDC Notification, Rx and Tx Endpoints */
	Endpoint_ConfigureEndpoint(CDC_NOTIFICATION_EPNUM, EP_TYPE_INTERRUPT,
	                           ENDPOINT_DIR_IN, CDC_NOTIFICATION_EPSIZE,
	                           ENDPOINT_BANK_SINGLE);

	Endpoint_ConfigureEndpoint(CDC_TX_EPNUM, EP_TYPE_BULK,
	                           ENDPOINT_DIR_IN, CDC_TXRX_EPSIZE,
	                           ENDPOINT_BANK_SINGLE);

	Endpoint_ConfigureEndpoint(CDC_RX_EPNUM, EP_TYPE_BULK,
	                           ENDPOINT_DIR_OUT, CDC_TXRX_EPSIZE,
	                           ENDPOINT_BANK_SINGLE);
}

/** Event handler for the USB_ControlRequest event. This is used to catch and process control requests sent to
 *  the device from the USB host before passing along unhandled control requests to the library for processing
 *  internally.
 */
void EVENT_USB_Device_ControlRequest(void)
{
	/* Ignore any requests that aren't directed to the CDC interface */
	if ((USB_ControlRequest.bmRequestType & (CONTROL_REQTYPE_TYPE | CONTROL_REQTYPE_RECIPIENT)) !=
		(REQTYPE_CLASS | REQREC_INTERFACE))
	{
		return;
	}

	/* Process CDC specific control requests */
	switch (USB_ControlRequest.bRequest)
	{
		case CDC_REQ_GetLineEncoding:
			if (USB_ControlRequest.bmRequestType == (REQDIR_DEVICETOHOST | REQTYPE_CLASS | REQREC_INTERFACE))
			{
				Endpoint_ClearSETUP();

				/* Write the line coding data to the control endpoint */
				Endpoint_Write_Control_Stream_LE(&LineEncoding, sizeof(CDC_LineEncoding_t));
				Endpoint_ClearOUT();
			}

			break;
		case CDC_REQ_SetLineEncoding:
			if (USB_ControlRequest.bmRequestType == (REQDIR_HOSTTODEVICE | REQTYPE_CLASS | REQREC_INTERFACE))
			{
				Endpoint_ClearSETUP();

				/* Read the line coding data in from the host into the global struct */
				Endpoint_Read_Control_Stream_LE(&LineEncoding, sizeof(CDC_LineEncoding_t));
				Endpoint_ClearIN();
			}

			break;
	}
}

void runtimeError(void) {
	/* Send error byte back to the host */
	WriteNextResponseByte('?');
	
#ifdef UART1_BOOT
	if (commInterface == InterfaceUART1) {
		UCSR1B = 0; //disable uart1
		commInterface = InterfaceUnknown;
	}
#endif
}

#if !defined(NO_BLOCK_SUPPORT)
/** Reads or writes a block of EEPROM or FLASH memory to or from the appropriate CDC data endpoint, depending
 *  on the AVR910 protocol command issued.
 *
 *  \param[in] Command  Single character AVR910 protocol command indicating what memory operation to perform
 */
static void ReadWriteMemoryBlock(const uint8_t Command) {
	uint16_t BlockSize;
	char MemoryType;
	uint16_t address = CurrAddress;

	BlockSize = (FetchNextCommandByte() << 8);
	BlockSize |= FetchNextCommandByte();

	MemoryType = FetchNextCommandByte();
	
	if (MemoryType == 'F') {
		if (Command == 'B') {
			if ((address & (SPM_PAGESIZE - 1)) || (address + BlockSize > BOOT_START_ADDR)) {
				runtimeError();
				return;
			}
#ifdef WR_LED_PIN
			WRITE(WR_LED_PIN, WR_LED_POL);
#endif
			BlockSize >>= 1; //we work with words here, not bytes
			while (BlockSize--) {
				uint16_t wd = FetchNextCommandByte();
				wd |= FetchNextCommandByte() << 8;
				BootloaderAPI_FillWord(address, wd);
				address += 2;
			}
			if (commInterface != InterfaceUnknown) { // data is valid. It is safe to commit
				BootloaderAPI_ErasePageWrite(CurrAddress);
			}
			WriteNextResponseByte('\r');
#ifdef WR_LED_PIN
			WRITE(WR_LED_PIN, !WR_LED_POL);
#endif
		}
		else if (Command == 'g') {
#ifdef RD_LED_PIN
			WRITE(RD_LED_PIN, RD_LED_POL);
#endif
			while (BlockSize--) {
				WriteNextResponseByte(pgm_read_byte(address));
				address++;
			}
#ifdef RD_LED_PIN
			WRITE(RD_LED_PIN, !RD_LED_POL);
#endif
		}
		else {
			runtimeError();
			return;
		}
	}
	else if (MemoryType == 'E') {
		if (Command == 'B') {
#ifdef WR_LED_PIN
			WRITE(WR_LED_PIN, WR_LED_POL);
#endif
			while (BlockSize--) {
				eeprom_write_byte((uint8_t*)(address >> 1), FetchNextCommandByte());
				address += 2;
			}
			WriteNextResponseByte('\r');
#ifdef WR_LED_PIN
			WRITE(WR_LED_PIN, !WR_LED_POL);
#endif
		}
		else if (Command == 'g') {
#ifdef RD_LED_PIN
			WRITE(RD_LED_PIN, RD_LED_POL);
#endif
			while (BlockSize--) {
				WriteNextResponseByte(eeprom_read_byte((uint8_t*)(address >> 1)));
				address += 2;
			}
#ifdef RD_LED_PIN
			WRITE(RD_LED_PIN, !RD_LED_POL);
#endif
		}
		else {
			runtimeError();
			return;
		}
	}
	else {
		runtimeError();
		return;
	}
	CurrAddress = address;
}
#endif

/** Retrieves the next byte from the host in the CDC data OUT endpoint, and clears the endpoint bank if needed
 *  to allow reception of the next data packet from the host.
 *
 *  \return Next received byte from the host in the CDC data OUT endpoint
 */
static uint8_t FetchNextCommandByte(void)
{
	switch (commInterface) {
#ifdef UART1_BOOT
	case InterfaceUART1:
		while (uart1_size == 0) {};
		uart1_size--;
		return uart1_buff[uart1_rpos++];
#endif //UART1_BOOT
	case InterfaceUSB:
		/* Select the OUT endpoint so that the next data byte can be read */
		Endpoint_SelectEndpoint(CDC_RX_EPNUM);
		/* If OUT endpoint empty, clear it and wait for the next packet from the host */
		while (!(Endpoint_IsReadWriteAllowed())) {
			Endpoint_ClearOUT();
			while (!(Endpoint_IsOUTReceived())) {
				if (USB_DeviceState == DEVICE_STATE_Unattached)
					return 0;
			}
		}
		/* Fetch the next byte from the OUT endpoint */
		return Endpoint_Read_8();
	case InterfaceUnknown:
	default:
		return 0;
	}
}

/** Writes the next response byte to the CDC data IN endpoint, and sends the endpoint back if needed to free up the
 *  bank when full ready for the next byte in the packet to the host.
 *
 *  \param[in] Response  Next response byte to send to the host
 */
static void WriteNextResponseByte(const uint8_t Response)
{
	LEDPulse();
#ifdef UART1_BOOT
	if (commInterface == InterfaceUART1) {
		while (!(UCSR1A & (1 << UDRE1))) {};   // wait until buffer empty
		UDR1 = Response;                       // send byte
		while (!(UCSR1A & (1 << TXC1))) {};   // wait until TX is complete
	}
#endif
	if (commInterface == InterfaceUSB) {
		/* Select the IN endpoint so that the next data byte can be written */
		Endpoint_SelectEndpoint(CDC_TX_EPNUM);

		/* If IN endpoint full, clear it and wait until ready for the next packet to the host */
		if (!(Endpoint_IsReadWriteAllowed())) {
			Endpoint_ClearIN();
			while (!(Endpoint_IsINReady())) {
				if (USB_DeviceState == DEVICE_STATE_Unattached) {
					return;
				}
			}
		}

		/* Write the next byte to the IN endpoint */
		Endpoint_Write_8(Response);
	}
}

#define STK_OK              0x10
#define STK_INSYNC          0x14  // ' '
#define CRC_EOP             0x20  // 'SPACE'
#define STK_GET_SYNC        0x30  // '0'

#define STK_GET_PARAMETER   0x41  // 'A'
#define STK_SET_DEVICE      0x42  // 'B'
#define STK_SET_DEVICE_EXT  0x45  // 'E'
#define STK_LOAD_ADDRESS    0x55  // 'U'
#define STK_UNIVERSAL       0x56  // 'V'
#define STK_PROG_PAGE       0x64  // 'd'
#define STK_READ_PAGE       0x74  // 't'
#define STK_READ_SIGN       0x75  // 'u'

/** Task to read in AVR910 commands from the CDC data OUT endpoint, process them, perform the required actions
 *  and send the appropriate response back to the host.
 */
void CDC_Task(void)
{
	if ((USB_DeviceState == DEVICE_STATE_Configured) && (commInterface == InterfaceUnknown)) {
		commInterface = InterfaceUSB;
	}
	
	switch (commInterface) {
#ifdef UART1_BOOT
	case InterfaceUART1:
		if (uart1_size == 0) {
			return;
		}
		break;
#endif
	case InterfaceUSB:
		/* Select the OUT endpoint */
		Endpoint_SelectEndpoint(CDC_RX_EPNUM);
		/* Check if endpoint has a command in it sent from the host */
		if (!(Endpoint_IsOUTReceived()))
			return;
		break;
	case InterfaceUnknown:
	default:
		return;
	}

	/* Read in the bootloader command (first byte sent from host) */
	uint8_t Command = FetchNextCommandByte();

	switch (Command) {
	case 'E':
		/* We nearly run out the bootloader timeout clock, 
		* leaving just a few hundred milliseconds so the 
		* bootloder has time to respond and service any 
		* subsequent requests */
		Timeout = TIMEOUT_PERIOD - 500;
	
		//@leptun - Should already be enabled immediately after a page erase/write
		// /* Re-enable RWW section - must be done here in case
		// * user has disabled verification on upload.  */
		// boot_rww_enable_safe();

		// Send confirmation byte back to the host 
		WriteNextResponseByte('\r');
		break;
	case 'T':
		FetchNextCommandByte();

		// Send confirmation byte back to the host 
		WriteNextResponseByte('\r');
		break;
	case 'P':
		programmingMode = true;
		// Send confirmation byte back to the host 
		WriteNextResponseByte('\r');
		break;
	case 'L':
		programmingMode = false;
		// Send confirmation byte back to the host 
		WriteNextResponseByte('\r');
		break;
	case 't':
		// Return ATMEGA128 part code - this is only to allow AVRProg to use the bootloader 
		WriteNextResponseByte(0x44);
		WriteNextResponseByte(0x00);
		break;
	case 'a':
		// Indicate auto-address increment is supported 
		WriteNextResponseByte('Y');
		break;
	case 'A':
		// Set the current address to that given by the host
		if (!programmingMode) {
			runtimeError();
			break;
		}
		
		CurrAddress   = (FetchNextCommandByte() << 9);
		CurrAddress  |= (FetchNextCommandByte() << 1);

		// Send confirmation byte back to the host 
		WriteNextResponseByte('\r');
		break;
	case 'p':
		// Indicate serial programmer back to the host 
		WriteNextResponseByte('S');
		break;
	case 'S':
		// Write the 7-byte software identifier to the endpoint 
		for (uint8_t CurrByte = 0; CurrByte < 7; CurrByte++)
			WriteNextResponseByte(SOFTWARE_IDENTIFIER[CurrByte]);
		break;
	case 'V':
		WriteNextResponseByte('0' + BOOTLOADER_VERSION_MAJOR);
		WriteNextResponseByte('0' + BOOTLOADER_VERSION_MINOR);
		break;
	case 'v':
		WriteNextResponseByte('0' + BOOTLOADER_HWVERSION_MAJOR);
		WriteNextResponseByte('0' + BOOTLOADER_HWVERSION_MINOR);
		break;
	case 's':
		WriteNextResponseByte(AVR_SIGNATURE_3);
		WriteNextResponseByte(AVR_SIGNATURE_2);
		WriteNextResponseByte(AVR_SIGNATURE_1);
		break;
	case 'e':
		// Clear the application section of flash 
		if (!programmingMode) {
			runtimeError();
			break;
		}
#ifdef WR_LED_PIN
		WRITE(WR_LED_PIN, WR_LED_POL);
#endif
		for (uint16_t CurrFlashAddress = 0; CurrFlashAddress < BOOT_START_ADDR; CurrFlashAddress += SPM_PAGESIZE) {
			BootloaderAPI_ErasePageWrite(CurrFlashAddress);
		}

		// Send confirmation byte back to the host 
		WriteNextResponseByte('\r');
#ifdef WR_LED_PIN
		WRITE(WR_LED_PIN, !WR_LED_POL);
#endif
		break;
#if !defined(NO_LOCK_BYTE_WRITE_SUPPORT)
	case 'l':
		if (!programmingMode) {
			runtimeError();
			break;
		}
		// Set the lock bits to those given by the host 
		boot_lock_bits_set(FetchNextCommandByte());

		// Send confirmation byte back to the host 
		WriteNextResponseByte('\r');
		break;
#endif
	case 'r':
		WriteNextResponseByte(boot_lock_fuse_bits_get(GET_LOCK_BITS));
		break;
	case 'F':
		WriteNextResponseByte(boot_lock_fuse_bits_get(GET_LOW_FUSE_BITS));
		break;
	case 'N':
		WriteNextResponseByte(boot_lock_fuse_bits_get(GET_HIGH_FUSE_BITS));
		break;
	case 'Q':
		WriteNextResponseByte(boot_lock_fuse_bits_get(GET_EXTENDED_FUSE_BITS));
		break;
#if !defined(NO_BLOCK_SUPPORT)
	case 'b':
		WriteNextResponseByte('Y');

		// Send block size to the host 
		WriteNextResponseByte(SPM_PAGESIZE >> 8);
		WriteNextResponseByte(SPM_PAGESIZE & 0xFF);
		break;
	case 'B':
		if (!programmingMode) {
			runtimeError();
			break;
		}
		// fall through
	case 'g':
		// Keep resetting the timeout counter if we're receiving self-programming instructions
		Timeout = 0;
		// Delegate the block write/read to a separate function for clarity 
		ReadWriteMemoryBlock(Command);
		break;
#endif
#if !defined(NO_EEPROM_BYTE_SUPPORT)
	case 'D':
		if (!programmingMode) {
			runtimeError();
			break;
		}
		// Read the byte from the endpoint and write it to the EEPROM 
		eeprom_write_byte((uint8_t*)((intptr_t)(CurrAddress >> 1)), FetchNextCommandByte());

		// Increment the address after use
		CurrAddress += 2;

		// Send confirmation byte back to the host 
		WriteNextResponseByte('\r');
		break;
	case 'd':
		// Read the EEPROM byte and write it to the endpoint 
		WriteNextResponseByte(eeprom_read_byte((uint8_t*)((intptr_t)(CurrAddress >> 1))));

		// Increment the address after use 
		CurrAddress += 2;
		break;
#endif
	case 0x1b: // aka 27, aka escape character
		break;
	default:
		runtimeError();
	}


	if (commInterface != InterfaceUSB) {
		return;
	}

	/* Select the IN endpoint */
	Endpoint_SelectEndpoint(CDC_TX_EPNUM);

	/* Remember if the endpoint is completely full before clearing it */
	bool IsEndpointFull = !(Endpoint_IsReadWriteAllowed());

	/* Send the endpoint data to the host */
	Endpoint_ClearIN();

	/* If a full endpoint's worth of data was sent, we need to send an empty packet afterwards to signal end of transfer */
	if (IsEndpointFull) {
		while (!(Endpoint_IsINReady())) {
			if (USB_DeviceState == DEVICE_STATE_Unattached)
				return;
		}
		Endpoint_ClearIN();
	}

	/* Wait until the data has been sent to the host */
	while (!(Endpoint_IsINReady())) {
		if (USB_DeviceState == DEVICE_STATE_Unattached)
			return;
	}

	/* Select the OUT endpoint */
	Endpoint_SelectEndpoint(CDC_RX_EPNUM);

	/* Acknowledge the command from the host */
	Endpoint_ClearOUT();
}

