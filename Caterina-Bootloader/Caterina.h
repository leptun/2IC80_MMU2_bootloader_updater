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
 *  Header file for BootloaderCDC.c.
 */

#ifndef _CDC_H_
#define _CDC_H_

/* Includes: */
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/boot.h>
#include <avr/eeprom.h>
#include <avr/power.h>
#include <avr/interrupt.h>
#include <stdbool.h>

#include CONFIG
#include "Descriptors.h"
#include "fastio.h"

#include <LUFA/Drivers/USB/USB.h>

void StartSketch(void);
void LEDPulse(void);

void CDC_Task(void);
void SetupHardware(void);

void EVENT_USB_Device_ConfigurationChanged(void);

#if defined(INCLUDE_FROM_CATERINA_C) || defined(__DOXYGEN__)
	#if !defined(NO_BLOCK_SUPPORT)
	static void    ReadWriteMemoryBlock(const uint8_t Command);
	#endif
	static uint8_t FetchNextCommandByte(void);
	static void    WriteNextResponseByte(const uint8_t Response);
#endif

#endif

