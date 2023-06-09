/*
  This code contributed by Triffid_Hunter and modified by Kliment
  why double up on these macros? see http://gcc.gnu.org/onlinedocs/cpp/Stringification.html
*/

#pragma once

#include <avr/io.h>

#define CRITICAL_SECTION_START  unsigned char _sreg = SREG; cli();
#define CRITICAL_SECTION_END    SREG = _sreg;

/*
  magic I/O routines
  now you can simply SET_OUTPUT(STEP); WRITE(STEP, 1); WRITE(STEP, 0);
*/

/// Read a pin
#define _READ(IO) ((bool)(DIO ## IO ## _RPORT & _BV(DIO ## IO ## _PIN)))
/// write to a pin
// On some boards pins > 0x100 are used. These are not converted to atomic actions. An critical section is needed.

#define _WRITE_NC(IO, v)  do { if (v) {DIO ##  IO ## _WPORT |= _BV(DIO ## IO ## _PIN); } else {DIO ##  IO ## _WPORT &= ~_BV(DIO ## IO ## _PIN); }; } while (0)

#define _WRITE_C(IO, v)   do { if (v) { \
                                         CRITICAL_SECTION_START; \
                                         {DIO ##  IO ## _WPORT |= _BV(DIO ## IO ## _PIN); }\
                                         CRITICAL_SECTION_END; \
                                       }\
                                       else {\
                                         CRITICAL_SECTION_START; \
                                         {DIO ##  IO ## _WPORT &= ~_BV(DIO ## IO ## _PIN); }\
                                         CRITICAL_SECTION_END; \
                                       }\
                                     }\
                                     while (0)

#define _WRITE(IO, v)  do {  if (&(DIO ##  IO ## _RPORT) >= (uint8_t *)0x100) {_WRITE_C(IO, v); } else {_WRITE_NC(IO, v); }; } while (0)

/// toggle a pin
#define _TOGGLE(IO)  do {DIO ##  IO ## _RPORT = _BV(DIO ## IO ## _PIN); } while (0)

/// set pin as input
#define	_SET_INPUT(IO) do {DIO ##  IO ## _DDR &= ~_BV(DIO ## IO ## _PIN); } while (0)
/// set pin as output
#define	_SET_OUTPUT(IO) do {DIO ##  IO ## _DDR |=  _BV(DIO ## IO ## _PIN); } while (0)

/// check if pin is an input
#define	_GET_INPUT(IO)  ((DIO ## IO ## _DDR & _BV(DIO ## IO ## _PIN)) == 0)
/// check if pin is an output
#define	_GET_OUTPUT(IO)  ((DIO ## IO ## _DDR & _BV(DIO ## IO ## _PIN)) != 0)

/// check if pin is an timer
#define	_GET_TIMER(IO)  (DIO ## IO ## _PWM)

/// check if pin is an ADC
#define	_GET_ADC(IO)  (DIO ## IO ## _ADC)

//  why double up on these macros? see http://gcc.gnu.org/onlinedocs/cpp/Stringification.html

/// Read a pin wrapper
#define READ(IO)  _READ(IO)
/// Write to a pin wrapper
#define WRITE(IO, v)  _WRITE(IO, v)
/// Write to a pin wrapper, non critical.
/// This macro is cheaper than WRITE(IO,v) on ports H,I,J,K,L, as _WRITE_C disables / enables interrupts
/// and stores the old CPU flags on the stack.
/// This macro should only be called, where it cannot be interrupted. 
#define WRITE_NC(IO, v)  _WRITE_NC(IO, v)

/// toggle a pin wrapper
#define TOGGLE(IO)  _TOGGLE(IO)

/// set pin as input wrapper
#define SET_INPUT(IO)  _SET_INPUT(IO)
/// set pin as output wrapper
#define SET_OUTPUT(IO)  _SET_OUTPUT(IO)

/// check if pin is an input wrapper
#define GET_INPUT(IO)  _GET_INPUT(IO)
/// check if pin is an output wrapper
#define GET_OUTPUT(IO)  _GET_OUTPUT(IO)

/// check if pin is an timer wrapper
#define GET_TIMER(IO)  _GET_TIMER(IO)

/// check if pin is an ADC wrapper
#define GET_ADC(IO)  _GET_ADC(IO)

/*
	ports and functions

	added as necessary or if I feel like it- not a comprehensive list!
*/

#if defined (__AVR_ATmega1280__) || defined (__AVR_ATmega2560__)

#define	SCK_PIN					52
#define	MISO_PIN				50
#define	MOSI_PIN				51
#define SS_PIN					53

#define	DIO0_PIN		PINE0
#define	DIO0_RPORT	PINE
#define	DIO0_WPORT	PORTE
#define	DIO0_DDR		DDRE
#define DIO0_PWM		NULL

#define	DIO1_PIN		PINE1
#define	DIO1_RPORT	PINE
#define	DIO1_WPORT	PORTE
#define	DIO1_DDR		DDRE
#define DIO1_PWM		NULL

#define	DIO2_PIN		PINE4
#define	DIO2_RPORT	PINE
#define	DIO2_WPORT	PORTE
#define	DIO2_DDR		DDRE
#define DIO2_PWM		&OCR3BL

#define	DIO3_PIN		PINE5
#define	DIO3_RPORT	PINE
#define	DIO3_WPORT	PORTE
#define	DIO3_DDR		DDRE
#define DIO3_PWM		&OCR3CL

#define	DIO4_PIN		PING5
#define	DIO4_RPORT	PING
#define	DIO4_WPORT	PORTG
#define	DIO4_DDR		DDRG
#define DIO4_PWM		&OCR0B

#define	DIO5_PIN		PINE3
#define	DIO5_RPORT	PINE
#define	DIO5_WPORT	PORTE
#define	DIO5_DDR		DDRE
#define DIO5_PWM		&OCR3AL

#define	DIO6_PIN		PINH3
#define	DIO6_RPORT	PINH
#define	DIO6_WPORT	PORTH
#define	DIO6_DDR		DDRH
#define DIO6_PWM		&OCR4AL

#define	DIO7_PIN		PINH4
#define	DIO7_RPORT	PINH
#define	DIO7_WPORT	PORTH
#define	DIO7_DDR		DDRH
#define DIO7_PWM		&OCR4BL

#define	DIO8_PIN		PINH5
#define	DIO8_RPORT	PINH
#define	DIO8_WPORT	PORTH
#define	DIO8_DDR		DDRH
#define DIO8_PWM		&OCR4CL

#define	DIO9_PIN		PINH6
#define	DIO9_RPORT	PINH
#define	DIO9_WPORT	PORTH
#define	DIO9_DDR		DDRH
#define DIO9_PWM		&OCR2B

#define	DIO10_PIN		PINB4
#define	DIO10_RPORT	PINB
#define	DIO10_WPORT	PORTB
#define	DIO10_DDR		DDRB
#define DIO10_PWM		&OCR2A

#define	DIO11_PIN		PINB5
#define	DIO11_RPORT	PINB
#define	DIO11_WPORT	PORTB
#define	DIO11_DDR		DDRB
#define DIO11_PWM		NULL

#define	DIO12_PIN		PINB6
#define	DIO12_RPORT	PINB
#define	DIO12_WPORT	PORTB
#define	DIO12_DDR		DDRB
#define DIO12_PWM		NULL

#define	DIO13_PIN		PINB7
#define	DIO13_RPORT	PINB
#define	DIO13_WPORT	PORTB
#define	DIO13_DDR		DDRB
#define DIO13_PWM		&OCR0A

#define	DIO14_PIN		PINJ1
#define	DIO14_RPORT	PINJ
#define	DIO14_WPORT	PORTJ
#define	DIO14_DDR		DDRJ
#define DIO14_PWM		NULL

#define	DIO15_PIN		PINJ0
#define	DIO15_RPORT	PINJ
#define	DIO15_WPORT	PORTJ
#define	DIO15_DDR		DDRJ
#define DIO15_PWM		NULL

#define	DIO16_PIN		PINH1
#define	DIO16_RPORT	PINH
#define	DIO16_WPORT	PORTH
#define	DIO16_DDR		DDRH
#define DIO16_PWM		NULL

#define	DIO17_PIN		PINH0
#define	DIO17_RPORT	PINH
#define	DIO17_WPORT	PORTH
#define	DIO17_DDR		DDRH
#define DIO17_PWM		NULL

#define	DIO18_PIN		PIND3
#define	DIO18_RPORT	PIND
#define	DIO18_WPORT	PORTD
#define	DIO18_DDR		DDRD
#define DIO18_PWM		NULL

#define	DIO19_PIN		PIND2
#define	DIO19_RPORT	PIND
#define	DIO19_WPORT	PORTD
#define	DIO19_DDR		DDRD
#define DIO19_PWM		NULL

#define	DIO20_PIN		PIND1
#define	DIO20_RPORT	PIND
#define	DIO20_WPORT	PORTD
#define	DIO20_DDR		DDRD
#define DIO20_PWM		NULL

#define	DIO21_PIN		PIND0
#define	DIO21_RPORT	PIND
#define	DIO21_WPORT	PORTD
#define	DIO21_DDR		DDRD
#define DIO21_PWM		NULL

#define	DIO22_PIN		PINA0
#define	DIO22_RPORT	PINA
#define	DIO22_WPORT	PORTA
#define	DIO22_DDR		DDRA
#define DIO22_PWM		NULL

#define	DIO23_PIN		PINA1
#define	DIO23_RPORT	PINA
#define	DIO23_WPORT	PORTA
#define	DIO23_DDR		DDRA
#define DIO23_PWM		NULL

#define	DIO24_PIN		PINA2
#define	DIO24_RPORT	PINA
#define	DIO24_WPORT	PORTA
#define	DIO24_DDR		DDRA
#define DIO24_PWM		NULL

#define	DIO25_PIN		PINA3
#define	DIO25_RPORT	PINA
#define	DIO25_WPORT	PORTA
#define	DIO25_DDR		DDRA
#define DIO25_PWM		NULL

#define	DIO26_PIN		PINA4
#define	DIO26_RPORT	PINA
#define	DIO26_WPORT	PORTA
#define	DIO26_DDR		DDRA
#define DIO26_PWM		NULL

#define	DIO27_PIN		PINA5
#define	DIO27_RPORT	PINA
#define	DIO27_WPORT	PORTA
#define	DIO27_DDR		DDRA
#define DIO27_PWM		NULL

#define	DIO28_PIN		PINA6
#define	DIO28_RPORT	PINA
#define	DIO28_WPORT	PORTA
#define	DIO28_DDR		DDRA
#define DIO28_PWM		NULL

#define	DIO29_PIN		PINA7
#define	DIO29_RPORT	PINA
#define	DIO29_WPORT	PORTA
#define	DIO29_DDR		DDRA
#define DIO29_PWM		NULL

#define	DIO30_PIN		PINC7
#define	DIO30_RPORT	PINC
#define	DIO30_WPORT	PORTC
#define	DIO30_DDR		DDRC
#define DIO30_PWM		NULL

#define	DIO31_PIN		PINC6
#define	DIO31_RPORT	PINC
#define	DIO31_WPORT	PORTC
#define	DIO31_DDR		DDRC
#define DIO31_PWM		NULL

#define	DIO32_PIN		PINC5
#define	DIO32_RPORT	PINC
#define	DIO32_WPORT	PORTC
#define	DIO32_DDR		DDRC
#define DIO32_PWM		NULL

#define	DIO33_PIN		PINC4
#define	DIO33_RPORT	PINC
#define	DIO33_WPORT	PORTC
#define	DIO33_DDR		DDRC
#define DIO33_PWM		NULL

#define	DIO34_PIN		PINC3
#define	DIO34_RPORT	PINC
#define	DIO34_WPORT	PORTC
#define	DIO34_DDR		DDRC
#define DIO34_PWM		NULL

#define	DIO35_PIN		PINC2
#define	DIO35_RPORT	PINC
#define	DIO35_WPORT	PORTC
#define	DIO35_DDR		DDRC
#define DIO35_PWM		NULL

#define	DIO36_PIN		PINC1
#define	DIO36_RPORT	PINC
#define	DIO36_WPORT	PORTC
#define	DIO36_DDR		DDRC
#define DIO36_PWM		NULL

#define	DIO37_PIN		PINC0
#define	DIO37_RPORT	PINC
#define	DIO37_WPORT	PORTC
#define	DIO37_DDR		DDRC
#define DIO37_PWM		NULL

#define	DIO38_PIN		PIND7
#define	DIO38_RPORT	PIND
#define	DIO38_WPORT	PORTD
#define	DIO38_DDR		DDRD
#define DIO38_PWM		NULL

#define	DIO39_PIN		PING2
#define	DIO39_RPORT	PING
#define	DIO39_WPORT	PORTG
#define	DIO39_DDR		DDRG
#define DIO39_PWM		NULL

#define	DIO40_PIN		PING1
#define	DIO40_RPORT	PING
#define	DIO40_WPORT	PORTG
#define	DIO40_DDR		DDRG
#define DIO40_PWM		NULL

#define	DIO41_PIN		PING0
#define	DIO41_RPORT	PING
#define	DIO41_WPORT	PORTG
#define	DIO41_DDR		DDRG
#define DIO41_PWM		NULL

#define	DIO42_PIN		PINL7
#define	DIO42_RPORT	PINL
#define	DIO42_WPORT	PORTL
#define	DIO42_DDR		DDRL
#define DIO42_PWM		NULL

#define	DIO43_PIN		PINL6
#define	DIO43_RPORT	PINL
#define	DIO43_WPORT	PORTL
#define	DIO43_DDR		DDRL
#define DIO43_PWM		NULL

#define	DIO44_PIN		PINL5
#define	DIO44_RPORT	PINL
#define	DIO44_WPORT	PORTL
#define	DIO44_DDR		DDRL
#define DIO44_PWM		&OCR5CL

#define	DIO45_PIN		PINL4
#define	DIO45_RPORT	PINL
#define	DIO45_WPORT	PORTL
#define	DIO45_DDR		DDRL
#define DIO45_PWM		&OCR5BL

#define	DIO46_PIN		PINL3
#define	DIO46_RPORT	PINL
#define	DIO46_WPORT	PORTL
#define	DIO46_DDR		DDRL
#define DIO46_PWM		&OCR5AL

#define	DIO47_PIN		PINL2
#define	DIO47_RPORT	PINL
#define	DIO47_WPORT	PORTL
#define	DIO47_DDR		DDRL
#define DIO47_PWM		NULL

#define	DIO48_PIN		PINL1
#define	DIO48_RPORT	PINL
#define	DIO48_WPORT	PORTL
#define	DIO48_DDR		DDRL
#define DIO48_PWM		NULL

#define	DIO49_PIN		PINL0
#define	DIO49_RPORT	PINL
#define	DIO49_WPORT	PORTL
#define	DIO49_DDR		DDRL
#define DIO49_PWM		NULL

#define	DIO50_PIN		PINB3
#define	DIO50_RPORT	PINB
#define	DIO50_WPORT	PORTB
#define	DIO50_DDR		DDRB
#define DIO50_PWM		NULL

#define	DIO51_PIN		PINB2
#define	DIO51_RPORT	PINB
#define	DIO51_WPORT	PORTB
#define	DIO51_DDR		DDRB
#define DIO51_PWM		NULL

#define	DIO52_PIN		PINB1
#define	DIO52_RPORT	PINB
#define	DIO52_WPORT	PORTB
#define	DIO52_DDR		DDRB
#define DIO52_PWM		NULL

#define	DIO53_PIN		PINB0
#define	DIO53_RPORT	PINB
#define	DIO53_WPORT	PORTB
#define	DIO53_DDR		DDRB
#define DIO53_PWM		NULL

#define DIO54_PIN		PINF0
#define DIO54_RPORT	PINF
#define DIO54_WPORT	PORTF
#define DIO54_DDR		DDRF
#define DIO54_PWM		NULL
#define DIO54_ADC		0

#define DIO55_PIN		PINF1
#define DIO55_RPORT	PINF
#define DIO55_WPORT	PORTF
#define DIO55_DDR		DDRF
#define DIO55_PWM		NULL
#define DIO55_ADC		1

#define DIO56_PIN		PINF2
#define DIO56_RPORT	PINF
#define DIO56_WPORT	PORTF
#define DIO56_DDR		DDRF
#define DIO56_PWM		NULL
#define DIO56_ADC		2

#define DIO57_PIN		PINF3
#define DIO57_RPORT	PINF
#define DIO57_WPORT	PORTF
#define DIO57_DDR		DDRF
#define DIO57_PWM		NULL
#define DIO57_ADC		3

#define DIO58_PIN		PINF4
#define DIO58_RPORT	PINF
#define DIO58_WPORT	PORTF
#define DIO58_DDR		DDRF
#define DIO58_PWM		NULL
#define DIO58_ADC		4

#define DIO59_PIN		PINF5
#define DIO59_RPORT	PINF
#define DIO59_WPORT	PORTF
#define DIO59_DDR		DDRF
#define DIO59_PWM		NULL
#define DIO59_ADC		5

#define DIO60_PIN		PINF6
#define DIO60_RPORT	PINF
#define DIO60_WPORT	PORTF
#define DIO60_DDR		DDRF
#define DIO60_PWM		NULL
#define DIO60_ADC		6

#define DIO61_PIN		PINF7
#define DIO61_RPORT	PINF
#define DIO61_WPORT	PORTF
#define DIO61_DDR		DDRF
#define DIO61_PWM		NULL
#define DIO61_ADC		7

#define DIO62_PIN		PINK0
#define DIO62_RPORT	PINK
#define DIO62_WPORT	PORTK
#define DIO62_DDR		DDRK
#define DIO62_PWM		NULL
#define DIO62_ADC		8

#define DIO63_PIN		PINK1
#define DIO63_RPORT	PINK
#define DIO63_WPORT	PORTK
#define DIO63_DDR		DDRK
#define DIO63_PWM		NULL
#define DIO63_ADC		9

#define DIO64_PIN		PINK2
#define DIO64_RPORT	PINK
#define DIO64_WPORT	PORTK
#define DIO64_DDR		DDRK
#define DIO64_PWM		NULL
#define DIO64_ADC		10

#define DIO65_PIN		PINK3
#define DIO65_RPORT	PINK
#define DIO65_WPORT	PORTK
#define DIO65_DDR		DDRK
#define DIO65_PWM		NULL
#define DIO65_ADC		11

#define DIO66_PIN		PINK4
#define DIO66_RPORT	PINK
#define DIO66_WPORT	PORTK
#define DIO66_DDR		DDRK
#define DIO66_PWM		NULL
#define DIO66_ADC		12

#define DIO67_PIN		PINK5
#define DIO67_RPORT	PINK
#define DIO67_WPORT	PORTK
#define DIO67_DDR		DDRK
#define DIO67_PWM		NULL
#define DIO67_ADC		13

#define DIO68_PIN		PINK6
#define DIO68_RPORT	PINK
#define DIO68_WPORT	PORTK
#define DIO68_DDR		DDRK
#define DIO68_PWM		NULL
#define DIO68_ADC		14

#define DIO69_PIN		PINK7
#define DIO69_RPORT	PINK
#define DIO69_WPORT	PORTK
#define DIO69_DDR		DDRK
#define DIO69_PWM		NULL
#define DIO69_ADC		15

#define DIO70_PIN		PING4
#define DIO70_RPORT	PING
#define DIO70_WPORT	PORTG
#define DIO70_DDR		DDRG
#define DIO70_PWM		NULL

#define DIO71_PIN		PING3
#define DIO71_RPORT	PING
#define DIO71_WPORT	PORTG
#define DIO71_DDR		DDRG
#define DIO71_PWM		NULL

#define DIO72_PIN		PINJ2
#define DIO72_RPORT	PINJ
#define DIO72_WPORT	PORTJ
#define DIO72_DDR		DDRJ
#define DIO72_PWM		NULL

#define DIO73_PIN		PINJ3
#define DIO73_RPORT	PINJ
#define DIO73_WPORT	PORTJ
#define DIO73_DDR		DDRJ
#define DIO73_PWM		NULL

#define DIO74_PIN		PINJ7
#define DIO74_RPORT	PINJ
#define DIO74_WPORT	PORTJ
#define DIO74_DDR		DDRJ
#define DIO74_PWM		NULL

#define DIO75_PIN		PINJ4
#define DIO75_RPORT	PINJ
#define DIO75_WPORT	PORTJ
#define DIO75_DDR		DDRJ
#define DIO75_PWM		NULL

#define DIO76_PIN		PINJ5
#define DIO76_RPORT	PINJ
#define DIO76_WPORT	PORTJ
#define DIO76_DDR		DDRJ
#define DIO76_PWM		NULL

#define DIO77_PIN		PINJ6
#define DIO77_RPORT	PINJ
#define DIO77_WPORT	PORTJ
#define DIO77_DDR		DDRJ
#define DIO77_PWM		NULL

#define DIO78_PIN		PINE2
#define DIO78_RPORT	PINE
#define DIO78_WPORT	PORTE
#define DIO78_DDR		DDRE
#define DIO78_PWM		NULL

#define DIO79_PIN		PINE6
#define DIO79_RPORT	PINE
#define DIO79_WPORT	PORTE
#define DIO79_DDR		DDRE
#define DIO79_PWM		NULL

#define DIO80_PIN		PINE7
#define DIO80_RPORT	PINE
#define DIO80_WPORT	PORTE
#define DIO80_DDR		DDRE
#define DIO80_PWM		NULL

#define DIO81_PIN		PIND4
#define DIO81_RPORT	PIND
#define DIO81_WPORT	PORTD
#define DIO81_DDR		DDRD
#define DIO81_PWM		NULL

#define DIO82_PIN		PIND5
#define DIO82_RPORT	PIND
#define DIO82_WPORT	PORTD
#define DIO82_DDR		DDRD
#define DIO82_PWM		NULL

#define DIO83_PIN		PIND6
#define DIO83_RPORT	PIND
#define DIO83_WPORT	PORTD
#define DIO83_DDR		DDRD
#define DIO83_PWM		NULL

#define DIO84_PIN		PINH2
#define DIO84_RPORT	PINH
#define DIO84_WPORT	PORTH
#define DIO84_DDR		DDRH
#define DIO84_PWM		NULL

#define DIO85_PIN		PINH7
#define DIO85_RPORT	PINH
#define DIO85_WPORT	PORTH
#define DIO85_DDR		DDRH
#define DIO85_PWM		NULL

#endif


#ifdef __AVR_ATmega32U4__

#define	SCK_PIN					15
#define	MISO_PIN				14
#define	MOSI_PIN				16
#define SS_PIN					17

#define DIO0_PIN		PIND2
#define DIO0_RPORT	PIND
#define DIO0_WPORT	PORTD
#define DIO0_DDR		DDRD
#define DIO0_PWM		NULL

#define DIO1_PIN		PIND3
#define DIO1_RPORT	PIND
#define DIO1_WPORT	PORTD
#define DIO1_DDR		DDRD
#define DIO1_PWM		NULL

#define DIO2_PIN		PIND1
#define DIO2_RPORT	PIND
#define DIO2_WPORT	PORTD
#define DIO2_DDR		DDRD
#define DIO2_PWM		NULL

#define DIO3_PIN		PIND0
#define DIO3_RPORT	PIND
#define DIO3_WPORT	PORTD
#define DIO3_DDR		DDRD
#define DIO3_PWM		&OCR0B

#define DIO4_PIN		PIND4
#define DIO4_RPORT	PIND
#define DIO4_WPORT	PORTD
#define DIO4_DDR		DDRD
#define DIO4_PWM		NULL
#define DIO4_ADC		8

#define DIO5_PIN		PINC6
#define DIO5_RPORT	PINC
#define DIO5_WPORT	PORTC
#define DIO5_DDR		DDRC
#define DIO5_PWM		&OCR3A

#define DIO6_PIN		PIND7
#define DIO6_RPORT	PIND
#define DIO6_WPORT	PORTD
#define DIO6_DDR		DDRD
#define DIO6_PWM		NULL
#define DIO6_ADC		10

#define DIO7_PIN		PINE6
#define DIO7_RPORT	PINE
#define DIO7_WPORT	PORTE
#define DIO7_DDR		DDRE
#define DIO7_PWM		NULL

#define DIO8_PIN		PINB4
#define DIO8_RPORT	PINB
#define DIO8_WPORT	PORTB
#define DIO8_DDR		DDRB
#define DIO8_PWM		NULL
#define DIO8_ADC		11

#define DIO9_PIN		PINB5
#define DIO9_RPORT	PINB
#define DIO9_WPORT	PORTB
#define DIO9_DDR		DDRB
#define DIO9_PWM		&OCR1A
#define DIO9_ADC		12

#define DIO10_PIN		PINB6
#define DIO10_RPORT	PINB
#define DIO10_WPORT	PORTB
#define DIO10_DDR		DDRB
#define DIO10_PWM		&OCR1B
#define DIO10_ADC		13

#define DIO11_PIN		PINB7
#define DIO11_RPORT	PINB
#define DIO11_WPORT	PORTB
#define DIO11_DDR		DDRB
#define DIO11_PWM		&OCR1C
// #define DIO11_PWM		&OCR0A

#define DIO12_PIN		PIND6
#define DIO12_RPORT	PIND
#define DIO12_WPORT	PORTD
#define DIO12_DDR		DDRD
#define DIO12_PWM		NULL
#define DIO12_ADC		9

#define DIO13_PIN		PINC7
#define DIO13_RPORT	PINC
#define DIO13_WPORT	PORTC
#define DIO13_DDR		DDRC
#define DIO13_PWM		NULL


#define DIO14_PIN		PINB3
#define DIO14_RPORT	PINB
#define DIO14_WPORT	PORTB
#define DIO14_DDR		DDRB
#define DIO14_PWM		NULL

#define DIO15_PIN		PINB1
#define DIO15_RPORT	PINB
#define DIO15_WPORT	PORTB
#define DIO15_DDR		DDRB
#define DIO15_PWM		NULL

#define DIO16_PIN		PINB2
#define DIO16_RPORT	PINB
#define DIO16_WPORT	PORTB
#define DIO16_DDR		DDRB
#define DIO16_PWM		NULL

#define DIO17_PIN		PINB0
#define DIO17_RPORT	PINB
#define DIO17_WPORT	PORTB
#define DIO17_DDR		DDRB
#define DIO17_PWM		NULL

#define DIO18_PIN		PINF7
#define DIO18_RPORT	PINF
#define DIO18_WPORT	PORTF
#define DIO18_DDR		DDRF
#define DIO18_PWM		NULL
#define DIO18_ADC		7

#define DIO19_PIN		PINF6
#define DIO19_RPORT	PINF
#define DIO19_WPORT	PORTF
#define DIO19_DDR		DDRF
#define DIO19_PWM		NULL
#define DIO19_ADC		6

#define DIO20_PIN		PINF5
#define DIO20_RPORT	PINF
#define DIO20_WPORT	PORTF
#define DIO20_DDR		DDRF
#define DIO20_PWM		NULL
#define DIO20_ADC		5

#define DIO21_PIN		PINF4
#define DIO21_RPORT	PINF
#define DIO21_WPORT	PORTF
#define DIO21_DDR		DDRF
#define DIO21_PWM		NULL
#define DIO21_ADC		4

#define DIO22_PIN		PINF1
#define DIO22_RPORT	PINF
#define DIO22_WPORT	PORTF
#define DIO22_DDR		DDRF
#define DIO22_PWM		NULL
#define DIO22_ADC		1

#define DIO23_PIN		PINF0
#define DIO23_RPORT	PINF
#define DIO23_WPORT	PORTF
#define DIO23_DDR		DDRF
#define DIO23_PWM		NULL
#define DIO23_ADC		0

#define DIO24_PIN		PIND4
#define DIO24_RPORT	PIND
#define DIO24_WPORT	PORTD
#define DIO24_DDR		DDRD
#define DIO24_PWM		NULL
#define DIO24_ADC		8

#define DIO25_PIN		PIND7
#define DIO25_RPORT	PIND
#define DIO25_WPORT	PORTD
#define DIO25_DDR		DDRD
#define DIO25_PWM		NULL
#define DIO25_ADC		10

#define DIO26_PIN		PINB4
#define DIO26_RPORT	PINB
#define DIO26_WPORT	PORTB
#define DIO26_DDR		DDRB
#define DIO26_PWM		NULL
#define DIO26_ADC		11

#define DIO27_PIN		PINB5
#define DIO27_RPORT	PINB
#define DIO27_WPORT	PORTB
#define DIO27_DDR		DDRB
#define DIO27_PWM		NULL
#define DIO27_ADC		12

#define DIO28_PIN		PINB6
#define DIO28_RPORT	PINB
#define DIO28_WPORT	PORTB
#define DIO28_DDR		DDRB
#define DIO28_PWM		NULL
#define DIO28_ADC		13

#define DIO29_PIN		PIND6
#define DIO29_RPORT	PIND
#define DIO29_WPORT	PORTD
#define DIO29_DDR		DDRD
#define DIO29_PWM		NULL
#define DIO29_ADC		9

#define DIO30_PIN		PIND5
#define DIO30_RPORT	PIND
#define DIO30_WPORT	PORTD
#define DIO30_DDR		DDRD
#define DIO30_PWM		NULL
#define DIO30_ADC		NULL

#endif


#ifndef	DIO0_PIN
#error pins for this chip not defined in arduino.h! If you write an appropriate pin definition and have this firmware work on your chip, please submit a pull request
#endif
