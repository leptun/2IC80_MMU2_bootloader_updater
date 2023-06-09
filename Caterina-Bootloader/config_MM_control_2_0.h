#pragma once

#define DEVICE_VID 0x2C99UL
#define DEVICE_PID 0x0003UL
#define PRODUCT_STRING L"Original Prusa i3 MK3 Multi Material 2.0 upgrade (bootloader)"
#define MANUFACTURER_NAME_STRING L"Prusa Research (prusa3d.com)"

/** Seven character bootloader firmware identifier reported to the host when requested */
#define SOFTWARE_IDENTIFIER "MMU2   "

#define TIMEOUT_PERIOD 8000U


/** UART1 bootloader support */
#define UART1_BOOT
#define UART1_BAUD 115200UL


#define RD_LED_PIN 30
#define WR_LED_PIN 17
#define RD_LED_POL 0 //active low
#define WR_LED_POL 0

#define TMC1_CS_PIN 5
#define TMC2_CS_PIN 6
#define TMC3_CS_PIN 11

#define SHR16_DS_PIN 9
#define SHR16_STCP_PIN 10
#define SHR16_SHCP_PIN 13
