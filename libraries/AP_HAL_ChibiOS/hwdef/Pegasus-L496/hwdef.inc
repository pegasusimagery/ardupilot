# hw definition file for processing by chibios_pins.py

AUTOBUILD_TARGETS None

# MCU class and specific type
MCU STM32L496 STM32L496xx

# bootloader starts firmware at 36k + 4k (STORAGE_FLASH)
FLASH_RESERVE_START_KB 40

# store parameters in pages 18 and 19
STORAGE_FLASH_PAGE 18
define HAL_STORAGE_SIZE 800

# board ID for firmware load
APJ_BOARD_ID 1300

# setup build for a peripheral firmware
env AP_PERIPH 1

# enable watchdog

# crystal frequency
OSCILLATOR_HZ 0

# assume the 512k flash part
FLASH_SIZE_KB 512

# order of UARTs
SERIAL_ORDER USART2

define HAL_CAN_POOL_SIZE 6000

#STDOUT_SERIAL SD1
#STDOUT_BAUDRATE 57600

# USART1, telemetry
PD5 USART2_TX USART2 SPEED_HIGH
PD6 USART2_RX USART2 SPEED_HIGH

# LEDs
PB7 LED OUTPUT LOW GPIO(1)

# a fault LED
# PB14 LED_FAULT OUTPUT LOW # red

# one I2C bus
PB10 I2C2_SCL I2C2
PB11 I2C2_SDA I2C2

I2C_ORDER I2C2

# allow for reboot command for faster development
define HAL_PERIPH_LISTEN_FOR_SERIAL_UART_REBOOT_CMD_PORT 0

# debugger support
PA13 JTMS-SWDIO SWD
PA14 JTCK-SWCLK SWD

define HAL_NO_GPIO_IRQ
define SERIAL_BUFFERS_SIZE 512

define DMA_RESERVE_SIZE 2048


# stack for fast interrupts
define PORT_INT_REQUIRED_STACK 64

# MAIN_STACK is stack for ISR handlers
MAIN_STACK 0x300

# PROCESS_STACK controls stack for main thread
PROCESS_STACK 0xA00

define HAL_DISABLE_LOOP_DELAY

define HAL_USE_ADC FALSE
define STM32_ADC_USE_ADC1 FALSE

define HAL_NO_MONITOR_THREAD

include ../include/minimize_features.inc

define AP_PARAM_MAX_EMBEDDED_PARAM 512

define CAN_APP_NODE_NAME "org.ardupilot.Pegasus-L496"
