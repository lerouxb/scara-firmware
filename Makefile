ARDUINO_DIR   = /Applications/Arduino.app/Contents/Resources/Java
ARDMK_DIR     = /usr/local/opt/arduino-mk/
MONITOR_PORT  = /dev/ttyACM0
ARCHITECTURE  = arm
#BOARD_TAG     = leonardo
BOARD_TAG     = teensy31
#AVR_TOOLS_DIR = /usr/local
AVR_TOOLS_DIR = /Applications/Arduino.app/Contents/Resources/Java/hardware/tools/arm/
ARDUINO_LIBS  = EEPROM PID SPI AS5048A
MONITOR_PORT  = /dev/tty.usbmodem1421
MONITOR_BAUDRATE = 115200
MCU = cortex-m4
#MONITOR_PARAMS = inlcr,onlcr,echo
#include /usr/local/opt/arduino-mk/Arduino.mk
include /usr/local/opt/arduino-mk/Teensy.mk
