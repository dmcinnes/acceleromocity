# Arduino Make file. Refer to https://github.com/sudar/Arduino-Makefile

ARDMK_DIR = /usr/local/opt/arduino-mk

ALTERNATE_CORE = Microduino
MICRODUINO_DIR = /Users/doug/Library/Arduino15/packages/Microduino
ALTERNATE_CORE_PATH = $(MICRODUINO_DIR)/hardware/avr/1.0.1
BOARD_TAG = 328p
BOARD_SUB = 16MHzatmega328

include $(ARDMK_DIR)/Arduino.mk
