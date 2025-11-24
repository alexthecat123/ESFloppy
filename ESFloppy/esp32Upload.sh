#!/bin/zsh

arduino-cli compile --fqbn esp32:esp32:esp32 && arduino-cli upload --fqbn esp32:esp32:esp32 -p /dev/cu.SLAB_USBtoUART
#arduino-cli compile --fqbn esp32:esp32:esp32s3 && arduino-cli upload --fqbn esp32:esp32:esp32s3 -p /dev/cu.usbserial-4