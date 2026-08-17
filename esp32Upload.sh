#!/bin/bash

arduino-cli compile ESFloppy/ --fqbn esp32:esp32:esp32s3:CDCOnBoot=cdc --build-property "compiler.cpp.extra_flags=-DUSE_SPI_ARRAY_TRANSFER=1" && arduino-cli upload ESFloppy/ --fqbn esp32:esp32:esp32s3:CDCOnBoot=cdc -p /dev/cu.usbmodem11301