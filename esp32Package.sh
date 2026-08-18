#!/bin/bash

# Compiles the ESFloppy firmware and then packages it into esfloppy_fw.bin in the build/ directory
# This file can be copied to the root of an SD card and will auto-update the firmware on the next boot

arduino-cli compile ESFloppy/ --fqbn esp32:esp32:esp32s3:CDCOnBoot=cdc --output-dir build && python3 package_fw.py && echo "Firmware packaged into build/esfloppy_fw.bin and ready to be copied to an SD card!"