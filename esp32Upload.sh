#!/bin/bash

arduino-cli compile ESFloppy/ --fqbn esp32:esp32:esp32s3:CDCOnBoot=cdc && arduino-cli upload ESFloppy/ --fqbn esp32:esp32:esp32s3:CDCOnBoot=cdc -p /dev/cu.usbmodem11301