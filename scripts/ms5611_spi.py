#!/usr/bin/env python
# encoding: utf-8
"""
MS5611-01BA0 test in SPI mode.

Copyright 2015 Arvid Juskaitis <arvydas.juskaitis@gmail.com>
"""
import sys
from pyBusPirateLite.SPI import *

spi = SPI("/dev/ttyUSB0", 115200)

print "Entering binmode: ",
if spi.BBmode():
	print "OK."
else:
	print "failed."
	sys.exit()

print "Entering raw SPI mode: ",
if spi.enter_SPI():
	print "OK."
else:
	print "failed."
	sys.exit()
	
print "Configuring SPI."
if not spi.cfg_pins(PinCfg.POWER | PinCfg.CS | PinCfg.AUX):
	print "Failed to set SPI peripherals."
	sys.exit()
if not spi.set_speed(SPISpeed._250KHZ):
	print "Failed to set SPI Speed."
	sys.exit()
if not spi.cfg_spi(SPICfg.CLK_EDGE | SPICfg.OUT_TYPE):
	print "Failed to set SPI configuration.";
	sys.exit()
spi.timeout(0.2)

print "Reset: ",
spi.CS_Low()
spi.bulk_trans(1, [0x1E])
spi.CS_High()
print "OK."

print "Read calibration values: "
for addr in [0xA2, 0xA4, 0xA6, 0xA8, 0xAA, 0xAC]:
	spi.CS_Low()
	d = spi.bulk_trans(3, [addr, 0, 0])
	spi.CS_High()
	print "%02x: %02x%02x" % (addr, ord(d[1]), ord(d[2]))

print "ADC conversion, OSR=4096, D1: "
spi.CS_Low()
spi.bulk_trans(1, [0x48])
spi.CS_High()
time.sleep(.01)
spi.CS_Low()
d = spi.bulk_trans(4, [0, 0, 0, 0])
spi.CS_High()
print "%02x%02x%02x" % (ord(d[1]), ord(d[2]), ord(d[3]))

print "ADC conversion, OSR=4096, D2: "
spi.CS_Low()
spi.bulk_trans(1, [0x58])
spi.CS_High()
time.sleep(.01)
spi.CS_Low()
d = spi.bulk_trans(4, [0, 0, 0, 0])
spi.CS_High()
print "%02x%02x%02x" % (ord(d[1]), ord(d[2]), ord(d[3]))

print "Reset Bus Pirate to user terminal: ",
if spi.resetBP():
	print "OK."
else:
	print "failed."
	sys.exit()
	
