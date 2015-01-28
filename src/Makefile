CXX=avr-gcc
INCLUDE=-I ~/include/arduino/
LIBS=-L ~/lib -lm -larduino
MCU=-mmcu=atmega328p
CPU_SPEED=-DF_CPU=16000000UL
CFLAGS=$(MCU) $(CPU_SPEED) -Os -w -Wl,--gc-sections -ffunction-sections -fdata-sections
PORT=/dev/ttyUSB0

default: build upload

build: htsss.hex

htsss.hex: htsss.elf
	avr-objcopy -O ihex $< $@

OBJECTS= # Put other objects here
htsss.elf: htsss.cpp $(OBJECTS)
	$(CXX) $(CFLAGS) $(INCLUDE) $^ -o $@ $(LIBS)

upload:
	avrdude -V -F -p m328p -c arduino -b 115200 -Uflash:w:htsss.hex -P$(PORT)

clean:
	@echo -n Cleaning ...
	$(shell rm htsss.elf 2> /dev/null)
	$(shell rm htsss.hex 2> /dev/null)
	$(shell rm *.o 2> /dev/null)
	@echo " done"

%.o: %.cpp
	$(CXX) $< $(CFLAGS) $(INCLUDE) -c -o $@
