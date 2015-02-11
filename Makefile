CXX=avr-gcc
INCLUDE=-I ~/avr-projects/include/arduino/
LIBS=-L ~/avr-projects/lib -lm -larduino
MCU=-mmcu=atmega328p
CPU_SPEED=-DF_CPU=8000000UL
CFLAGS=$(MCU) $(CPU_SPEED) -Os -w -Wl,--gc-sections -ffunction-sections -fdata-sections
PORT=/dev/tty.usbserial-A600acii
ifeq ($(shell uname),Linux)
	PORT=/dev/ttyUSB0
endif

.PHONY: all

all: build upload

build: htsss.hex

htsss.hex: htsss.elf
	avr-objcopy -O ihex $< $@

OBJECTS= # Put other objects here
htsss.elf: src/htsss.cpp $(OBJECTS)
	$(CXX) $(CFLAGS) $(INCLUDE) $^ -o $@ $(LIBS)

upload:
	avrdude -V -F -p atmega328p -c arduino -b 57600 -Uflash:w:htsss.hex -P$(PORT)

clean:
	@echo -n Cleaning ...
	$(shell rm htsss.elf 2> /dev/null)
	$(shell rm htsss.hex 2> /dev/null)
	$(shell rm *.o 2> /dev/null)
	@echo " done"

%.o: src/%.c
	$(CXX) $< $(CFLAGS) $(INCLUDE) -c -o $@

%.o: src/%.cpp
	$(CXX) $< $(CFLAGS) $(INCLUDE) -c -o $@
	
