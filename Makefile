DEVICE=atmega2560
CFLAGS=-mmcu=$(DEVICE) -Wall -Werror
LDFLAGS=-mmcu=$(DEVICE) -lm
ASFLAGS=-mmcu=$(DEVICE)

CC=avr-gcc 
CPP=avr-gcc -E
CXX=avr-g++ 

LD=avr-gcc

# include implicit rules for arduino
include Makefile.implicit
include Makefile.device

TRG=main

all: $(TRG).hex

main.elf: main.o pwm.o motor.o serial.o power.o adc.o system.o servo.o gps.o sonar.o

download: $(TRG).hex
	avrdude -pm2560 -P${COM} -cstk500v2 -u -U flash:w:$(TRG).hex
# I'm a little worried that I don't need to specify a baud rate, but it works
