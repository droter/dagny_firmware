DEVICE=atmega2560
#CFLAGS=-mmcu=$(DEVICE) -Wall -Werror -O -Iros_lib
CFLAGS=-mmcu=$(DEVICE) -Wall -Werror -Iros_lib -O -DF_CPU=16000000UL -Idagny_protocol
LDFLAGS=-mmcu=$(DEVICE)
ASFLAGS=-mmcu=$(DEVICE)
CXXFLAGS=$(CFLAGS)

LDLIBS=-lm 

include Makefile.avr

VPATH=drivers:dagny_protocol

CSRC=motor.c i2c.c estop.c encoder.c
CXXSRC=gps.cpp interrupt.cpp main.cpp steer.cpp TinyGPS.cpp sonar.cpp imu.cpp protocol.cpp battery.cpp
DRIVERS=adc.o bump.o power.o pwm.o serial.o serial-interrupt.o servo.o

OBJS=$(CSRC:.c=.o) $(CXXSRC:.cpp=.o)

# include implicit rules for arduino
include Makefile.implicit

# include computer-specific file defining programmer port
include Makefile.device

TRG=main

.PHONY: all
all: $(TRG).hex

main.elf: $(OBJS) $(DRIVERS)

drivers/libdrivers.a:
	$(MAKE) -C drivers

program: $(TRG).hex
	avrdude -pm2560 -cusbtiny -u -U flash:w:$(TRG).hex -B 1
#	avrdude -pm2560 -P${COM} -cstk500v2 -u -U flash:w:$(TRG).hex
#	avrdude -pm2560 -P${COM} -b115200 -cstk500v2 -u -U flash:w:$(TRG).hex
#  no need to specify baud rate with new Arduio UNO/Mega 2560 programmer
	touch program

.PHONY: size
size: $(TRG).elf
	avr-size $(TRG).elf

.PHONY: clean
clean:
	rm *.o *.i *.ii *.s *.hex || true

.PHONY: lines
lines:
	./lines.pl *.c *.cpp *.h drivers/*.h drivers/*.c   

#include dependency rules
include $(CSRC:%.c=.%.mk)
include $(CXXSRC:%.cpp=.%.mk)
