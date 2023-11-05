# Output Binary name
OUTPUT = final
# Directories
TOP=../kern
TARGET = target
OBJECT = object
USERLAND = ../userland
MAPDIR = mapfiles
MCU=stm32f446re

# $(TOP)/lib/kmath.c \
# $(TOP)/lib/kfloat.c \
# $(TOP)/lib/syscall.c \

debug: CFLAGS += -g3
debug: all

all: $(TARGET)/$(OUTPUT)

# C Source file list
C_SOURCES =  \
	$(TOP)/lib/sys_init.c \
	$(TOP)/lib/kstring.c \
	$(TOP)/lib/kmath.c \
	$(TOP)/lib/kfloat.c \
	$(TOP)/arch/$(MCU)/dev/clock.c \
	$(TOP)/arch/$(MCU)/dev/gpio.c \
	$(TOP)/arch/$(MCU)/dev/usart.c \
	$(TOP)/arch/$(MCU)/dev/i2c.c \
	$(TOP)/arch/$(MCU)/dev/timer.c \
	$(TOP)/arch/cm4/cm4.c \
	$(TOP)/arch/$(MCU)/sys/stm32_startup.c \
	$(TOP)/lib/ktimes.c \
	$(TOP)/kmain/kstdio.c \
	$(TOP)/kmain/kmain.c \
	$(USERLAND)/device/mpu9255/mpu9255.c \
	$(USERLAND)/device//bmp280/bmp280.c \
	$(USERLAND)/main.c 


# fpu
FPU = -mfpu=fpv4-sp-d16 -mfloat-abi=hard
# machine
MACH=cortex-m4
# Compiler
CC=arm-none-eabi-gcc
# Compiler Flags
CFLAGS = -c -mcpu=$(MACH) -mthumb $(FPU) -std=gnu11 -Wall -O0
# Linker Flags
LIBS = -lc -lm -lnosys 
LDFLAGS = -nostartfiles -mcpu=$(MACH) -mthumb $(FPU)  -T $(TOP)/arch/$(MCU)/linker/linker.ld $(LIBS) -Wl,-Map=$(MAPDIR)/$(OUTPUT).map,--cref
# include paths
INCLUDE=-I$(TOP)/arch/include \
 -I$(TOP)/arch/$(MCU)/include \
 -I$(TOP)/include -I$(TOP)/include/kern \
 -I$(TOP)/arch/$(MCU)/include \
 -I$(TOP)/arch/$(MCU)/include/dev \
 -I$(TOP)/arch/include/cm4 \
 -I$(TOP)/arch/$(MCU)/include/sys \
 -I$(USERLAND)/include


# list of objects
OBJECTS = $(addprefix $(OBJECT)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))


$(OBJECT)/%.o: %.c Makefile | $(OBJECT) 
	$(CC) -c $(CFLAGS) $(INCLUDE) $< -o $@

$(TARGET)/$(OUTPUT): $(OBJECTS) Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	
# Make directories
$(OBJECT):
	mkdir $@

$(TARGET):
	mkdir $@

$(MAPDIR):
	mkdir $@



clean:
	rm -rf $(OBJECT)/*.o $(MAPDIR)/*.map $(TARGET)/* *.o *.elf


objects:
	echo $(OBJECTS)


# To Students -- Modify to match with the appropriate driver file.
BOARDCFG = /usr/share/openocd/scripts/board/stm32f4discovery.cfg
STLINKCFG = /usr/share/openocd/scripts/interface/stlink.cfg

load:
	openocd -f $(BOARDCFG) -f $(STLINKCFG) -c "program target/$(OUTPUT) verify reset exit"


# debug:
# 	openocd -f board/st_nucleo_f4.cfg