APP_NAME=usb

BIN_DIR=bin
OBJ_DIR=obj

EXECUTABLE=$(APP_NAME)
LINK_SCRIPT=stm32f0.ld

CCARCH=arm-none-eabi-
CC=$(CCARCH)gcc
CPP=$(CCARCH)g++
AR=$(CCARCH)ar
OBJCOPY=$(CCARCH)objcopy
SIZE=$(CCARCH)size
OBJDUMP=$(CCARCH)objdump
READELF=$(CCARCH)readelf
ELF=$(OBJ_DIR)/$(EXECUTABLE)

GCCPATH=$(shell $(CC) -print-search-dirs | awk '/install/{print $$2}')

CPUARCH   = -mcpu=cortex-m0 -mthumb -mlittle-endian
BASEFLAGS = -DNDEBUG -Wall -Werror -g3 -Os -fomit-frame-pointer -falign-functions=16 -ffunction-sections -fdata-sections -fno-common -flto $(CPUARCH) -ffreestanding --specs=nosys.specs
BASEFLAGS += -I CMSIS_5-5.7.0/CMSIS/Core/Include/ -I src -fdata-sections -ffunction-sections -mfloat-abi=soft -DSTM32F072 -DF_CPU=48000000

CFLAGS    = $(BASEFLAGS)
CPPFLAGS  = $(BASEFLAGS) -std=c++17 -fno-use-cxa-atexit -fno-rtti
LDFLAGS   = -T link/$(LINK_SCRIPT) -nostartfiles $(BASEFLAGS) $(CPUARCH)  -Wl,--gc-sections 


SOURCES_SRC = $(shell find src/ -iname "*.c")
SOURCES_CPP_SRC = $(shell find src/ -iname "*.cpp")
INCLUDES_SRC = $(shell find src/ -iname "*.h")

OBJECTS_OBJ = $(addprefix obj/, $(subst src/,,$(SOURCES_SRC:.c=.o) $(subst src/,,$(SOURCES_CPP_SRC:.cpp=.o))))

all: bin/$(APP_NAME).bin

bin/$(APP_NAME).bin: $(ELF)
	@mkdir -p bin
	$(OBJCOPY) $< -O binary $@

bin/$(APP_NAME).hex: $(ELF)
	@mkdir -p bin
	$(OBJCOPY) $< -O ihex $@

$(ELF): $(OBJECTS_OBJ) link/$(LINK_SCRIPT)
	$(CPP) -o $(@) $(OBJECTS_OBJ) $(LDFLAGS)
	$(SIZE) $(ELF)
			
obj/%.o: src/%.c $(INCLUDES_SRC) $(MAKEFILE_LIST)
	@mkdir -p $(dir $@)
	$(CC) -c $(CFLAGS) $< -o $@

obj/%.o: src/%.cpp $(INCLUDES_SRC) $(MAKEFILE_LIST)
	@mkdir -p $(dir $@)
	$(CPP) -c $(CPPFLAGS) $< -o $@

clean:
	$(RM) $(OBJECTS_OBJ) $(ELF) bin/$(APP_NAME).bin bin/$(APP_NAME).hex
	$(RM) -r obj
	$(RM) -r bin

.PHONY3 raminfo: $(ELF)
	@$(OBJDUMP) -x $(ELF) |grep \\.bss |grep "O \\."
	@$(OBJDUMP) -x $(ELF) |grep \\.data |grep "O \\."

.PHONY_flash flash: bin/$(APP_NAME).bin
	st-flash erase
	st-flash write bin/$(APP_NAME).bin 0x8000000
	st-flash reset
