######################################
# target
######################################
TARGET = firmware_liquids


######################################
# building variables
######################################
# debug build?
DEBUG = 1
# optimization
OPT = -Og


#######################################
# paths
#######################################

BUILD_DIR = build


#######################################
# debugger
#######################################

DBG_TARGET = stm32g4x
DBG_INTERFACE = stlink


######################################
# source
######################################

C_SOURCES =  \
Core/Src/stm32g4xx_it.c \
Core/Src/syscalls.c \
Core/Src/sysmem.c \
Core/Src/system_stm32g4xx.c \
Drivers/STM32G491/LL_Driver/Src/stm32g4xx_ll_adc.c \
Drivers/STM32G491/LL_Driver/Src/stm32g4xx_ll_comp.c \
Drivers/STM32G491/LL_Driver/Src/stm32g4xx_ll_cordic.c \
Drivers/STM32G491/LL_Driver/Src/stm32g4xx_ll_crc.c \
Drivers/STM32G491/LL_Driver/Src/stm32g4xx_ll_crs.c \
Drivers/STM32G491/LL_Driver/Src/stm32g4xx_ll_dac.c \
Drivers/STM32G491/LL_Driver/Src/stm32g4xx_ll_dma.c \
Drivers/STM32G491/LL_Driver/Src/stm32g4xx_ll_exti.c \
Drivers/STM32G491/LL_Driver/Src/stm32g4xx_ll_fmac.c \
Drivers/STM32G491/LL_Driver/Src/stm32g4xx_ll_gpio.c \
Drivers/STM32G491/LL_Driver/Src/stm32g4xx_ll_hrtim.c \
Drivers/STM32G491/LL_Driver/Src/stm32g4xx_ll_i2c.c \
Drivers/STM32G491/LL_Driver/Src/stm32g4xx_ll_lptim.c \
Drivers/STM32G491/LL_Driver/Src/stm32g4xx_ll_lpuart.c \
Drivers/STM32G491/LL_Driver/Src/stm32g4xx_ll_opamp.c \
Drivers/STM32G491/LL_Driver/Src/stm32g4xx_ll_pwr.c \
Drivers/STM32G491/LL_Driver/Src/stm32g4xx_ll_rcc.c \
Drivers/STM32G491/LL_Driver/Src/stm32g4xx_ll_rng.c \
Drivers/STM32G491/LL_Driver/Src/stm32g4xx_ll_rtc.c \
Drivers/STM32G491/LL_Driver/Src/stm32g4xx_ll_spi.c \
Drivers/STM32G491/LL_Driver/Src/stm32g4xx_ll_tim.c \
Drivers/STM32G491/LL_Driver/Src/stm32g4xx_ll_ucpd.c \
Drivers/STM32G491/LL_Driver/Src/stm32g4xx_ll_usart.c \
Drivers/STM32G491/LL_Driver/Src/stm32g4xx_ll_utils.c \
Drivers/STM32G491/STRHAL/Src/STRHAL.c \
Drivers/STM32G491/STRHAL/Src/STRHAL_ADC.c \
Drivers/STM32G491/STRHAL/Src/STRHAL_CAN.c \
Drivers/STM32G491/STRHAL/Src/STRHAL_Clock.c \
Drivers/STM32G491/STRHAL/Src/STRHAL_Container.c \
Drivers/STM32G491/STRHAL/Src/STRHAL_GPIO.c \
Drivers/STM32G491/STRHAL/Src/STRHAL_OPAMP.c \
Drivers/STM32G491/STRHAL/Src/STRHAL_QSPI.c \
Drivers/STM32G491/STRHAL/Src/STRHAL_SPI.c \
Drivers/STM32G491/STRHAL/Src/STRHAL_I2C.c \
Drivers/STM32G491/STRHAL/Src/STRHAL_SysTick.c \
Drivers/STM32G491/STRHAL/Src/STRHAL_TIM.c \
Drivers/STM32G491/STRHAL/Src/STRHAL_UART.c

CPP_SOURCES = \
Core/Src/AbstractCom.cpp \
Core/Src/Can.cpp \
Core/Src/ECU.cpp \
Core/Src/main.cpp \
Core/Src/Speaker.cpp \
Core/Src/Radio.cpp \
Core/Src/Channels/ADCChannel.cpp \
Core/Src/Channels/AbstractChannel.cpp \
Core/Src/Channels/DigitalInChannel.cpp \
Core/Src/Channels/DigitalOutChannel.cpp \
Core/Src/Channels/GenericChannel.cpp \
Core/Src/Channels/IMUChannel.cpp \
Core/Src/Channels/PressureControlChannel.cpp \
Core/Src/Channels/PyroChannel.cpp \
Core/Src/Channels/ServoChannel.cpp \
Core/Src/Modules/W25Qxx_Flash.cpp \
Core/Src/Modules/VL53L1X.cpp

ASM_SOURCES =  \
Core/Startup/startup_stm32g491metx.s


#######################################
# binaries
#######################################

PREFIX = arm-none-eabi-
POSTFIX = "
# The gcc compiler bin path can be either defined in make command via GCC_PATH variable (> make GCC_PATH=xxx)
# or it can be added to the PATH environment variable.
GCC_PATH="/opt/gcc-arm-none-eabi-9-2020-q2-update/bin
ifdef GCC_PATH
CXX = $(GCC_PATH)/$(PREFIX)g++$(POSTFIX)
CC = $(GCC_PATH)/$(PREFIX)gcc$(POSTFIX)
AS = $(GCC_PATH)/$(PREFIX)gcc$(POSTFIX) -x assembler-with-cpp
CP = $(GCC_PATH)/$(PREFIX)objcopy$(POSTFIX)
SZ = $(GCC_PATH)/$(PREFIX)size$(POSTFIX)
else
CXX = $(PREFIX)g++
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size
endif
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S


#######################################
# CFLAGS
#######################################

# cpu
CPU = -mcpu=cortex-m4

# fpu
FPU = -mfpu=fpv4-sp-d16

# float-abi
FLOAT-ABI = -mfloat-abi=hard

# mcu
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

# macros for gcc
# AS defines
AS_DEFS = 

# C defines
C_DEFS =  \
-DSTM32G491xx \
-DUSE_FULL_LL_DRIVER \
-D__STATIC_INLINE='static inline'

# CXX defines
CXX_DEFS =  \
-DSTM32G491xx \
-DUSE_FULL_LL_DRIVER \
-D__STATIC_INLINE='static inline' \
-DECU_BOARD

# AS includes
AS_INCLUDES = \

# C includes
C_INCLUDES =  \
-I$(GCC_PATH)"/../arm-none-eabi/include \
-ICore/Inc \
-ICore/Inc/can_houbolt \
-IDrivers/STM32G491/CMSIS/Device/ST/STM32G4xx/Include \
-IDrivers/STM32G491/CMSIS/Include \
-IDrivers/STM32G491/LL_Driver/Inc \
-IDrivers/STM32G491/STRHAL/Inc

# compile gcc flags
ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

CFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

CXXFLAGS = $(MCU) $(CXX_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections -feliminate-unused-debug-types

ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf -ggdb
CXXFLAGS += -g -gdwarf -ggdb
endif

# Add additional flags
CFLAGS += -Wall -fdata-sections -ffunction-sections -specs=nano.specs 
ASFLAGS += -Wall -fdata-sections -ffunction-sections -specs=nano.specs 
CXXFLAGS += -Wall -fdata-sections -ffunction-sections -specs=nano.specs 

# Generate dependency information
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"
CXXFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"


#######################################
# LDFLAGS
#######################################

# link script
LDSCRIPT = Linker/STM32G491METX_FLASH.ld

# libraries
LIBS = -lm -lstdc++ -lsupc++ 
LIBDIR = \


# Additional LD Flags from config file
ADDITIONALLDFLAGS = 

LDFLAGS = $(MCU) $(ADDITIONALLDFLAGS) -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections


#######################################
# build
#######################################

# default action: build all
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin

# list of cpp program objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(CPP_SOURCES:.cpp=.o)))
vpath %.cpp $(sort $(dir $(CPP_SOURCES)))

# list of C objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

$(BUILD_DIR)/%.o: %.cpp | $(BUILD_DIR) 
	$(CXX) -c $(CXXFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.cpp=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.cxx | $(BUILD_DIR) 
	$(CXX) -c $(CXXFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.cxx=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.c | $(BUILD_DIR) 
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS)
	$(CXX) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@

$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@

$(BUILD_DIR):
	mkdir $@


#######################################
# flash
#######################################

flash: all
	openocd -f interface/$(DBG_INTERFACE).cfg -f target/$(DBG_TARGET).cfg -c "program $(BUILD_DIR)/$(TARGET).elf verify; reset run; exit"


#######################################
# erase
#######################################

erase:
	openocd -f interface/$(DBG_INTERFACE).cfg -f target/$(DBG_TARGET).cfg -c "init; reset halt; $(DBG_TARGET) mass_erase 0; exit"


#######################################
# clean
#######################################

clean:
	-rm -fR $(BUILD_DIR)
