CC		= arm-none-eabi-gcc
CXX		= arm-none-eabi-g++
AS		= arm-none-eabi-as
LD		= arm-none-eabi-gcc
OBJCOPY	= arm-none-eabi-objcopy

BUILD_DIR	= build
OBJ_DIR		= $(BUILD_DIR)/Object_Files
SRC_DIR		= src
INC_DIRS	= include include/FreeRTOS_include include/CMSIS
LIB_DIR		= lib

MCU			= -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard
CFLAGS		= $(MCU) -Wall -Wextra -O2 -ffunction-sections -fdata-sections
CXXFLAGS	= $(CFLAGS) -fno-exceptions -fno-rtti
LDFLAGS		= $(MCU) -Wl,--gc-sections,--no-warn-rwx-segments --specs=nosys.specs -TMIMXRT1062xxxxx_flexspi_nor.ld

INCLUDES	= $(addprefix -I, $(INC_DIRS))
LIBS		= -L$(LIB_DIR) -lfreertos

C_SOURCES	= $(wildcard $(SRC_DIR)/*.c)
CPP_SOURCES	= $(wildcard $(SRC_DIR)/*.cpp)
ASM_SOURCES = $(wildcard *.S)
OBJS 		= $(addprefix $(OBJ_DIR)/, $(notdir $(C_SOURCES:.c=.o)) \
        		$(notdir $(CPP_SOURCES:.cpp=.o)) \
        		$(notdir $(ASM_SOURCES:.S=.o)))

TARGET = firmware

###############################################################################
# Build rules
###############################################################################

ifeq ($(OS),Windows_NT)
    MKDIR = if not exist "$(1)" mkdir "$(1)"
else
    MKDIR = mkdir -p $(1)
endif

all: $(BUILD_DIR) $(OBJ_DIR) $(LIB_DIR) $(LIB_DIR)/libfreertos.a $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex

$(BUILD_DIR):
	$(call MKDIR,$@)

$(OBJ_DIR):
	$(call MKDIR,$@)

$(LIB_DIR):
	$(call MKDIR,$@)

$(LIB_DIR)/libfreertos.a:
	$(MAKE) -C External

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.c | $(OBJ_DIR)
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp | $(OBJ_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

$(OBJ_DIR)/%.o: %.S | $(OBJ_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJS)
	$(LD) $(LDFLAGS) $^ $(LIBS) -o $@

$(BUILD_DIR)/$(TARGET).hex: $(BUILD_DIR)/$(TARGET).elf
	$(OBJCOPY) -O ihex $< $@

###############################################################################
# Utility targets
###############################################################################

clean:
ifeq ($(OS),Windows_NT)
	if exist "$(BUILD_DIR)\*.elf" del /Q "$(BUILD_DIR)\*.elf"
	if exist "$(BUILD_DIR)\*.hex" del /Q "$(BUILD_DIR)\*.hex"
	if exist "$(BUILD_DIR)\Object_Files\*.o" del /Q "$(BUILD_DIR)\Object_Files\*.o"
else
	rm -f $(BUILD_DIR)/*.elf $(BUILD_DIR)/*.hex $(OBJ_DIR)/*.o
endif

clean_all:
ifeq ($(OS),Windows_NT)
	if exist "$(BUILD_DIR)\*.elf" del /Q "$(BUILD_DIR)\*.elf"
	if exist "$(BUILD_DIR)\*.hex" del /Q "$(BUILD_DIR)\*.hex"
	if exist "$(BUILD_DIR)\Object_Files\*.o" del /Q "$(BUILD_DIR)\Object_Files\*.o"
	$(MAKE) -C External clean
else
	rm -f $(BUILD_DIR)/*.elf $(BUILD_DIR)/*.hex $(OBJ_DIR)/*.o
	$(MAKE) -C External clean
endif

.PHONY: all clean
