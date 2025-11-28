CC = arm-none-eabi-gcc
AR = arm-none-eabi-ar

SRC_DIR = FreeRTOS_src
BUILD_DIR = FreeRTOS_build
LIB_DIR = ../lib
INCLUDE_DIR = ../include/FreeRTOS_include

CFLAGS = -Wall -O2 -I$(INCLUDE_DIR) -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard

LIB_NAME = libfreertos.a

SRCS = $(wildcard $(SRC_DIR)/*.c)
OBJS = $(patsubst $(SRC_DIR)/%.c,$(BUILD_DIR)/%.o,$(SRCS))

ifeq ($(OS),Windows_NT)
    MKDIR = if not exist "$(1)" mkdir "$(1)"
else
    MKDIR = mkdir -p $(1)
endif

all: $(BUILD_DIR) $(LIB_DIR)/$(LIB_NAME)

$(BUILD_DIR):
	$(call MKDIR,$@)

$(BUILD_DIR)/%.o: $(SRC_DIR)/%.c $(INCLUDE_DIR)/*.h
	$(CC) $(CFLAGS) -c $< -o $@

$(LIB_DIR)/$(LIB_NAME): $(OBJS)
	$(AR) rcs $@ $(OBJS)

clean:
ifeq ($(OS),Windows_NT)
	if exist "$(BUILD_DIR)\*.o" del /Q "$(BUILD_DIR)\*.o"
	if exist "..\lib\$(LIB_NAME)" del /Q "..\lib\$(LIB_NAME)"
else
	rm -rf $(BUILD_DIR)/*.o $(LIB_DIR)/$(LIB_NAME)
endif

.PHONY: all clean