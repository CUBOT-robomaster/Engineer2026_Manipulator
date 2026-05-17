# Minimal GNU Make build for STM32H750VBTx (GCC from STM32CubeCLT)
# Run from project root:  make

# 显式指定编译器所在的目录
BIN_PATH := D:/STM32CubeCLT_1.21.0/GNU-tools-for-STM32/bin

# 将该路径前置插入到当前的系统环境变量中
export PATH := $(BIN_PATH);$(PATH)

# 强制指定使用 Windows 原生 cmd 作为 shell，避免 PowerShell 路径转义问题
SHELL := cmd.exe

PREFIX     ?= arm-none-eabi-
CC          = $(PREFIX)gcc
AS          = $(PREFIX)gcc -x assembler-with-cpp
OBJCOPY     = $(PREFIX)objcopy
SIZE        = $(PREFIX)size

TARGET      = TEXT_USART
BUILD_DIR   = build
LDSCRIPT    = STM32H750VBTx_FLASH.ld

# Windows: "gcc --version" only runs the driver; compiling invokes cc1.exe beside it. Make often
# inherits a shorter PATH than your interactive shell, so cc1 can fail with 0xC0000135 (-1073741515).
# We prepend GNU Tools bin to PATH. Optional override: make TOOLCHAIN_BIN=D:/.../GNU-tools-for-STM32/bin
ifeq ($(OS),Windows_NT)
  ifneq ($(strip $(TOOLCHAIN_BIN)),)
    export PATH := $(TOOLCHAIN_BIN);$(PATH)
  else
    _where_gcc := $(subst \,/,$(firstword $(shell cmd /c "where $(PREFIX)gcc 2>nul")))
    ifneq ($(strip $(_where_gcc)),)
      TOOLBIN := $(subst /,\,$(patsubst %/,%,$(dir $(_where_gcc))))
      export PATH := $(TOOLBIN);$(PATH)
    endif
  endif
# cmd has no "mkdir -p"; avoid trailing \ inside quotes (breaks if exist). Win10+ mkdir creates parents.
win_dir_for = $(subst /,\,$(patsubst %/,%,$(subst \,/,$(dir $(1)))))
MKDIR_OBJDIR = @if not exist "$(call win_dir_for,$@)" mkdir "$(call win_dir_for,$@)"
else
MKDIR_OBJDIR = @mkdir -p $(dir $@)
endif

# MCU / ABI (matches STM32H750 Cortex-M7 + double-precision FPU)
CPU_FLAGS   = -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard

# Keil project defines + CMSIS-DSP (only sin table + arm_sin_f32 linked)
DEFS        = -DUSE_HAL_DRIVER -DSTM32H750xx \
              -DARM_MATH_CM7 -DARM_MATH_MATRIX_CHECK -DARM_MATH_ROUNDING \
              -DARM_DSP_CONFIG_TABLES -DARM_FAST_ALLOW_TABLES -DARM_TABLE_SIN_F32

INCLUDES    = -ICore/Inc \
              -IDrivers/STM32H7xx_HAL_Driver/Inc \
              -IDrivers/STM32H7xx_HAL_Driver/Inc/Legacy \
              -IDrivers/CMSIS/Device/ST/STM32H7xx/Include \
              -IDrivers/CMSIS/Include \
              -IDrivers/CMSIS/DSP/Include \
              -ICubotMiddleware/dr16 \
              -IUser \
              -ICubotMiddleware/drivers \
              -ICubotMiddleware/cubotdevice/motor \
              -ICubotMiddleware/Algorithms \
              -ICubotMiddleware/Support \
              -ICubotMiddleware/cubotdevice/check \
              -ICubotMiddleware/cubotdevice \
              -ICubotMiddleware/cubotmodules \
              -ICubotMiddleware/interaction \
              -ICubotMiddleware/referee \
              -IJoint

OPT         = -O0
WARN        = -Wall -Wno-unused-parameter
CFLAGS      = $(CPU_FLAGS) $(OPT) $(WARN) -fdata-sections -ffunction-sections -std=gnu11
CFLAGS     += $(DEFS) $(INCLUDES)
ASFLAGS     = $(CPU_FLAGS) $(DEFS) $(INCLUDES)
LDFLAGS     = $(CPU_FLAGS) -T$(LDSCRIPT) -Wl,--gc-sections
LDFLAGS    += -specs=nano.specs -specs=nosys.specs
LDFLAGS    += -Wl,-Map=$(BUILD_DIR)/$(TARGET).map
LIBS        = -lc -lm

# --- Sources (same set as MDK-ARM/TEXT_USART.uvprojx) ---
SRCS_C := \
  Core/Src/main.c Core/Src/gpio.c Core/Src/dma.c Core/Src/fdcan.c Core/Src/tim.c \
  Core/Src/usart.c Core/Src/stm32h7xx_it.c Core/Src/stm32h7xx_hal_msp.c \
  Core/Src/system_stm32h7xx.c \
  CubotMiddleware/dr16/dr16.c \
  CubotMiddleware/cubotdevice/motor/motor.c \
  CubotMiddleware/cubotdevice/check/check.c \
  CubotMiddleware/cubotdevice/servo.c \
  CubotMiddleware/cubotdevice/mit.c CubotMiddleware/cubotdevice/vt13.c \
  CubotMiddleware/cubotdevice/filter.c \
  CubotMiddleware/interaction/interaction.c \
  CubotMiddleware/interaction/interaction_image.c \
  CubotMiddleware/interaction/wifi.c CubotMiddleware/interaction/interboard.c \
  CubotMiddleware/interaction/auto.c \
  CubotMiddleware/referee/referee.c \
  CubotMiddleware/drivers/driver_usart.c CubotMiddleware/drivers/driver_can.c \
  CubotMiddleware/drivers/driver_timer.c \
  CubotMiddleware/cubotmodules/modbus.c \
  CubotMiddleware/cubotmodules/mecanum_chassis.c CubotMiddleware/cubotmodules/attitude.c \
  CubotMiddleware/cubotmodules/lifting.c CubotMiddleware/cubotmodules/ore.c \
  CubotMiddleware/Algorithms/pid.c CubotMiddleware/Algorithms/kalman.c \
  CubotMiddleware/Algorithms/Custom_controller.c \
  Joint/AIM_PDO.c Joint/AIM_SDO.c \
  Joint/joint_control.c Joint/velocity_pl.c Joint/controllerl.c \
  User/control_logic.c User/hardware_config.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_cortex.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_fdcan.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc_ex.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_flash.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_flash_ex.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_gpio.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_hsem.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma_ex.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_mdma.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr_ex.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_i2c.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_i2c_ex.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_exti.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_tim.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_tim_ex.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_uart.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_uart_ex.c \
  Drivers/CMSIS/DSP/Source/FastMathFunctions/arm_sin_f32.c \
  Drivers/CMSIS/DSP/Source/CommonTables/arm_common_tables.c

SRCS_S := Drivers/CMSIS/Device/ST/STM32H7xx/Source/Templates/gcc/startup_stm32h750xx.s

OBJS := $(SRCS_C:%.c=$(BUILD_DIR)/%.o) $(SRCS_S:%.s=$(BUILD_DIR)/%.o)

ELF := $(BUILD_DIR)/$(TARGET).elf
HEX := $(BUILD_DIR)/$(TARGET).hex
BIN := $(BUILD_DIR)/$(TARGET).bin

.PHONY: all clean size

all: $(ELF) $(HEX) $(BIN)
	@echo "Output: $(ELF) $(HEX) $(BIN)"

size: $(ELF)
	$(SIZE) $<

$(ELF): $(OBJS) $(LDSCRIPT) Makefile
	$(CC) $(LDFLAGS) -o $@ $(OBJS) $(LIBS)
	$(SIZE) $@

$(HEX): $(ELF)
	$(OBJCOPY) -O ihex $< $@

$(BIN): $(ELF)
	$(OBJCOPY) -O binary -S $< $@

$(BUILD_DIR)/%.o: %.c Makefile
	$(MKDIR_OBJDIR)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile
	$(MKDIR_OBJDIR)
	$(AS) -c $(ASFLAGS) $< -o $@

clean:
	@if exist "$(subst /,\,$(BUILD_DIR))" rmdir /s /q "$(subst /,\,$(BUILD_DIR))"
