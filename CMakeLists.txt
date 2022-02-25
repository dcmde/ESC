cmake_minimum_required(VERSION 3.10)

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

# Prevent error from CLion
SET(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# Set compiler for ARM
set(CMAKE_C_COMPILER "arm-none-eabi-gcc")
set(CMAKE_OBJCOPY "arm-none-eabi-objcopy")

project(ESC C ASM)

# Header files directory
include_directories(
        include
        lib/FreeRTOS-Kernel/portable/GCC/ARM_CM3/
        lib/STM32F10x_StdPeriph_Lib_V3.3.0/Libraries/STM32F10x_StdPeriph_Driver/inc
        lib/STM32F10x_StdPeriph_Lib_V3.3.0/Libraries/CMSIS/CM3/CoreSupport
        lib/STM32F10x_StdPeriph_Lib_V3.3.0/Libraries/CMSIS/CM3/DeviceSupport/ST/STM32F10x
        )

# Source files directory
SET(USER_SOURCES
        src/main.c
        src/stm32f10x_it.c
        )

SET(USER_SOURCES_TF
        src/transfer.c
        src/stm32f10x_it.c
        )

SET(SPL_SOURCES
        lib/STM32F10x_StdPeriph_Lib_V3.3.0/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_rcc.c
        lib/STM32F10x_StdPeriph_Lib_V3.3.0/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_spi.c
        lib/STM32F10x_StdPeriph_Lib_V3.3.0/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_tim.c
        lib/STM32F10x_StdPeriph_Lib_V3.3.0/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_usart.c
        lib/STM32F10x_StdPeriph_Lib_V3.3.0/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_gpio.c
        lib/STM32F10x_StdPeriph_Lib_V3.3.0/Libraries/STM32F10x_StdPeriph_Driver/src/misc.c
        lib/STM32F10x_StdPeriph_Lib_V3.3.0/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_adc.c
        lib/STM32F10x_StdPeriph_Lib_V3.3.0/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_bkp.c
        lib/STM32F10x_StdPeriph_Lib_V3.3.0/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_can.c
        lib/STM32F10x_StdPeriph_Lib_V3.3.0/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_cec.c
        lib/STM32F10x_StdPeriph_Lib_V3.3.0/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_crc.c
        lib/STM32F10x_StdPeriph_Lib_V3.3.0/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_dac.c
        lib/STM32F10x_StdPeriph_Lib_V3.3.0/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_dbgmcu.c
        lib/STM32F10x_StdPeriph_Lib_V3.3.0/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_dma.c
        lib/STM32F10x_StdPeriph_Lib_V3.3.0/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_exti.c
        lib/STM32F10x_StdPeriph_Lib_V3.3.0/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_flash.c
        lib/STM32F10x_StdPeriph_Lib_V3.3.0/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_fsmc.c
        lib/STM32F10x_StdPeriph_Lib_V3.3.0/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_i2c.c
        lib/STM32F10x_StdPeriph_Lib_V3.3.0/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_iwdg.c
        lib/STM32F10x_StdPeriph_Lib_V3.3.0/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_pwr.c
        lib/STM32F10x_StdPeriph_Lib_V3.3.0/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_rtc.c
        lib/STM32F10x_StdPeriph_Lib_V3.3.0/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_sdio.c
        lib/STM32F10x_StdPeriph_Lib_V3.3.0/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_wwdg.c
        )

SET(CMSIS_SOURCES
        lib/STM32F10x_StdPeriph_Lib_V3.3.0/Libraries/CMSIS/CM3/DeviceSupport/ST/STM32F10x/system_stm32f10x.c
        lib/STM32F10x_StdPeriph_Lib_V3.3.0/Libraries/CMSIS/CM3/DeviceSupport/ST/STM32F10x/startup/TrueSTUDIO/startup_stm32f10x_md.s)

# C flags with and without printf
SET(COMMON_FLAGS "-g -O0 -Wall -ffunction-sections -DSTM32F103x6 -DUSE_STDPERIPH_DRIVER -mcpu=cortex-m3 -mthumb -mthumb-interwork -mfpu=vfp -msoft-float -mfix-cortex-m3-ldrd -ffreestanding -lc -flto -specs=nano.specs -specs=nosys.specs")
#SET(COMMON_FLAGS "-g -O1 -Wall -DSTM32F103x6 -DUSE_STDPERIPH_DRIVER -mcpu=cortex-m3 -mthumb -mthumb-interwork -mfpu=vfp -msoft-float -mfix-cortex-m3-ldrd -ffreestanding -lc -flto -specs=nano.specs -specs=nosys.specs")

set(CMAKE_C_FLAGS "${COMMON_FLAGS} -std=gnu99")

# Search for the SPL and create linker flag.
SET(CMAKE_EXE_LINKER_FLAGS "-Wl,--no-wchar-size-warning,--gc-sections,-M=binary.map,-T../lib/STM32F10x_StdPeriph_Lib_V3.3.0/Project/STM32F10x_StdPeriph_Template/TrueSTUDIO/STM3210B-EVAL/stm32_flash.ld")

# Libraries
add_library(SPL ${SPL_SOURCES})
add_library(CMSIS ${CMSIS_SOURCES})

# Executable
add_executable(${PROJECT_NAME}.elf ${USER_SOURCES})
target_link_libraries(${PROJECT_NAME}.elf SPL CMSIS m)

add_executable(transfer_function.elf ${USER_SOURCES_TF})
target_link_libraries(transfer_function.elf SPL CMSIS m)

# Create build directory.
file(MAKE_DIRECTORY ${PROJECT_SOURCE_DIR}/bin/build/)
