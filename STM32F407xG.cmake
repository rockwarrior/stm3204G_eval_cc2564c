set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)

set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_ASM_COMPILER arm-none-eabi-as)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)

set(LINKER_SCRIPT ${PROJECT_SOURCE_DIR}/STM32F407xG.ld)

set(COMMON_FLAGS "-mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16")
set(USER_FLAGS "-DSTM32F40_41xxx -DDEBUG_ENABLED -DHSE_VALUE=25000000 -D__SUPPORT_CC256XC_PATCH__ -DA3DP_SRC_PLAY_SIN -DARM_MATH_CM4 -DUSE_HAL_DRIVER")

set(CMAKE_C_FLAGS "${COMMON_FLAGS} ${USER_FLAGS} -std=gnu99 -O3 -g3 -fmessage-length=0 -ffunction-sections")
set(CMAKE_ASM_FLAGS "${COMMON_FLAGS} -g")
set(CMAKE_EXE_LINKER_FLAGS "${COMMON_FLAGS} -L -specs=nano.specs -specs=nosys.specs -T ${LINKER_SCRIPT} -Wl,-Map=output.map -Wl,-print-memory-usage,--gc-sections -lm -u _printf_float")
