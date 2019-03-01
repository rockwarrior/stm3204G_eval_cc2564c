cmake_minimum_required(VERSION 3.10.3)

project(STM32_EVAL C ASM)

include_directories(
  Application
  CMSIS
  STM32_StdPeriph_Lib
  Bluetopia/btpskrnl
  Bluetopia/btpsvend
  Bluetopia/btvs
  Bluetopia/hcitrans.dma
  Bluetopia/include
  Bluetopia/profiles/A2DP/include
  Bluetopia/profiles/Audio/include
  Bluetopia/profiles/AVRCP/include
  Bluetopia/profiles/AVCTP/include
  Bluetopia/profiles/GAVD/include)

file(GLOB_RECURSE USER_SOURCES
  Application/*.c
  Bluetopia/btpskrnl/*.c
  Bluetopia/btpsvend/*.c
  Bluetopia/btvs/*.c
  Bluetopia/hcitrans/*.c
  CMSIS/*.c
  STM32_StdPeriph_Lib/*.c
  startup_stm32f407xx.s)

add_executable(${PROJECT_NAME}.elf ${USER_SOURCES} ${LINKER_SCRIPT})

target_link_libraries(${PROJECT_NAME}.elf
  ${PROJECT_SOURCE_DIR}/Bluetopia/lib/gcc/libBluetopia_32_M4.fp_HW_FP.a
  ${PROJECT_SOURCE_DIR}/Bluetopia/profiles/audio/lib/gcc/libSS1BTAUD_32_M4.fp_HW_FP.a
  ${PROJECT_SOURCE_DIR}/Bluetopia/profiles/AVCTP/lib/gcc/libSS1BTAVC_32_M4.fp_HW_FP.a
  ${PROJECT_SOURCE_DIR}/Bluetopia/profiles/AVRCP/lib/gcc/libSS1BTAVR_32_M4.fp_HW_FP.a
  ${PROJECT_SOURCE_DIR}/Bluetopia/profiles/GAVD/lib/gcc/libSS1BTGAV_32_M4.fp_HW_FP.a)

set(BIN_FILE ${PROJECT_SOURCE_DIR}/cmake-build-debug/${PROJECT_NAME}.bin)

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_FILE}
        COMMENT "Building ${BIN_FILE}")
