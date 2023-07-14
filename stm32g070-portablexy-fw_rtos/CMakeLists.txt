cmake_minimum_required(VERSION 3.12)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/tools")
include(arm-none-eabi-gcc)

set(PRJ_NAME stm32g070-portablexy-fw_rtos)

project(${PRJ_NAME}
  LANGUAGES ASM C CXX
)
set(EXECUTABLE ${PRJ_NAME}.elf)
set(LINKER_FILE ${CMAKE_SOURCE_DIR}/src/bsp/ldscript/STM32G070RBTX_FLASH.ld)


file(GLOB SRC_FILES CONFIGURE_DEPENDS
  src/hw/*.c
  src/hw/driver/*.c
  src/exhw/*.c
  src/exhw/driver/*.c
)

file(GLOB_RECURSE SRC_FILES_RECURSE CONFIGURE_DEPENDS
  src/*.cpp
  src/ap/*.cpp
  src/ap/*.c
  src/bsp/*.c
  src/common/*.c    
  lib/hal/Src/*.c
  #lib/rtos/*.c
)

add_executable(${EXECUTABLE} 
  src/main.cpp
  src/bsp/startup/startup_stm32g070rbtx.s

  ${SRC_FILES}
  ${SRC_FILES_RECURSE}
)

target_compile_definitions(${EXECUTABLE} PRIVATE
  -DSTM32G070xx
  #-D_USE_HW_RTOS
  -DUSER_VECT_TAB_ADDRESS
  -DDEBUG
  )

target_include_directories(${EXECUTABLE} PRIVATE 
  src  
  src/ap
  src/ap/_inc
  src/bsp
  src/common 
  src/common/core
  src/common/inc_hw
  src/common/inc_exhw
  src/exhw
  src/hw
  lib
  
  lib/cmsis/Device/ST/STM32G0xx/Include
  lib/cmsis/Include
  lib/hal/Inc

  #lib/rtos
  #lib/rtos/Source/CMSIS_RTOS
  #lib/rtos/Source/portable/GCC/ARM_CM0
  #lib/rtos/Source/include
)



target_compile_options(${EXECUTABLE} PRIVATE
  -mcpu=cortex-m0plus
  -mthumb

  #-mfpu=fpv4-sp-d16
  -mfloat-abi=soft

  -fdata-sections
  -ffunction-sections

  -Wall
  -Os
  -g3
  )

target_link_options(${EXECUTABLE} PRIVATE
-T${LINKER_FILE}
-mcpu=cortex-m0plus
-mthumb
#-mfpu=fpv4-sp-d16
-mfloat-abi=soft
-specs=nano.specs
-lc
-lm
-Wl,-Map=${PROJECT_NAME}.map,--cref
-Wl,--gc-sections
-Xlinker -print-memory-usage -Xlinker
)


add_custom_command(TARGET ${EXECUTABLE} 
    POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} ARGS -O binary ${EXECUTABLE} ${PROJECT_NAME}.bin
    COMMENT "Invoking: Make Binary"
    )

#set_target_properties(${EXECUTABLE} PROPERTIES SUFFIX ".elf")
#add_custom_command(TARGET ${EXECUTABLE} POST_BUILD
#        COMMAND arm-none-eabi-size --format=berkeley --totals "$<TARGET_FILE:${EXECUTABLE}>"
#        COMMENT "Invoking: Cross ARM GNU Print Size")