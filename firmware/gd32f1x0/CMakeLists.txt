set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)
cmake_minimum_required(VERSION 3.16)

# specify cross compilers and tools
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set(CMAKE_AR arm-none-eabi-ar)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_OBJDUMP arm-none-eabi-objdump)
set(SIZE arm-none-eabi-size)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# project settings
project(Ctrl-Step-GD32-fw C CXX ASM)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)


# add_compile_options(-mfloat-abi=soft)
add_compile_options(-mcpu=cortex-m3 -mthumb -mthumb-interwork )
add_compile_options(-ffunction-sections -fdata-sections -fno-common -fmessage-length=0 -fno-exceptions)
# add_compile_options(-O2)

set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} -fno-rtti )

if ("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
    message(STATUS "Maximum optimization for speed")
    add_compile_options(-Ofast)
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "RelWithDebInfo")
    message(STATUS "Maximum optimization for speed, debug info included")
    add_compile_options(-Ofast -g)
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "MinSizeRel")
    message(STATUS "Maximum optimization for size")
    add_compile_options(-Os)
else ()
    message(STATUS "Minimal optimization, debug info included")
    add_compile_options(-Og -g)
endif ()

set(FIRMWARE_DIR ${CMAKE_SOURCE_DIR}/..)

include_directories(
        system  
        algorithm 
        GD32F1x0_Firmware_Library_V3.4.0/GD32F1x0_standard_peripheral/Include
        GD32F1x0_Firmware_Library_V3.4.0/CMSIS/GD/GD32F1x0/Include
        GD32F1x0_Firmware_Library_V3.4.0/CMSIS
        Core
        ${FIRMWARE_DIR}/Ctrl
        ${FIRMWARE_DIR}/Ctrl/Sensor
        ${FIRMWARE_DIR}/Ctrl/Signal
        ${FIRMWARE_DIR}/Ctrl/LowLevel
        ${FIRMWARE_DIR}/Ctrl/MotorControl
        ${FIRMWARE_DIR}/UserApp
        Port
        )

add_definitions(-DUSE_STDPERIPH_DRIVER -DGD32F130_150)

file(GLOB_RECURSE SOURCES 
"GD32F1x0_Firmware_Library_V3.4.0/GD32F1x0_standard_peripheral/Source/*.c"
"system/*.c" 
"system/startup_gd32f130.s" 
"${FIRMWARE_DIR}/Ctrl/*.*"
"Core/*.c"
"${FIRMWARE_DIR}/UserApp/*.*"
"Port/*.*"
)

set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/system/gd32f130g8u6.ld)

add_link_options(-Wl,-gc-sections,--print-memory-usage,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map)
add_link_options(-mcpu=cortex-m3 -mthumb -mthumb-interwork)
add_link_options(-T ${LINKER_SCRIPT})
add_executable(${PROJECT_NAME}.elf ${SOURCES} ${LINKER_SCRIPT})
set(HEX_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin)
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${HEX_FILE}
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_FILE}
        COMMENT "Building ${HEX_FILE}
Building ${BIN_FILE}")

add_custom_target(flash 
COMMAND "D:/MySoftware/JLink_V794/JLink.exe" -device STM32F103CB  -speed 2000 -if SWD -CommanderScript ${CMAKE_CURRENT_LIST_DIR}/jlink-flash-fw-standalone.jlink
DEPENDS ${PROJECT_NAME}.elf
)