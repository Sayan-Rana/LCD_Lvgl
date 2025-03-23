# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(LCD_Lvgl C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# LCD & Touch related bsp package include
set(BSP_PATH ${CMAKE_CURRENT_LIST_DIR}/bsp)

# Add executable. Default name is the project name, version 0.1

add_executable(LCD_Lvgl 
        src/main.c
        ${BSP_PATH}/lcd/bsp_lcd.c 
)

# Specify path to own LVGL config header
set(LV_CONF_PATH ${CMAKE_CURRENT_SOURCE_DIR}/inc/lv_conf.h
        CACHE STRING "" FORCE
)

# Include LVGL library
add_subdirectory(lib/lvgl)
target_include_directories(LCD_Lvgl PRIVATE 
        ${CMAKE_CURRENT_LIST_DIR}/lib/lvgl
)

pico_set_program_name(LCD_Lvgl "LCD_Lvgl")
pico_set_program_version(LCD_Lvgl "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(LCD_Lvgl 0)
pico_enable_stdio_usb(LCD_Lvgl 0)

# Add the standard library to the build
target_link_libraries(LCD_Lvgl
        pico_stdlib
        hardware_spi
        lvgl
)

# Add the standard include files to the build
target_include_directories(LCD_Lvgl PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/inc
        ${BSP_PATH}
        # ${BSP_PATH}/lcd
        # ${BSP_PATH}/lvgl
        # ${BSP_PATH}/tsc
)

pico_add_extra_outputs(LCD_Lvgl)

