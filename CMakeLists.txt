# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
# ====================================================================================
#  RP Watch Firmware Build Configuration
#  distribution-ready (No hardcoded paths)
# ====================================================================================
cmake_minimum_required(VERSION 3.13)

# Automatically fetch the SDK from git if not found locally
set(PICO_SDK_FETCH_FROM_GIT ON)

# Initialize the SDK (Loads from PICO_SDK_PATH environment variable)
include(pico_sdk_import.cmake)

# Board Definition
set(PICO_BOARD waveshare_rp2040_touch_lcd_1.28 CACHE STRING "Board type")

project(waveshare_watch_test C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Disable picotool build (prevents common Windows compilation errors)
set(PICO_NO_PICOTOOL ON)

pico_sdk_init()

# ==========================================
#            LVGL CONFIGURATION
# ==========================================
# 1. Look for lv_conf.h in the current source dir
add_compile_definitions(LV_CONF_INCLUDE_SIMPLE)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# 2. Add the library subdirectory (Must exist at ./lib/lvgl)
add_subdirectory(lib/lvgl)

# 3. Link LVGL to Pico SDK
target_link_libraries(lvgl PUBLIC pico_stdlib)

# ==========================================
#            MAIN FIRMWARE TARGET
# ==========================================
add_executable(waveshare_watch_test
    main.c
)

# --- LINKER SETTINGS ---
target_link_libraries(waveshare_watch_test 
    pico_stdlib 
    hardware_spi
    hardware_i2c
    hardware_rtc  
    hardware_adc
    hardware_pwm
    hardware_watchdog
    hardware_clocks
    hardware_timer
    hardware_flash
    hardware_sync
    lvgl          
)

# ==========================================
#            OUTPUT GENERATION
# ==========================================
# This generates .uf2, .hex, and .bin files in the /build directory
pico_add_extra_outputs(waveshare_watch_test)

# Enable USB Serial (for 'S' settings dump), Disable UART
pico_enable_stdio_usb(waveshare_watch_test 1)
pico_enable_stdio_uart(waveshare_watch_test 0)

# Optimization (Size vs Speed)
# -Os is usually best for RP2040 to save flash space, -O3 for max speed
target_compile_options(waveshare_watch_test PRIVATE -Os)