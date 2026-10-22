cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# ==================== 修改点 1：添加 C++ 标准 ====================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
# ================================================================

# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME Blink)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# ==================== 修改点 2：启用 CXX 语言 ====================
# 原来的: enable_language(C ASM)
enable_language(C CXX ASM)
# ================================================================

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # ==================== 修改点 3：添加你的 C++ 文件 ====================
    # 假设你把 esp8266.cpp 放在了 Core/Src 目录下，就加上这句：
    Core/Src/esp8266.cpp
    Core/Src/esp8266_port.c        # ← 新增:IDLE+DMA 硬件层
    Core/Src/motion.c
    Core/Src/protocol.c
    Core/Src/app.c
    # ================================================================
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    Core/Inc
)

set(ESP_WIFI_SSID     "ChinaNet-cUup"      CACHE STRING "WiFi SSID")
set(ESP_WIFI_PASSWORD "pmv4awfx"           CACHE STRING "WiFi Password")
set(ESP_TCP_HOST      "192.168.1.15"       CACHE STRING "TCP Server IP")
set(ESP_TCP_PORT      "10000"              CACHE STRING "TCP Server Port")
# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
    ESP_WIFI_SSID="${ESP_WIFI_SSID}"
    ESP_WIFI_PASSWORD="${ESP_WIFI_PASSWORD}"
    ESP_TCP_HOST="${ESP_TCP_HOST}"
    ESP_TCP_PORT=${ESP_TCP_PORT}       # 注意:数字不带引号
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)