cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME ClioneMotot-42)

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

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
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)
add_subdirectory(CppInterface)
add_subdirectory(Lwbtn)
add_subdirectory(Communication)
add_subdirectory(Shell)
# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    CPP_INTERFACE
    Communication
    lwbtn
    Shell
    # Add user defined libraries
)



target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
        -Wl,--gc-sections               # 核心：删除未被调用的函数（垃圾回收）
        --specs=nosys.specs             # 告诉编译器没有操作系统（裸机/RTOS）
        -Wl,--print-memory-usage        # 编译完自动打印 Flash/RAM 占用详情
)

target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE
        -ffunction-sections
        -fdata-sections
        $<$<CONFIG:Debug>:-Og>          # Debug 模式用 -Og (兼顾调试和体积)
        $<$<CONFIG:Release>:-Os>        # Release 模式用 -Os (极致体积优化)
)