cmake_minimum_required(VERSION 3.16)
project(espMCP23017 VERSION 1.0.0 LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Define the library
add_library(espMCP23017 STATIC)

# Set include directories
target_include_directories(espMCP23017 PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/MCP230XX
)


# Collect source files
file(GLOB_RECURSE SOURCES src/MCP230XX/**/*.cpp src/MCP230XX/**/*.c)
target_sources(espMCP23017 PRIVATE ${SOURCES})

# Link against PlatformIO/Arduino framework if needed
if(DEFINED ENV{PLATFORMIO_HOME_DIR})
    message(STATUS "Building with PlatformIO")
    add_definitions(-DPLATFORMIO)
    include_directories($ENV{PLATFORMIO_HOME_DIR}/packages/framework-arduinoespressif32/libraries)
endif()

# Link the Embedded Template Library (ETL) if available
find_package(ETL QUIET)
if(ETL_FOUND)
    target_link_libraries(espMCP23017 PRIVATE ETL)
else()
    message(WARNING "ETL not found, make sure it's available via PlatformIO or manually installed!")
endif()

# Add support for ESP-IDF
if(DEFINED ENV{IDF_PATH})
    message(STATUS "Building with ESP-IDF")
    include($ENV{IDF_PATH}/tools/cmake/project.cmake)
    target_link_libraries(espMCP23017 PRIVATE espidf)
endif()

# Define examples as separate projects
# add_subdirectory(examples)

# # Add example executable
# add_executable(example_main examples/PlatformIO/main.cpp)
# target_link_libraries(example_main PRIVATE espMCP23017)

# Enable testing
# enable_testing()
# add_subdirectory(test)

# Enable LittleFS support
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/partitions.csv")
    message(STATUS "Using partitions.csv for memory layout")
    target_compile_definitions(espMCP23017 PRIVATE -DLITTLEFS)
endif()

# Set additional build flags
# target_compile_options(espMCP23017 PRIVATE -Os -Wall -Wextra -Wno-unused-parameter)
