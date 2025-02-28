cmake_minimum_required(VERSION 3.16)
project(espMCP23017 VERSION 1.0.0 LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Create the static library target
add_library(espMCP23017 STATIC)

# ----------------------------------------------------------------------------
# Set Include Directories
# ----------------------------------------------------------------------------
# Add the base include directory (where MCP23017.hpp lives)
# and also add subdirectories so that includes like "MCPDevice.hpp" (in COM/) work.
target_include_directories(espMCP23017 PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/
    ${CMAKE_CURRENT_SOURCE_DIR}/src/COM
    ${CMAKE_CURRENT_SOURCE_DIR}/src/CORE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Util
)

# ----------------------------------------------------------------------------
# Collect Source Files
# ----------------------------------------------------------------------------
# This collects all C and C++ source files from your library folder.
file(GLOB_RECURSE LIB_SOURCES 
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp 
    ${CMAKE_CURRENT_SOURCE_DIR}/src/CORE/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/COM/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Util/*.cpp
    

)
target_sources(espMCP23017 PRIVATE ${LIB_SOURCES})

# ----------------------------------------------------------------------------
# Framework-Specific Settings
# ----------------------------------------------------------------------------
# If building with PlatformIO (Arduino framework)
if(DEFINED ENV{PLATFORMIO_HOME_DIR})
    message(STATUS "Building with PlatformIO (Arduino)")
    add_definitions(-DPLATFORMIO)
    # Optionally add Arduino include directories if required
    include_directories($ENV{PLATFORMIO_HOME_DIR}/packages/framework-arduinoespressif32/libraries)
endif()

# If building with ESP-IDF
if(DEFINED ENV{IDF_PATH})
    message(STATUS "Building with ESP-IDF")
    # This will include the ESP-IDF project configuration; note that this
    # might conflict with an idf_component_register() in src/CMakeLists.txt,
    # so remove or rename that file when building with this unified CMake.
    include($ENV{IDF_PATH}/tools/cmake/project.cmake)
    target_link_libraries(espMCP23017 PRIVATE espidf)
else()
    message(STATUS "Building without ESP-IDF (assuming Arduino/PlatformIO)")
endif()

# ----------------------------------------------------------------------------
# External Dependencies
# ----------------------------------------------------------------------------
# Link the Embedded Template Library (ETL) if available
find_package(ETL QUIET)
if(ETL_FOUND)
    target_link_libraries(espMCP23017 PRIVATE ETL)
else()
    message(WARNING "ETL not found! Make sure it's installed (via PlatformIO or manually).")
endif()

# ----------------------------------------------------------------------------
# Optional: LittleFS Support
# ----------------------------------------------------------------------------
# If a partitions.csv file exists, define LITTLEFS support.
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/partitions.csv")
    message(STATUS "Using partitions.csv for memory layout (LITTLEFS enabled)")
    target_compile_definitions(espMCP23017 PRIVATE -DLITTLEFS)
endif()
