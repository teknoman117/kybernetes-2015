# CMakeLists.txt to set up the compilation of the software that drives
# Kybernetes, our navigations platform
cmake_minimum_required (VERSION 2.6)
project (kybernetes)

# Setup project directories
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY lib)

# Use the NEON fpu
#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mfpu=neon -std=c11")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mfpu=neon -std=c++11")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fblocks -std=c11")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fblocks -std=c++11")

# Tell CMAKE to invoke gcc on assembly sources
#set_property(SOURCE src/kybernetes/cv/yuv422_bithreshold.s PROPERTY LANGUAGE C)

# Include path
include_directories (${PROJECT_SOURCE_DIR}/include)

# Create the drivers library for Kybernetes, all applications will link
# with this shared library
add_library(kybernetes SHARED src/kybernetes/io/serialdispatchdevice.cpp
                              src/kybernetes/io/serialport.cpp
                              src/kybernetes/sensor/garmingps.cpp
                              src/kybernetes/utility/application.cpp
                              src/kybernetes/utility/utility.cpp
           )

# Link our library to boost
target_link_libraries (kybernetes dispatch)
target_link_libraries (kybernetes pthread)
