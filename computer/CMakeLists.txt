# CMakeLists.txt to set up the compilation of the software that drives
# Kybernetes, our navigations platform
cmake_minimum_required (VERSION 2.6)
project (kybernetes)

# Setup project directories
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY lib)

# Use the NEON fpu
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fblocks -march=native -mfpu=neon -std=c11")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fblocks -march=native -mfpu=neon -std=c++14")

# Tell CMAKE to invoke gcc on assembly sources
set_property(SOURCE src/kybernetes/cv/yuv422_bithreshold.s PROPERTY LANGUAGE C)

# Include path
include_directories (${PROJECT_SOURCE_DIR}/include)

# Create the drivers library for Kybernetes, all applications will link
# with this shared library
add_library(kybernetes SHARED src/kybernetes/io/serialdispatchdevice.cpp
                              src/kybernetes/io/serialport.cpp
                              src/kybernetes/sensor/garmingps.cpp
                              src/kybernetes/utility/application.cpp
                              src/kybernetes/utility/utility.cpp
                              src/kybernetes/controller/sensor_controller.cpp
                              src/kybernetes/controller/motion_controller.cpp
                              src/kybernetes/cv/yuv422_bithreshold.s
           )

# Link our library to boost
target_link_libraries (kybernetes dispatch)
target_link_libraries (kybernetes pthread)
target_link_libraries (kybernetes opencv_core)
target_link_libraries (kybernetes opencv_imgproc)
target_link_libraries (kybernetes opencv_objdetect)

# Applications
add_executable(app_test src/test/apptest.cpp)
target_link_libraries(app_test kybernetes)

add_executable(garmingps_test src/test/garmingpstest.cpp)
target_link_libraries(garmingps_test kybernetes)

add_executable(sensor_test src/test/sensortest.cpp)
target_link_libraries(sensor_test kybernetes)
