# CMakeLists.txt to set up the compilation of the software that drives
# Kybernetes, our navigations platform
cmake_minimum_required (VERSION 2.6)
project (kybernetes)

# Setup project directories
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY lib)

# Use the NEON fpu
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g -fblocks -march=native -mfpu=neon -std=c11")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -fblocks -march=native -mfpu=neon -std=c++14")

# Tell CMAKE to invoke gcc on assembly sources
set_property(SOURCE src/kybernetes/cv/yuv422_bithreshold.s PROPERTY LANGUAGE C)
#set_property(SOURCE src/kybernetes/utility/posixsignalhandler.mm PROPERTY COMPILE_FLAGS -objc)

# Include path
include_directories (${PROJECT_SOURCE_DIR}/include)

# Create the drivers library for Kybernetes, all applications will link
# with this shared library
add_library(kybernetes SHARED src/kybernetes/io/serialdispatchdevice.cpp
                              src/kybernetes/io/serialport.cpp
                              src/kybernetes/sensor/garmingps.cpp
                              src/kybernetes/sensor/uvccamera.cpp
                              src/kybernetes/sensor/v4l2camera.cpp
                              src/kybernetes/utility/application.cpp
                              src/kybernetes/utility/utility.cpp
                              src/kybernetes/utility/pollhandler.cpp
                              src/kybernetes/network/serversocket.cpp
                              src/kybernetes/network/socket.cpp
                              src/kybernetes/controller/sensor_controller.cpp
                              src/kybernetes/controller/motion_controller.cpp
                              src/kybernetes/cv/yuv422_bithreshold.s
           )

# Link our library to boost
target_link_libraries (kybernetes dispatch)
target_link_libraries (kybernetes pthread)
target_link_libraries (kybernetes v4l2)
target_link_libraries (kybernetes opencv_core)
target_link_libraries (kybernetes opencv_imgproc)
target_link_libraries (kybernetes opencv_objdetect)

# Applications
add_executable(robomagellan src/robomagellan/main.cpp src/robomagellan/movetotask.cpp src/robomagellan/findconetask.cpp)
target_link_libraries(robomagellan kybernetes)

add_executable(sensor_diagnostics src/sensor_diagnostics/main.cpp)
target_link_libraries(sensor_diagnostics kybernetes)

add_executable(collision_test src/test/collisiontest.cpp)
target_link_libraries(collision_test kybernetes)

add_executable(vision_test src/test/visiontest.cpp)
target_link_libraries(vision_test kybernetes)
target_link_libraries(vision_test jpeg)

add_executable(dump_test src/test/imgdumptest.cpp src/test/yuv.c)
target_link_libraries(dump_test kybernetes)
target_link_libraries(dump_test jpeg)
target_link_libraries(dump_test v4l2)

add_executable(drive_test src/test/drivetest.cpp)
target_link_libraries(drive_test kybernetes)
