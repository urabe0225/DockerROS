cmake_minimum_required(VERSION 2.8.3)
project(posetest)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

get_filename_component(SOURCE_NAME posetest.cpp NAME_WE)
set(EXE_NAME "${SOURCE_NAME}.bin")

find_package(OpenCV REQUIRED)
find_package(GFlags)

include_directories(
  ${GFLAGS_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}

  /usr/local/cuda/include
  /openpose/include
  /openpose/3rdparty/caffe/include
  /openpose/build/caffe/src
)

add_executable(${EXE_NAME} posetest.cpp)

target_link_libraries(
  ${EXE_NAME}
  ${OpenCV_LIBS}
  /usr/lib/x86_64-linux-gnu/libglog.so
  /usr/lib/x86_64-linux-gnu/libgflags.so
  /usr/lib/x86_64-linux-gnu/libgflags_nothreads.so
  /openpose/build/src/openpose/libopenpose.so
  /openpose/build/caffe/lib/libcaffe.so
  )