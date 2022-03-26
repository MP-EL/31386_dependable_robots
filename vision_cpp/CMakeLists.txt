cmake_minimum_required(VERSION 2.8)
project( vision_cpp )
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/obj)
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
add_executable( vision_cpp src/vision.cpp )
target_link_libraries( vision_cpp ${OpenCV_LIBS} )

