cmake_minimum_required(VERSION 3.10)
project(goac)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS} include)

add_executable(goac src/main.cpp)
target_link_libraries(goac ${OpenCV_LIBS})