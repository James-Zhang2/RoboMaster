cmake_minimum_required(VERSION 3.10)
project(RoboMaster)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找 OpenCV 包
find_package(OpenCV REQUIRED)

# 显示路径（用于调试）
message(STATUS "OpenCV version: ${OpenCV_VERSION}")
message(STATUS "OpenCV include path: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCV libraries: ${OpenCV_LIBS}")

# 添加头文件目录
include_directories(${OpenCV_INCLUDE_DIRS}, ${CMAKE_CURRENT_SOURCE_DIR})

# 添加源文件
add_executable(main main.cpp)

# 链接 OpenCV 到可执行文件
target_link_libraries(main ${OpenCV_LIBS})