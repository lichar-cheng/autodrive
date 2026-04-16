cmake_minimum_required(VERSION 3.16)
project(slam_export_tool LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ZLIB REQUIRED)
find_path(MINIZIP_INCLUDE_DIR minizip/unzip.h REQUIRED)
find_library(MINIZIP_LIBRARY minizip REQUIRED)
find_path(NLOHMANN_INCLUDE_DIR nlohmann/json.hpp REQUIRED)

add_library(slam_export_tool
    src/slam_export_tool.cpp
)

target_include_directories(slam_export_tool
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${NLOHMANN_INCLUDE_DIR}
        ${MINIZIP_INCLUDE_DIR}
)

target_link_libraries(slam_export_tool
    PUBLIC
        ${MINIZIP_LIBRARY}
        ZLIB::ZLIB
)

enable_testing()

add_executable(test_slam_export_tool
    tests/test_slam_export_tool.cpp
)

target_link_libraries(test_slam_export_tool PRIVATE slam_export_tool)

add_test(NAME test_slam_export_tool COMMAND test_slam_export_tool)
