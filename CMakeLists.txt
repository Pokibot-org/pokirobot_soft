cmake_minimum_required(VERSION 3.20.0)

list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
list(APPEND DTS_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
find_package(Zephyr 3.0.0)
project(pokirobot_soft)

target_include_directories(app PRIVATE
    src lib)

file(GLOB_RECURSE APP_SOURCES
    src/*.c lib/*.c)

target_sources(app PRIVATE ${APP_SOURCES})
#target_sources(app PRIVATE src/main.c)

