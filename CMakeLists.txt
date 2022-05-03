cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr 3.0.0)
project(pokirobot_soft)

target_include_directories(app PRIVATE
    src lib)

file(GLOB_RECURSE APP_SOURCES
    src/*.c lib/*.c)

target_sources(app PRIVATE ${APP_SOURCES})

