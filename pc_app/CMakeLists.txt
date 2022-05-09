cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr)
project(PokiRobotSoft)

target_include_directories(app PRIVATE 
src 
../lib/pokibrain/
../lib/robot_utils/
)

target_sources(app PRIVATE src/main.c ../lib/pokibrain/pokibrain.c)