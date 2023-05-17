cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(pokirobot_soft)

target_include_directories(app PRIVATE 
    src 
    ../app/lib
    ../app/src
)

target_sources(app PRIVATE 
    src/main.c 
    src/strat_interface.c 
    ../app/lib/pokibrain/pokibrain.c
    ../app/lib/obstacles/obstacle.c
    ../app/lib/pathfinding/pathfinding.c
    ../app/lib/pokutils/robot_utils.c
    ../app/src/nav/obstacle_manager.c
    ../app/src/nav/path_manager.c
    ../app/src/nav/nav.c
    ../app/src/pokutils.c
    ../app/src/strat/strat.c
    )