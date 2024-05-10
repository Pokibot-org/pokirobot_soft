#ifndef GLOBAL_DEF_H
#define GLOBAL_DEF_H

#define ROBOT_RADIUS_INSCRIT_MM (335 / 2.0f)
#define ROBOT_MAX_RADIUS_MM     210
#define ROBOT_MIN_RADIUS_MM     120

#define BOARD_SIZE_X   3000
#define BOARD_MIN_X    -1500
#define BOARD_MAX_X    (BOARD_MIN_X + BOARD_SIZE_X)
#define BOARD_CENTER_X ((float)(BOARD_MIN_X + BOARD_MAX_X) / 2)
#define BOARD_SIZE_Y   2000
#define BOARD_MIN_Y    0
#define BOARD_MAX_Y    (BOARD_MIN_Y + BOARD_SIZE_Y)
#define BOARD_CENTER_Y ((float)(BOARD_MIN_Y + BOARD_MAX_Y) / 2)

#endif
