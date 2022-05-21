#include "odometry.h"


// speed_t robot_get_speed();
// speed_t robot_get_speed_latest();
// void robot_set_pos(pos_t pos);
// void robot_set_angle(float rad);
pos_t robot_get_pos() {
    pos_t p = {.a = 0, .a_rad = 0, .x = 100, .y = 100};
    return p;
}
