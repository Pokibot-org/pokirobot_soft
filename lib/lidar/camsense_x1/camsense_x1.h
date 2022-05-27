#ifndef CAMSENSE_X1_H
#define CAMSENSE_X1_H
#include <pokutils/lidar_message.h>

#define CAMSENSE_X1_MAX_NUMBER_OF_POINTS 400

typedef void (*camsense_x1_msg_clbk)(
    const lidar_message_t* message, void* user_data); // void * == user data

uint8_t camsense_x1_init(camsense_x1_msg_clbk fun, void* user_data);
float camsense_x1_get_sensor_speed(void);
void camsense_x1_stop();

#endif
