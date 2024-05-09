#include "strat/strat_interface.h"
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <math.h>

#include <stdio.h>
#include <sys/socket.h>
#include <sys/un.h>

LOG_MODULE_REGISTER(strat_interface);
#define LOG_POS(pos)                                                                               \
    LOG_DBG("position<x: %3.2f,y: %3.2f,a: %3.2f>", (double)(pos).x, (double)(pos).y,              \
            (double)(pos).a)

#define CONTROL_PERIOD_MS     1.0f
#define CONTROL_SPEED         10000.0f
#define CONTROL_MUTEX_TIMEOUT (K_MSEC(30))

void socket_send(const char *string);

typedef struct control {
    bool start;
    bool at_target;
    LOCKVAR(pos2_t) pos;
    int socket_fd;
    bool is_socket_connected;
    bool running;
    pos2_t waypoints[256];
    int n_waypoints;
    int current_waypoint;
    float dir_angle;
} control_t;
control_t fake_ctrl;

#define CONTROL_LOCKVAR_SETTER(_var, _type)                                                        \
    int control_set_##_var(control_t *dev, _type _var)                                             \
    {                                                                                              \
        int err = 0;                                                                               \
        SET_LOCKVAR(dev->_var, _var, err, CONTROL_MUTEX_TIMEOUT);                                  \
        if (err) {                                                                                 \
            LOG_ERR("could not lock _var mutex for write access");                                 \
            goto exit_error;                                                                       \
        }                                                                                          \
        return 0;                                                                                  \
    exit_error:                                                                                    \
        return -1;                                                                                 \
    }

#define CONTROL_LOCKVAR_GETTER(_var, _type)                                                        \
    int control_get_##_var(control_t *dev, _type *_var)                                            \
    {                                                                                              \
        int err = 0;                                                                               \
        READ_LOCKVAR(dev->_var, *_var, err, CONTROL_MUTEX_TIMEOUT);                                \
        if (err) {                                                                                 \
            LOG_ERR("could not lock _var mutex for write access");                                 \
            goto exit_error;                                                                       \
        }                                                                                          \
        return 0;                                                                                  \
    exit_error:                                                                                    \
        return -1;                                                                                 \
    }

CONTROL_LOCKVAR_SETTER(pos, pos2_t)
CONTROL_LOCKVAR_GETTER(pos, pos2_t)

int strat_set_robot_brake(bool brake)
{
    return -1;
}

int strat_get_robot_pos(pos2_t *pos)
{
    int ret = control_get_pos(&fake_ctrl, pos);
    LOG_POS(*pos);
    return ret;
}

float strat_get_robot_dir_angle(void)
{
    return fake_ctrl.dir_angle;
}

int strat_set_robot_pos(pos2_t pos)
{
    LOG_POS(pos);
    return control_set_pos(&fake_ctrl, pos);
}

int strat_set_waypoints(pos2_t *pos_list, int n)
{
    k_mutex_lock(&fake_ctrl.pos.lock, K_MSEC(100));
    memcpy(fake_ctrl.waypoints, pos_list, n * sizeof(pos2_t));
    fake_ctrl.n_waypoints = n;
    fake_ctrl.current_waypoint = 0;
    k_mutex_unlock(&fake_ctrl.pos.lock);
    return 0;
}

int strat_set_target(pos2_t pos)
{
    return strat_set_waypoints(&pos, 1);
}

int strat_wait_target(float planar_sensivity, float angular_sensivity, uint32_t timeout_ms,
                      uint32_t timeout_break_ms)
{
    fake_ctrl.at_target = false;
    for (int i = 0; i < timeout_ms; i++) {
        if (fake_ctrl.at_target) {
            LOG_DBG("At target");
            return 0;
        }
        k_sleep(K_MSEC(1));
    }
    return -1;
}

void strat_force_motor_stop(void)
{
    fake_ctrl.running = false;
}

int strat_move_robot_to(pos2_t pos, k_timeout_t timeout)
{
    if (!fake_ctrl.start) {
        fake_ctrl.start = true;
    }

    LOG_DBG("pos.a %f", (double)pos.a);
    if (strat_set_target(pos)) {
        return -1;
    }

    if (strat_wait_target_default(k_ticks_to_ms_near64(timeout.ticks), 2000)) {
        return -2;
    }
    return 0;
}

int strat_grab_layer(pos2_t layer_pos, k_timeout_t timeout)
{
    k_sleep(K_SECONDS(2));
    return 0;
}

int strat_put_layer(pos2_t plate_pos, uint8_t current_cake_height, k_timeout_t timeout)
{
    k_sleep(K_SECONDS(1));
    return 0;
}

void fake_control_task(void)
{
    const float speed = CONTROL_SPEED / (CONTROL_PERIOD_MS * 1000.0f);
    const float precision = speed + 0.1f;
    char buffer[256];
    fake_ctrl.running = true;
    while (fake_ctrl.running) {
        pos2_t current_pos;
        k_mutex_lock(&fake_ctrl.pos.lock, K_MSEC(100));
        current_pos = fake_ctrl.pos.val;

        point2_t current_point = {.x = current_pos.x, .y = current_pos.y};
        point2_t target_point = {.x = fake_ctrl.waypoints[fake_ctrl.current_waypoint].x,
                                 .y = fake_ctrl.waypoints[fake_ctrl.current_waypoint].y};
        float target_angle = fake_ctrl.waypoints[fake_ctrl.current_waypoint].a;

        vec2_t diff = point2_diff(target_point, current_point);
        float dist = vec2_distance(target_point, current_point);
        vec2_t norm = vec2_normalize(diff);

        current_point.x += norm.dx * fminf(speed, dist);
        current_point.y += norm.dy * fminf(speed, dist);

        if (fabsf(target_point.x - current_point.x) < precision &&
            fabsf(target_point.y - current_point.y) < precision) {
            if (fake_ctrl.current_waypoint + 1 == fake_ctrl.n_waypoints) {
                fake_ctrl.at_target = true;
            } else {
                fake_ctrl.current_waypoint += 1;
            }
        }
        fake_ctrl.pos.val = (pos2_t){.x = current_point.x, .y = current_point.y, .a = target_angle};
        k_mutex_unlock(&fake_ctrl.pos.lock);

        snprintf(buffer, 256, "{\"pos\": {\"x\":%.2f, \"y\":%.2f, \"a\":%.2f}}",
                 (double)current_pos.x, (double)current_pos.y, (double)current_pos.a);
        socket_send(buffer);
        k_sleep(K_USEC((uint64_t)(CONTROL_PERIOD_MS * 1000)));
    }
}

K_THREAD_DEFINE(fake_control, 1024, fake_control_task, NULL, NULL, NULL, 5, 0, 0);

// SOCKET
void socket_init(void)
{
    struct sockaddr_un addr;

    if ((fake_ctrl.socket_fd = socket(PF_UNIX, SOCK_DGRAM, 0)) < 0) {
        LOG_ERR("Failed to open socket");
        return;
    }

    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    strcpy(addr.sun_path, "/tmp/strat_visu_server.sock");
    if (connect(fake_ctrl.socket_fd, (struct sockaddr *)&addr, sizeof(addr)) == -1) {
        LOG_ERR("Failed to connect to the socket");
        return;
    }

    fake_ctrl.is_socket_connected = true;
}

void socket_send(const char *string)
{
    if (!fake_ctrl.is_socket_connected) {
        return;
    }
    if (send(fake_ctrl.socket_fd, string, strlen(string), 0) == -1) {
        LOG_ERR("Error while sending data on socket");
    }
}

// LIDAR
#include "lidar/camsense_x1/camsense_x1.h"

uint8_t camsense_x1_init(camsense_x1_msg_clbk fun, void *user_data)
{
    return 0;
}
float camsense_x1_get_sensor_speed(void)
{
    return 0;
}
void camsense_x1_kill(void)
{
}

// POKPUSH

int pokpush_deploy(void)
{
    return 0;
}
int pokpush_retract(void)
{
    return 0;
}

// POKSITCK

int pokstick_deploy(void)
{
    LOG_INF("STICK DEPLOYED");
    return 0;
}
int pokstick_retract(void)
{
    LOG_INF("STICK RETRACTED");
    return 0;
}

// POKUICOM

void pokuicom_send_score(uint8_t score)
{
    LOG_INF("SCORE: %d", score);
}

// INIT

int fake_init(void)
{
    INIT_LOCKVAR(fake_ctrl.pos);
    socket_init();
    socket_send("{\"init\": \"ok\"}");
    return 0;
}

SYS_INIT(fake_init, APPLICATION, 1);