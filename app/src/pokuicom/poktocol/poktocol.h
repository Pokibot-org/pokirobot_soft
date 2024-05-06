#ifndef POKTOCOL_H
#define POKTOCOL_H
#include "stddef.h"
#include "stdint.h"
#include "stdbool.h"

#define MAX_DATA_PAYLOAD_SIZE 16

enum poktocol_commands_types {
    POKTOCOL_CMD_TYPE_WRITE,
    POKTOCOL_CMD_TYPE_REQUEST,
};

enum poktocol_data_types {
    POKTOCOL_DATA_TYPE_SCORE,
    POKTOCOL_DATA_TYPE_TEAM,
    POKTOCOL_DATA_TYPE_MATCH_STARTED,
};

enum pokprotocol_team {
    POKTOCOL_TEAM_BLUE,
    POKTOCOL_TEAM_YELLOW,
};

struct poktocol_msg
{
    enum poktocol_commands_types cmd;
    enum poktocol_data_types type;
    union
    {
        uint8_t score;
        enum pokprotocol_team team;
    } data;
};


typedef void (* pokprotocol_receive_clbk)(struct poktocol_msg *msg, void *user_data);
typedef void (* pokprotocol_send_buffer_clbk)(char * buffer, size_t len, void *user_data);

struct poktocol_config
{
    pokprotocol_receive_clbk receive;
    pokprotocol_send_buffer_clbk send;
    void *user_data;
};

struct poktocol {
    uint8_t receive_buffer[MAX_DATA_PAYLOAD_SIZE];
    size_t receive_index;
    size_t packet_completed_index;
    bool receiving;
    struct poktocol_config cfg;
    struct poktocol_msg ans_msg;
};


void pokprotocol_init(struct poktocol *obj, struct poktocol_config *cfg);
int pokprotocol_send(struct poktocol *obj, const struct poktocol_msg *msg);
int pokprotocol_feed_byte(struct poktocol *obj, uint8_t byte);

#endif