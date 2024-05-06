#include "stdint.h"
#include "stdbool.h"
#include "poktocol.h"

#define POKMAC_HEADER_SIZE    3
#define POKMAC_CRC_SIZE       2

void pokprotocol_init(struct poktocol *obj, struct poktocol_config *cfg)
{
    obj->cfg = *cfg;
    obj->receive_index = 0;
    obj->receiving = false;
}

int pokprotocol_send(struct poktocol *obj, const struct poktocol_msg *msg)
{
    uint8_t send_buffer[MAX_DATA_PAYLOAD_SIZE];
    uint8_t data_len = 0;
    send_buffer[data_len++] = msg->cmd;
    send_buffer[data_len++] = msg->type;
    switch (msg->type) {
        case POKTOCOL_DATA_TYPE_SCORE:
            send_buffer[data_len++] = msg->data.score;
            break;
        case POKTOCOL_DATA_TYPE_TEAM:
            send_buffer[data_len++] = msg->data.team;
            break;
        case POKTOCOL_DATA_TYPE_MATCH_STARTED:
            break;
        default:
            break;
    }

    // Unused crc
    send_buffer[data_len++] = 0;
    send_buffer[data_len++] = 0;

    obj->cfg.send(send_buffer, data_len, obj->cfg.user_data);
    return 0;
}

void pokprotocol_decode_app(struct poktocol *obj, uint8_t *payload, size_t size)
{
    if (size < 2) {
        return;
    }

    obj->ans_msg.cmd = payload[0];
    obj->ans_msg.type = payload[1];
    switch (obj->ans_msg.type) {
        case POKTOCOL_DATA_TYPE_SCORE:
            obj->ans_msg.data.score = payload[2];
            break;
        case POKTOCOL_DATA_TYPE_TEAM:
            obj->ans_msg.data.team = payload[2];
            break;
        case POKTOCOL_DATA_TYPE_MATCH_STARTED:
            break;
        default:
            break;
    }

    obj->cfg.receive(&obj->ans_msg, obj->cfg.user_data);
}

void pokprotocol_decode_mac(struct poktocol *obj)
{
    // Dont care for now no ack is implemented
    // uint8_t cmd = receive_buffer[0];
    uint16_t payload_size = (uint16_t)obj->receive_buffer[1] << 8 | obj->receive_buffer[2];
    uint8_t *payload = &obj->receive_buffer[3];
    pokprotocol_decode_app(obj, payload, payload_size);
    // Dont care for now will implement this if necessary
    // uint16_t crc = &obj->receive_buffer[POKMAC_HEADER_SIZE + payload_size];
}

int pokprotocol_feed_byte(struct poktocol *obj, uint8_t byte)
{
    if (!obj->receiving) {
        // SYNCHING on the 0xDEAD word
        if (byte == 0xDE && obj->receive_index == 0) {
            obj->receive_index++;
        } else if (byte == 0xAD && obj->receive_index == 1) {
            obj->receive_index = 0;
            obj->receiving = true;
        }
    } else {
        if (obj->receive_index >= MAX_DATA_PAYLOAD_SIZE) {
            obj->receive_index = 0;
            return -1;
        }
        obj->receive_buffer[obj->receive_index] = byte;
        obj->receive_index++;

        if (obj->receive_index == POKMAC_HEADER_SIZE) {
            uint16_t payload_size = (uint16_t)obj->receive_buffer[1] << 8 | obj->receive_buffer[2];
            obj->packet_completed_index = POKMAC_HEADER_SIZE + payload_size + POKMAC_CRC_SIZE;
        }
        if (obj->receive_index >= POKMAC_HEADER_SIZE && obj->packet_completed_index == obj->receive_index) {
            pokprotocol_decode_mac(obj);
        }
    }

    return 0;
}
