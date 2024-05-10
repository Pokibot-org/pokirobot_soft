#include "stdint.h"
#include "stdbool.h"
#include "poktocol.h"

#define POKMAC_SYNC_SIZE      2
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
    send_buffer[0] = 0xDE;
    send_buffer[1] = 0xAD;

    send_buffer[2] = 1; // WRITE NO ACK

    // APPLICATIF
    uint8_t index = POKMAC_SYNC_SIZE + POKMAC_HEADER_SIZE;
    send_buffer[index++] = msg->cmd;
    send_buffer[index++] = msg->type;

    if( msg->cmd == POKTOCOL_CMD_TYPE_WRITE ) {
        switch (msg->type) {
            case POKTOCOL_DATA_TYPE_SCORE:
                send_buffer[index++] = msg->data.score;
                break;
            case POKTOCOL_DATA_TYPE_TEAM:
                send_buffer[index++] = msg->data.team;
                break;
            case POKTOCOL_DATA_TYPE_TIRETTE_STATUS:
                send_buffer[index++] = msg->data.tirette;
                break;
            default:
                break;
        }
    }

    uint16_t app_payload_size = index - (POKMAC_SYNC_SIZE + POKMAC_HEADER_SIZE);
    send_buffer[3] = app_payload_size >> 8;
    send_buffer[4] = app_payload_size & 0xFFU;

    // Unused crc
    send_buffer[index++] = 0;
    send_buffer[index++] = 0;

    obj->cfg.send(send_buffer, index, obj->cfg.user_data);
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
        case POKTOCOL_DATA_TYPE_TIRETTE_STATUS:
            obj->ans_msg.data.tirette = payload[2];
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
            obj->receiving = false;
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
				obj->receive_index = 0;
				obj->receiving = false;
        }
    }

    return 0;
}
