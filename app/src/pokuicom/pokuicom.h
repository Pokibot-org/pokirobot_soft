#ifndef POKUICOM_H
#define POKUICOM_H
#include "poktocol/poktocol.h"
#include "stdint.h"

void pokuicom_send_score(uint8_t score);
bool pokuicom_is_match_started(void);
int pokuicom_get_team_color(enum pokprotocol_team *color);
int pokuicom_request(enum poktocol_data_types type);

#endif