#ifndef STRAT_H
#define STRAT_H

enum strat_team_color {
    STRAT_TEAM_COLOR_NONE,
    STRAT_TEAM_COLOR_BLUE,
    STRAT_TEAM_COLOR_YELLOW,
};

void strat_init(enum strat_team_color color);
void strat_run(void);

#endif
