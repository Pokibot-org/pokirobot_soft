#ifndef STRAT_H
#define STRAT_H

enum team_color {
	TEAM_COLOR_NONE,
	TEAM_COLOR_BLUE,
	TEAM_COLOR_GREEN,
};

void strat_init(enum team_color color);
void strat_run(void);

#endif