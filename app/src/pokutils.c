#include "pokutils.h"

#include <math.h>
#include <zephyr/kernel.h>

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(utils);

vec2_t point2_diff(const point2_t terminal, const point2_t initial)
{
	vec2_t delta = {
		.dx = terminal.x - initial.x,
		.dy = terminal.y - initial.y,
	};
	return delta;
}

float vec2_abs(vec2_t vec)
{
	return sqrtf(vec.dx * vec.dx + vec.dy * vec.dy);
}

float vec2_dot(vec2_t a, vec2_t b)
{
	return a.dx * b.dx + a.dy * b.dy;
}

float vec2_angle(vec2_t a, vec2_t b)
{
	return acosf(vec2_dot(a, b) / (vec2_abs(a) * vec2_abs(b)));
}

float vec2_fast_abs(vec2_t vec)
{
	// https://www.flipcode.com/archives/Fast_Approximate_Distance_Functions.shtml
	// https://gamedev.stackexchange.com/questions/69241/how-to-optimize-the-distance-function
	return 0.5 * (vec.dx + vec.dy + Z_MAX(vec.dx, vec.dy));
}

pos2_t pos2_diff(const pos2_t terminal, const pos2_t initial)
{
	pos2_t delta = {
		.x = terminal.x - initial.x,
		.y = terminal.y - initial.y,
		.a = terminal.a - initial.a,
	};
	return delta;
}
