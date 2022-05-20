#ifndef POKARM_H
#define POKARM_H

/**
 * @brief pokarm
 * Module contolling the robot arm of the pokibot
 * AXES:
 * - Z : stepper
 * - rotation on Z axis : servo
 * - rotation on Y axis : servo
 */

void pokarm_init(const struct device* uart);

#endif