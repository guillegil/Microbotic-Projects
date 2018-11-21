/*
 * config.h
 *
 *  Created on: Oct 25, 2018
 *      Author: Pedro Rico Pinazo
 *
 */

#ifndef CONFIG_H_
#define CONFIG_H_

#include "driverlib/pwm.h"

#define MOTORS_FREQUENCY            50              // Frequency (Hz) of the servomotors
//#define SERVOM_DUTYCYCLE          0.076           // Duty cycle of the servomotors

#define MAX_FORWARD_SPEED           1.0f            // Max value passed to MotorsQueue for forward movement
#define MAX_BACKWARD_SPEED          -1.0f           // Max value passed to MotorsQueue for backward movement

#define MAX_MOTORS_DUTYCYCLE        0.1f            // Max forward speed duty cycle
#define NEUTRAL_MOTORS_DUTYCYCLE    0.076f          // Max backward speed duty cycle
#define MIN_MOTORS_DUTYCYCLE        0.052f          // Stop duty cycle

#define PWM_OUT_RIGHT_MOTOR         PWM_OUT_6       // Right wheel PWM output used in some PWM api functions
#define PWM_OUT_LEFT_MOTOR          PWM_OUT_7       // Left wheel PWM output used in some PWM api functions

#endif /* CONFIG_H_ */
