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
#include "driverlib/gpio.h"


#define MOTORS_FREQUENCY            50              // Frequency (Hz) of the servomotors
//#define SERVOM_DUTYCYCLE          0.076           // Duty cycle of the servomotors

#define MAX_FORWARD_SPEED           1.0f            // Max value passed to MotorsQueue for forward movement
#define MAX_BACKWARD_SPEED          -1.0f           // Max value passed to MotorsQueue for backward movement

#define MAX_MOTORS_DUTYCYCLE        0.1f            // Max forward speed duty cycle
#define NEUTRAL_MOTORS_DUTYCYCLE    0.076f          // Max backward speed duty cycle
#define MIN_MOTORS_DUTYCYCLE        0.052f          // Stop duty cycle

#define PWM_OUT_RIGHT_MOTOR         PWM_OUT_6       // Right wheel PWM output used in some PWM api functions
#define PWM_OUT_LEFT_MOTOR          PWM_OUT_7       // Left wheel PWM output used in some PWM api functions

#define OPTICAL_SENSORS_GPIO_PERIPH SYSCTL_PERIPH_GPIOB
#define OPTICAL_SENSORS_GPIO_BASE   GPIO_PORTB_BASE
#define RIGHT_ENCODER_PIN           GPIO_PIN_1
#define LEFT_ENCODER_PIN            GPIO_PIN_2
#define FLOOR_SENSOR_PIN            GPIO_PIN_6
#define ALL_OPTICAL_SENSOR_PINS     (RIGHT_ENCODER_PIN | LEFT_ENCODER_PIN | FLOOR_SENSOR_PIN)

#define ENCODER_STRIPES             (36)
#define WHEEL_DIAMETER              (59.0f)         // Millimeters
#define WHEELS_SEPARATION           (93)            // Millimeters FIXME: MEASURE!!!!

#define M_PI                        (3.14159f)      // Pi

#endif /* CONFIG_H_ */
