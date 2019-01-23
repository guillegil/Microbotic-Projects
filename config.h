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

#define PROX_SAMPLES_FREQ           100

#define MAX_FORWARD_SPEED           1.0f            // Max value passed to MotorsQueue for forward movement
#define MAX_BACKWARD_SPEED          -1.0f           // Max value passed to MotorsQueue for backward movement

#define MAX_MOTORS_DUTYCYCLE        0.1f            // Max forward speed duty cycle
#define NEUTRAL_MOTORS_DUTYCYCLE    0.076f          // Max backward speed duty cycle
#define MIN_MOTORS_DUTYCYCLE        0.052f          // Stop duty cycle

#define ADC_PROX_BASE               ADC0_BASE
#define ADC_PROX_GPIO_BASE          GPIO_PORTE_BASE
#define ADC_PROX_GPIO_PIN           GPIO_PIN_3

#define PWM_OUT_RIGHT_MOTOR         PWM_OUT_7       // Right wheel PWM output used in some PWM api functions
#define PWM_OUT_LEFT_MOTOR          PWM_OUT_6       // Left wheel PWM output used in some PWM api functions

#define OPTICAL_SENSORS_GPIO_PERIPH SYSCTL_PERIPH_GPIOB
#define OPTICAL_SENSORS_GPIO_BASE   GPIO_PORTB_BASE
#define RIGHT_ENCODER_PIN           GPIO_PIN_1
#define LEFT_ENCODER_PIN            GPIO_PIN_2


#define FLOOR_SENSOR_GPIO_PERIPH    SYSCTL_PERIPH_GPIOD
#define FLOOR_SENSORS_GPIO_BASE     GPIO_PORTD_BASE
#define RIGHT_FLOOR_SENSOR_PIN      GPIO_PIN_2
#define LEFT_FLOOR_SENSOR_PIN       GPIO_PIN_6
#define ALL_FLOOR_SENSORS           (RIGHT_FLOOR_SENSOR_PIN | LEFT_FLOOR_SENSOR_PIN)



#define FLOOR_SENSORS               (RIGHT_FLOOR_SENSOR_PIN | LEFT_FLOOR_SENSOR_PIN)

#define ALL_OPTICAL_SENSOR_PINS     (RIGHT_ENCODER_PIN | LEFT_ENCODER_PIN)

#define REACTIVE_QUEUE_SIZE         (4)
#define MOTION_QUEUE_SIZE           (3)
#define MOTORS_QUEUE_SIZE           (2)
#define PROXIMITY_QUEUE_SIZE        (1)
#define ARBITER_QUEUE_SIZE          (1)
#define MAPPING_QUEUE_SIZE          (5)

#define ENCODER_STRIPES             (36.0f)
#define WHEEL_DIAMETER              (59.0f)         // Millimeters
#define WHEELS_SEPARATION           (93.0f)         // Millimeters FIXME: MEASURE!!!!
#define FLOOR_SENSOR_SEPARATION     (45.0f)         // Millimeters FIXME: MEASURE!!!!
#define M_PI                        (3.14159f)      // Pi
#define STEP_DISTANCE               (WHEEL_DIAMETER * M_PI / ENCODER_STRIPES)
#define STEP_ANGLE_DEG              (WHEEL_DIAMETER * 360.0f / ENCODER_STRIPES / WHEELS_SEPARATION)
#define STEP_ANGLE_RAD              (WHEEL_DIAMETER * 2.0f * M_PI / ENCODER_STRIPES / WHEELS_SEPARATION)

#define LOWER_PROXIMITY_THRESHOLD 40
#define UPPER_PROXIMITY_THRESHOLD 150
#define OVER_RANGE_THRESHOLD      350

#define TURN_STATE                  (0)
#define MOVE_STATE                  (1)
#define RIGHT_WHEEL_OUT_STATE       (2)
#define LEFT_WHEEL_OUT_STATE        (3)

#define PROXIMITY_CALIBRATION_SAMPLES   10

#endif /* CONFIG_H_ */
