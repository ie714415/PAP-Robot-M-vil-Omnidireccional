/**
	\file
	\brief
		This is the source file for the Dual VNH5019 Motor Driver Shield.
		It contains all the implementation for all the elements for the configuration and use of the driver.
		i.e., this is the application programming interface (API) for the Dual VNH5019 Motor Driver Shield.
	\author Yadira Vanessa Bonifacio Benavides ie71445@iteso.mx
 */

#ifndef DUALVNH5019MOTORSHIELD_H_
#define DUALVNH5019MOTORSHIELD_H_

#include "stdint.h"
#include "stdbool.h"
#include "Bits.h"

/**Constant that represent the connection pins*/
#define ENABLE_PORT GPIO_E
#define ENABLE_PIN  bit_24
/*Pin configuration motor 1*/
#define M1INA_PORT 	   GPIO_C
#define M1INA_PIN 	   bit_9
#define M1INB_PORT 	   GPIO_C
#define M1INB_PIN 	   bit_8
#define M1PWM_PORT 	   GPIO_C
#define M1PWM_PIN 	   bit_1
/*Pin configuration motor 2*/
#define M2INA_PORT 	   GPIO_A
#define M2INA_PIN	   bit_1
#define M2INB_PORT 	   GPIO_B
#define M2INB_PIN	   bit_9
#define M2PWM_PORT	   GPIO_C
#define M2PWM_PIN	   bit_2
/*Pin configuration motor 3*/
#define M3INA_PORT 	   GPIO_C
#define M3INA_PIN	   bit_5
#define M3INB_PORT	   GPIO_C
#define M3INB_PIN	   bit_12
#define M3PWM_PORT	   GPIO_C
#define M3PWM_PIN	   bit_3
/*Pin configuration motor 4*/
#define M4INA_PORT	   GPIO_D
#define M4INA_PIN	   bit_2
#define M4INB_PORT	   GPIO_D
#define M4INB_PIN	   bit_0
#define M4PWM_PORT	   GPIO_C
#define M4PWM_PIN	   bit_4

/**Constant that represent the maximum duty cycle*/
#define MAX_DUTY_CYCLE 255
/**Constant that represent the half duty cycle*/
#define MIN_DUTY_CYCLE 0

/*! These constants are used to select an specific motor*/
typedef enum{MOTOR_1, MOTOR_2, MOTOR_3, MOTOR_4} motor_name_t;
/*! These constants are used to select an specific direction for the robot*/
typedef enum{FORWARD, BACKWARD, LEFT, RIGHT,
			 TOP_LEFT, TOP_RIGHT, BOTTOM_LEFT, BOTTOM_RIGHT,
			 CLOCK_WISE, COUNTER_CLOCK_WISE, STOP} motor_direction_t;

void initMotorDriverShield(void);
// Set voltage for the motors.
void setM1Voltage(int16_t voltage);
void setM2Voltage(int16_t voltage);
void setM3Voltage(int16_t voltage);
void setM4Voltage(int16_t voltage);
void setVoltages(int16_t m1Voltage, int16_t m2Voltage, int16_t m3Voltage, int16_t m4Voltage);
// Brake for the indicated motors.
void setM1Brake(int16_t brake);
void setM2Brake(int16_t brake);
void setM3Brake(int16_t brake);
void setM4Brake(int16_t brake);
void setBrakes(int16_t m1Brake, int16_t m2Brake, int16_t m3Brake, int16_t m4Brake);
// Set an specific combination of voltages to obtain a direction of the robot
void setMotorDirectionAndVoltage(motor_direction_t direction, int16_t m1Voltage, int16_t m2Voltage, int16_t m3Voltage, int16_t m4Voltage);

#endif /* DUALVNH5019MOTORSHIELD_H_ */
