/**
	\file
	\brief
		This is the source file for the PID of the omnidirectional mobile robot.
		It contains all the implementation for all the elements of the Proportional
		Integral Derivative controller.
		i.e., this is the implementation of the PID.
	\author Yadira Vanessa Bonifacio Benavides ie71445@iteso.mx
 */

#ifndef PID_H_
#define PID_H_

#include "stdint.h"
#include "DualVNH5019MotorShield.h"

#define TS 100000
#define N  100
#define KP 0.0000000000135
#define KI 0.000000001
#define KD 0
#define MAX_UK 6
#define MIN_UK -6

/*!These constants are used to save the error values*/
typedef struct {float m1, m2, m3, m4;} pid_e_k_t;
/*!These constants are used to save the integral values*/
typedef struct {float m1, m2, m3, m4;} pid_uI_k_t;
/*!These constants are used to save the derivative values*/
typedef struct {float m1, m2, m3, m4;} pid_uD_k_t;

float PID_proportional(float e_k);
float PID_integral(float e_k_1, float uI_k_1);
float PID_derivative(float e_k, float e_k_1, float uD_k_1);
int16_t PID_controller(motor_name_t motor, float w_ref, float w_acc);

#endif /* PID_H_ */
