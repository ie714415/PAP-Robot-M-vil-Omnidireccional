/**
	\file
	\brief
		This is the source file for the PID of the omnidirectional mobile robot.
		It contains all the implementation for all the elements of the Proportional
		Integral Derivative controller.
		i.e., this is the implementation of the PID.
	\author Yadira Vanessa Bonifacio Benavides ie71445@iteso.mx
 */

#include "PID.h"

static pid_e_k_t g_e_k_1 = {0, 0, 0, 0};
static pid_uI_k_t g_uI_k_1 = {0, 0, 0, 0};
static pid_uI_k_t g_uD_k_1 = {0, 0, 0, 0};

float PID_proportional(float e_k)
{
	float uP_k;
	uP_k = KP*e_k;
	return uP_k;
}

float PID_integral(float e_k_1, float uI_k_1)
{
	float uI_k;
	uI_k = (KI*TS*e_k_1) + uI_k_1;
	return uI_k;
}

float PID_derivative(float e_k, float e_k_1, float uD_k_1)
{
	float uD_k;
	uD_k = KD*N*(e_k - e_k_1) + uD_k_1*(1 - (N*TS));
	return uD_k;
}

int16_t PID_controller(motor_name_t motor, float w_ref, float w_acc)
{
	float e_k, uP_k, uI_k, uD_k, u_k;
	int16_t voltage;

	e_k  = w_ref - w_acc;
	uP_k = PID_proportional(e_k);
	switch(motor)
	{
	case MOTOR_1:
	{
		uI_k = PID_integral(g_e_k_1.m1, g_uI_k_1.m1);
		uD_k = PID_derivative(e_k, g_e_k_1.m1, g_uD_k_1.m1);
		g_e_k_1.m1 = e_k;
		g_uI_k_1.m1 = uI_k;
		g_uD_k_1.m1 = uD_k;
		break;
	}
	case MOTOR_2:
	{
		uI_k = PID_integral(g_e_k_1.m2, g_uI_k_1.m2);
		uD_k = PID_derivative(e_k, g_e_k_1.m2, g_uD_k_1.m2);
		g_e_k_1.m2 = e_k;
		g_uI_k_1.m2 = uI_k;
		g_uD_k_1.m2 = uD_k;
		break;
	}
	case MOTOR_3:
	{
		uI_k = PID_integral(g_e_k_1.m3, g_uI_k_1.m3);
		uD_k = PID_derivative(e_k, g_e_k_1.m3, g_uD_k_1.m3);
		g_e_k_1.m3 = e_k;
		g_uI_k_1.m3 = uI_k;
		g_uD_k_1.m3 = uD_k;
		break;
	}
	case MOTOR_4:
	{
		uI_k = PID_integral(g_e_k_1.m4, g_uI_k_1.m4);
		uD_k = PID_derivative(e_k, g_e_k_1.m4, g_uD_k_1.m4);
		g_e_k_1.m4 = e_k;
		g_uI_k_1.m4 = uI_k;
		g_uD_k_1.m4 = uD_k;
		break;
	}
	default:
		break;
	}
	u_k  = uP_k + uI_k + uD_k;

	if(MAX_UK < u_k)
		voltage = MAX_DUTY_CYCLE;
	else if(MIN_UK > u_k)
		voltage = -MAX_DUTY_CYCLE;
	else
		voltage = (u_k*MAX_DUTY_CYCLE)/MAX_UK;

	return voltage;
}
