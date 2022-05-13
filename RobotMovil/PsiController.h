/**
	\file
	\brief
		This is the source file for the Controller of the omnidirectional mobile robot.
		It contains all the implementation for all the elements of a controller where we
		are  for only one output.
		i.e., this is the implementation of the angle psi controller for the robot.
	\author Yadira Vanessa Bonifacio Benavides ie71445@iteso.mx
 */

#ifndef PSICONTROLLER_H_
#define PSICONTROLLER_H_

#include "stdint.h"
#include "Bits.h"

/**Constant that represent the system parameters*/
#define RW 0.045F
#define L  0.165F
#define l  0.1F
#define KE -4
#define PI 3.1416

/*!These constants are used to save the system matrixes elements*/
typedef struct {
	float f1c1, f1c2, f1c3, f1c4,
		  f2c1, f2c2, f2c3, f2c4,
		  f3c1, f3c2, f3c3, f3c4;
} psiController_matrix3_4_t;
/*!These constants are used to save the control law matrix*/
typedef struct {float w1, w2, w3, w4;} psiController_controlLawMatrix_t;
/*!These constants are used to save the states of the controller*/
typedef struct {float x, y, psi;} psiController_states_t;

float psiSin(uint16_t time);
float psiCos(uint16_t time);
psiController_matrix3_4_t psiController_systemMatrix(float psi);
psiController_controlLawMatrix_t psiController_controlLaw(float psi, uint16_t time, uint8_t NumberOfSin, uint8_t amplitude);
psiController_states_t psiController_odes(psiController_matrix3_4_t B, psiController_controlLawMatrix_t U);

#endif /* PSICONTROLLER_H_ */
