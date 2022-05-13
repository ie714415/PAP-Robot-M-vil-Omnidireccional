/**
	\file
	\brief
		This is the source file for the Controller of the omnidirectional mobile robot.
		It contains all the implementation for all the elements of the error feedback controller.
		i.e., this is the implementation of the error feedback controller for the robot.
	\author Yadira Vanessa Bonifacio Benavides ie71445@iteso.mx
 */

#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include "stdint.h"
#include "Bits.h"

/**Constant that represent the system parameters*/
#define RW 			  0.045F
#define L  			  0.165F
#define l  			  0.1F
#define KE 			  0.4
#define MAX_PSI 	  360
#define PI_CONTROLLER 3.1416

/*! These constants are used to select an specific reference*/
typedef enum {CIRCLE, SQUARE, SIN} controller_reference_t;
/*!These constants are used to save the coordinates*/
typedef struct {uint8_t x, y;} controller_coordinates_t;
/*!These constants are used to save the system matrixes elements*/
typedef struct {
	float f1c1, f1c2, f1c3, f1c4,
		  f2c1, f2c2, f2c3, f2c4,
		  f3c1, f3c2, f3c3, f3c4;
} controller_matrix3_4_t;
typedef struct {
	float f1c1, f1c2, f1c3,
		  f2c1, f2c2, f2c3,
		  f3c1, f3c2, f3c3,
		  f4c1, f4c2, f4c3;
} controller_matrix4_3_t;
typedef struct {
	float f1c1, f1c2, f1c3, f1c4,
		  f2c1, f2c2, f2c3, f2c4,
		  f3c1, f3c2, f3c3, f3c4,
		  f4c1, f4c2, f4c3, f4c4;
} controller_matrix4_4_t;
/*!These constants are used to save the references and their derivatives*/
typedef struct {
	float xRef, dxRef,
		  yRef, dyRef,
		  psiRef, dpsiRef;
} controller_references_t;
/*!These constants are used to save the states of the controller*/
typedef struct {float x, y, psi;} controller_states_t;
/*!These constants are used to save the control law matrix*/
typedef struct {float w1, w2, w3, w4;} controller_controlLawMatrix_t;

float sin(uint32_t time);
float cos(uint32_t time);
controller_matrix4_3_t transposed(controller_matrix3_4_t matrix);
controller_matrix4_4_t inverse(controller_matrix4_4_t matrix);
controller_references_t Controller_reference(uint32_t time, controller_reference_t ref, controller_coordinates_t c, uint8_t size);
controller_matrix3_4_t Controller_systemMatrix(int16_t psi);
controller_states_t Controller_error(float xRef, float yRef, float psiRef, controller_states_t currentState);
controller_matrix4_3_t Controller_pseudoInverseMatrix(controller_matrix3_4_t B);
controller_controlLawMatrix_t Controller_controlLaw(float dxRef, float dyRef, float dpsiRef, controller_states_t e, controller_matrix4_3_t pinvB);
controller_states_t Controller_odes(controller_matrix3_4_t B, controller_controlLawMatrix_t U, controller_states_t currentState);

#endif /* CONTROLLER_H_ */
