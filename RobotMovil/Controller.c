/**
	\file
	\brief
		This is the source file for the Controller of the omnidirectional mobile robot.
		It contains all the implementation for all the elements of the error feedback controller.
		i.e., this is the implementation of the error feedback controller for the robot.
	\author Yadira Vanessa Bonifacio Benavides ie71445@iteso.mx
 */

#include "Controller.h"
#include "Encoder.h"

const float sinVector[] = {0.0000, 0.0175, 0.0349, 0.0523, 0.0698, 0.0872, 0.1045, 0.1219, 0.1392, 0.1564, 0.1736, 0.1908,
						  0.2079, 0.2250, 0.2419, 0.2588, 0.2756, 0.2924, 0.3090, 0.3256, 0.3420, 0.3584, 0.3746, 0.3907,
						  0.4067, 0.4226, 0.4384, 0.4540, 0.4695, 0.4848, 0.5000, 0.5150, 0.5299, 0.5446, 0.5592, 0.5736,
						  0.5878, 0.6018, 0.6157, 0.6293, 0.6428, 0.6561, 0.6691, 0.6820, 0.6947, 0.7071, 0.7193, 0.7314,
						  0.7431, 0.7547, 0.7660, 0.7771, 0.7880, 0.7986, 0.8090, 0.8192, 0.8290, 0.8387, 0.8480, 0.8572,
						  0.8660, 0.8746, 0.8829, 0.8910, 0.8988, 0.9063, 0.9135, 0.9205, 0.9272, 0.9336, 0.9397, 0.9455,
						  0.9511, 0.9563, 0.9613, 0.9659, 0.9703, 0.9744, 0.9781, 0.9816, 0.9848, 0.9877, 0.9903, 0.9925,
						  0.9945, 0.9962, 0.9976, 0.9986, 0.9994, 0.9998, 1.0000, 0.9998, 0.9994, 0.9986, 0.9976, 0.9962,
						  0.9945, 0.9925, 0.9903, 0.9877, 0.9848, 0.9816, 0.9781, 0.9744, 0.9703, 0.9659, 0.9613, 0.9563,
						  0.9511, 0.9455, 0.9397, 0.9336, 0.9272, 0.9205, 0.9135, 0.9063, 0.8988, 0.8910, 0.8829, 0.8746,
						  0.8660, 0.8572, 0.8480, 0.8387, 0.8290, 0.8192, 0.8090, 0.7986, 0.7880, 0.7771, 0.7660, 0.7547,
						  0.7431, 0.7314, 0.7193, 0.7071, 0.6947, 0.6820, 0.6691, 0.6561, 0.6428, 0.6293, 0.6157, 0.6018,
						  0.5878, 0.5736, 0.5592, 0.5446, 0.5299, 0.5150, 0.5000, 0.4848, 0.4695, 0.4540, 0.4384, 0.4226,
						  0.4067, 0.3907, 0.3746, 0.3584, 0.3420, 0.3256, 0.3090, 0.2924, 0.2756, 0.2588, 0.2419, 0.2250,
						  0.2079, 0.1908, 0.1736, 0.1564, 0.1392, 0.1219, 0.1045, 0.0872, 0.0698, 0.0523, 0.0349, 0.0175,
						  0.0000, -0.0175, -0.0349, -0.0523, -0.0698, -0.0872, -0.1045, -0.1219, -0.1392, -0.1564, -0.1736, -0.1908,
						 -0.2079, -0.2250, -0.2419, -0.2588, -0.2756, -0.2924, -0.3090, -0.3256, -0.3420, -0.3584, -0.3746, -0.3907,
						 -0.4067, -0.4226, -0.4384, -0.4540, -0.4695, -0.4848, -0.5000, -0.5150, -0.5299, -0.5446, -0.5592, -0.5736,
						 -0.5878, -0.6018, -0.6157, -0.6293, -0.6428, -0.6561, -0.6691, -0.6820, -0.6947, -0.7071, -0.7193, -0.7314,
						 -0.7431, -0.7547, -0.7660, -0.7771, -0.7880, -0.7986, -0.8090, -0.8192, -0.8290, -0.8387, -0.8480, -0.8572,
						 -0.8660, -0.8746, -0.8829, -0.8910, -0.8988, -0.9063, -0.9135, -0.9205, -0.9272, -0.9336, -0.9397, -0.9455,
						 -0.9511, -0.9563, -0.9613, -0.9659, -0.9703, -0.9744, -0.9781, -0.9816, -0.9848, -0.9877, -0.9903, -0.9925,
						 -0.9945, -0.9962, -0.9976, -0.9986, -0.9994, -0.9998, -1.0000, -0.9998, -0.9994, -0.9986, -0.9976, -0.9962,
						 -0.9945, -0.9925, -0.9903, -0.9877, -0.9848, -0.9816, -0.9781, -0.9744, -0.9703, -0.9659, -0.9613, -0.9563,
						 -0.9511, -0.9455, -0.9397, -0.9336, -0.9272, -0.9205, -0.9135, -0.9063, -0.8988, -0.8910, -0.8829, -0.8746,
						 -0.8660, -0.8572, -0.8480, -0.8387, -0.8290, -0.8192, -0.8090, -0.7986, -0.7880, -0.7771, -0.7660, -0.7547,
						 -0.7431, -0.7314, -0.7193, -0.7071, -0.6947, -0.6820, -0.6691, -0.6561, -0.6428, -0.6293, -0.6157, -0.6018,
						 -0.5878, -0.5736, -0.5592, -0.5446, -0.5299, -0.5150, -0.5000, -0.4848, -0.4695, -0.4540, -0.4384, -0.4226,
						 -0.4067, -0.3907, -0.3746, -0.3584, -0.3420, -0.3256, -0.3090, -0.2924, -0.2756, -0.2588, -0.2419, -0.2250,
						 -0.2079, -0.1908, -0.1736, -0.1564, -0.1392, -0.1219, -0.1045, -0.0872, -0.0698, -0.0523, -0.0349, -0.0175};

float sin(uint32_t time)
{
	return sinVector[time];
}

float cos(uint32_t time)
{
	uint8_t angle;
	angle = time + 90;
	if(360 < angle)
	{

	}
	return sinVector[time + 90];
}

controller_matrix4_3_t transposed(controller_matrix3_4_t matrix)
{
	controller_matrix4_3_t T;

	T.f1c1 = matrix.f1c1;
	T.f1c2 = matrix.f2c1;
	T.f1c3 = matrix.f3c1;
	T.f2c1 = matrix.f1c2;
	T.f2c2 = matrix.f2c2;
	T.f2c3 = matrix.f3c2;
	T.f3c1 = matrix.f1c3;
	T.f3c2 = matrix.f2c3;
	T.f3c3 = matrix.f3c3;
	T.f4c1 = matrix.f1c4;
	T.f4c2 = matrix.f2c4;
	T.f4c3 = matrix.f3c4;
	return T;
}

// A^-1 = ((adj(A))^T)/det(A)
controller_matrix4_4_t inverse(controller_matrix4_4_t matrix)
{
	float det;
	controller_matrix4_4_t inv, adj;

	det = (matrix.f1c1*matrix.f2c2*matrix.f3c3*matrix.f4c4) - (matrix.f1c2*matrix.f2c3*matrix.f3c4*matrix.f4c1)
		+ (matrix.f1c3*matrix.f2c4*matrix.f3c1*matrix.f4c2) - (matrix.f1c4*matrix.f2c1*matrix.f3c2*matrix.f4c3)
		+ (matrix.f4c1*matrix.f3c2*matrix.f2c3*matrix.f1c4) - (matrix.f4c2*matrix.f3c3*matrix.f2c4*matrix.f1c1)
		+ (matrix.f4c3*matrix.f3c4*matrix.f2c1*matrix.f1c2) - (matrix.f4c4*matrix.f3c1*matrix.f2c2*matrix.f1c3);

	adj.f1c1 = (matrix.f2c2*matrix.f3c3*matrix.f4c4) + (matrix.f2c3*matrix.f3c4*matrix.f4c2) + (matrix.f2c4*matrix.f3c2*matrix.f4c3)
			 - (matrix.f4c2*matrix.f3c3*matrix.f2c4) - (matrix.f4c3*matrix.f3c4*matrix.f2c2) - (matrix.f4c4*matrix.f3c2*matrix.f2c3);
	adj.f1c2 = -((matrix.f1c2*matrix.f3c3*matrix.f4c4) + (matrix.f1c3*matrix.f3c4*matrix.f4c2) + (matrix.f1c4*matrix.f3c2*matrix.f4c3)
			    -(matrix.f4c2*matrix.f3c3*matrix.f1c4) - (matrix.f4c3*matrix.f3c4*matrix.f1c2) - (matrix.f4c4*matrix.f3c2*matrix.f1c3));
	adj.f1c3 = (matrix.f1c2*matrix.f2c3*matrix.f4c4) + (matrix.f1c3*matrix.f2c4*matrix.f4c2) + (matrix.f1c4*matrix.f2c2*matrix.f4c3)
			 - (matrix.f4c2*matrix.f2c3*matrix.f1c4) - (matrix.f4c3*matrix.f2c4*matrix.f1c2) - (matrix.f4c4*matrix.f2c2*matrix.f1c3);
	adj.f1c4 = -((matrix.f1c1*matrix.f2c3*matrix.f3c4) + (matrix.f1c3*matrix.f2c4*matrix.f3c2) + (matrix.f1c4*matrix.f2c2*matrix.f3c3)
				-(matrix.f3c2*matrix.f2c3*matrix.f1c4) - (matrix.f3c3*matrix.f2c4*matrix.f1c1) - (matrix.f3c4*matrix.f2c2*matrix.f1c3));
	adj.f2c1 = -((matrix.f2c1*matrix.f3c3*matrix.f4c4) + (matrix.f2c3*matrix.f3c4*matrix.f4c1) + (matrix.f2c4*matrix.f3c1*matrix.f4c3)
				-(matrix.f4c1*matrix.f3c3*matrix.f2c4) - (matrix.f4c3*matrix.f3c4*matrix.f2c1) - (matrix.f4c4*matrix.f3c1*matrix.f2c3));
	adj.f2c2 = (matrix.f1c1*matrix.f3c3*matrix.f4c4) + (matrix.f1c3*matrix.f3c4*matrix.f4c1) + (matrix.f1c4*matrix.f3c1*matrix.f4c3)
			 - (matrix.f4c1*matrix.f3c3*matrix.f1c4) - (matrix.f4c3*matrix.f3c4*matrix.f1c1) - (matrix.f4c4*matrix.f3c1*matrix.f1c3);
	adj.f2c3 = -((matrix.f1c1*matrix.f2c3*matrix.f4c4) + (matrix.f1c3*matrix.f2c4*matrix.f4c1) + (matrix.f1c4*matrix.f2c1*matrix.f4c3)
			    -(matrix.f4c1*matrix.f2c3*matrix.f1c4) - (matrix.f4c3*matrix.f2c4*matrix.f1c1) - (matrix.f4c4*matrix.f2c1*matrix.f1c3));
	adj.f2c4 = (matrix.f1c1*matrix.f2c3*matrix.f3c4) + (matrix.f1c3*matrix.f2c4*matrix.f3c1) + (matrix.f1c4*matrix.f2c1*matrix.f3c3)
			 - (matrix.f3c1*matrix.f2c3*matrix.f1c4) - (matrix.f3c3*matrix.f2c4*matrix.f1c1) - (matrix.f3c4*matrix.f2c1*matrix.f1c3);
	adj.f3c1 = (matrix.f2c1*matrix.f3c2*matrix.f4c4) + (matrix.f2c2*matrix.f3c4*matrix.f4c1) + (matrix.f2c4*matrix.f3c1*matrix.f4c2)
			 - (matrix.f4c1*matrix.f3c2*matrix.f2c4) - (matrix.f4c2*matrix.f3c4*matrix.f2c1) - (matrix.f4c4*matrix.f3c1*matrix.f2c2);
	adj.f3c2 = -((matrix.f1c1*matrix.f3c2*matrix.f4c4) + (matrix.f1c2*matrix.f3c4*matrix.f4c1) + (matrix.f1c4*matrix.f3c1*matrix.f4c2)
			    -(matrix.f4c1*matrix.f3c2*matrix.f1c4) - (matrix.f4c2*matrix.f3c4*matrix.f1c1) - (matrix.f4c4*matrix.f3c1*matrix.f1c2));
	adj.f3c3 = (matrix.f1c1*matrix.f2c2*matrix.f4c4) + (matrix.f1c2*matrix.f2c4*matrix.f4c1) + (matrix.f1c4*matrix.f2c1*matrix.f4c2)
			 - (matrix.f4c1*matrix.f2c2*matrix.f1c4) - (matrix.f4c2*matrix.f2c4*matrix.f1c1) - (matrix.f4c4*matrix.f2c1*matrix.f1c2);
	adj.f3c4 = -((matrix.f1c1*matrix.f2c2*matrix.f3c4) + (matrix.f1c2*matrix.f2c4*matrix.f3c1) + (matrix.f1c4*matrix.f2c1*matrix.f3c2)
				-(matrix.f3c1*matrix.f2c2*matrix.f1c4) - (matrix.f3c2*matrix.f2c4*matrix.f1c1) - (matrix.f3c4*matrix.f2c1*matrix.f1c2));
	adj.f4c1 = -((matrix.f2c1*matrix.f3c2*matrix.f4c3) + (matrix.f2c2*matrix.f3c3*matrix.f4c1) + (matrix.f2c3*matrix.f3c1*matrix.f4c2)
			    -(matrix.f4c1*matrix.f3c2*matrix.f2c3) - (matrix.f4c2*matrix.f3c3*matrix.f2c1) - (matrix.f4c3*matrix.f3c1*matrix.f2c2));
	adj.f4c2 = (matrix.f1c1*matrix.f3c2*matrix.f4c3) + (matrix.f1c2*matrix.f3c3*matrix.f4c1) + (matrix.f1c3*matrix.f3c1*matrix.f4c2)
			 - (matrix.f4c1*matrix.f3c2*matrix.f1c3) - (matrix.f4c2*matrix.f3c3*matrix.f1c1) - (matrix.f4c3*matrix.f3c1*matrix.f1c2);
	adj.f4c3 = -((matrix.f1c1*matrix.f2c2*matrix.f4c3) + (matrix.f1c2*matrix.f2c3*matrix.f4c1) + (matrix.f1c3*matrix.f2c1*matrix.f4c2)
			    -(matrix.f4c1*matrix.f2c2*matrix.f1c3) - (matrix.f4c2*matrix.f2c3*matrix.f1c1) - (matrix.f4c3*matrix.f2c1*matrix.f1c2));
	adj.f4c4 = (matrix.f1c1*matrix.f2c2*matrix.f3c3) + (matrix.f1c2*matrix.f2c3*matrix.f3c1) + (matrix.f1c3*matrix.f2c1*matrix.f3c2)
		     - (matrix.f3c1*matrix.f2c2*matrix.f1c3) - (matrix.f3c2*matrix.f2c3*matrix.f1c1) - (matrix.f3c3*matrix.f2c1*matrix.f1c2);

	inv.f1c1 = adj.f1c1/det;
	inv.f1c2 = adj.f2c1/det;
	inv.f1c3 = adj.f3c1/det;
	inv.f1c4 = adj.f4c1/det;
	inv.f2c1 = adj.f1c2/det;
	inv.f2c2 = adj.f2c2/det;
	inv.f2c3 = adj.f3c2/det;
	inv.f2c4 = adj.f4c2/det;
	inv.f3c1 = adj.f1c3/det;
	inv.f3c2 = adj.f2c3/det;
	inv.f3c3 = adj.f3c3/det;
	inv.f3c4 = adj.f4c3/det;
	inv.f4c1 = adj.f1c4/det;
	inv.f4c2 = adj.f2c4/det;
	inv.f4c3 = adj.f3c4/det;
	inv.f4c4 = adj.f4c4/det;

	return inv;
}

controller_references_t Controller_reference(uint32_t time, controller_reference_t ref, controller_coordinates_t c, uint8_t size)
{
	controller_references_t r;
	uint16_t t;
	t = time % 360;

	switch(ref)
	{
	case CIRCLE:
	{
		r.xRef  = c.x + size*cos(t);
		r.dxRef = -((size*PI_ENCODER)/180)*sin(t);
		r.yRef  = c.y + size*sin(t);
		r.dyRef = ((size*PI_ENCODER)/180)*cos(t);
		break;
	}
	case SQUARE:
	{
		if(t >= 0 && t <= 45)
		{
			r.xRef  = c.x + size/2;
			r.dxRef = 0;
			r.yRef  = (t*(c.y + size/2))/45;
			r.dyRef = (c.y + size/2)/45;
		}
		else if(t > 45 && t <= 135)
		{
			r.xRef  = c.x + size - (t*(c.x + size))/90;
			r.dxRef = -(c.x + size)/90;
			r.yRef = c.y + size/2;
			r.dyRef = 0;
		}
		else if(t > 135 && t <= 225)
		{
			r.xRef  = -(c.x + size/2);
			r.dxRef = 0;
			r.yRef  = c.y + 2*size - (t*(c.y + size))/90;
			r.dyRef = -(c.y + size)/90;
		}
		else if(t > 225 && t <= 315)
		{
			r.xRef  = -(c.x + 3*size) + (t*(c.x + size))/90;
			r.dxRef = c.x + size/90;
			r.yRef  = -(c.y + size/2);
			r.dyRef = 0;
		}
		else
		{
			r.xRef  = c.x + size/2;
			r.dxRef = 0;
			r.yRef  = -(c.y + 4*size) + (t*(c.y + size/2))/45;
			r.dyRef = (c.y + size/2)/45;
		}
		break;
	}
	case SIN:
	{
		r.xRef  = c.x + size*sin(t);
		r.dxRef = ((size*PI_ENCODER)/180)*cos(t);
		r.yRef  = c.y + t;
		r.dyRef = 1;
		break;
	}
	default: break;
	}

	return r;
}

//					 C1				   C2				 C3				   C4
//  		|cos(psi)+sin(psi) cos(psi)-sin(psi) cos(psi)-sin(psi) cos(psi)+sin(psi)|	F1
// B = Rw/4	|sin(psi)-cos(psi) sin(psi)+cos(psi) sin(psi)+cos(psi) sin(psi)-cos(psi)|	F2
//			|      -1/(L+l)           1/(L+l)          -1/(L+l)           1/(L+l)]	|	F3
// C = |1 0 0|
//	   |0 1 0|
//	   |0 0 1|
controller_matrix3_4_t Controller_systemMatrix(int16_t psi)
{
	controller_matrix3_4_t B;

	B.f1c1 = (RW/4)*(cos(psi) + sin(psi));
	B.f1c2 = (RW/4)*(cos(psi) - sin(psi));
	B.f1c3 = (RW/4)*(cos(psi) - sin(psi));
	B.f1c4 = (RW/4)*(cos(psi) + sin(psi));
	B.f2c1 = (RW/4)*(sin(psi) - cos(psi));
	B.f2c2 = (RW/4)*(sin(psi) + cos(psi));
	B.f2c3 = (RW/4)*(sin(psi) + cos(psi));
	B.f2c4 = (RW/4)*(sin(psi) - cos(psi));
	B.f3c1 = (RW/4)*(-1/(L+l));
	B.f3c2 = (RW/4)*(1/(L+l));
	B.f3c3 = (RW/4)*(-1/(L+l));
	B.f3c4 = (RW/4)*(1/(L+l));

	return B;
}

controller_states_t Controller_error(float xRef, float yRef, float psiRef, controller_states_t currentState)
{
	controller_states_t e;

	e.x   = xRef - currentState.x;
	e.y   = yRef - currentState.y;
	e.psi = psiRef - currentState.psi;

	return e;
}

// pseudo-inverse of B = ((B^T*B)^-1)*B^T
controller_matrix4_3_t Controller_pseudoInverseMatrix(controller_matrix3_4_t B)
{
	controller_matrix4_3_t tB, pinvB;
	controller_matrix4_4_t tBxB, invB;

	tB = transposed(B);

	//4x3 * 3x4 = 4x4
	tBxB.f1c1 = (tB.f1c1*B.f1c1) + (tB.f1c2*B.f2c1) + (tB.f1c3*B.f3c1);
	tBxB.f1c2 = (tB.f1c1*B.f1c2) + (tB.f1c2*B.f2c2) + (tB.f1c3*B.f3c2);
	tBxB.f1c3 = (tB.f1c1*B.f1c3) + (tB.f1c2*B.f2c3) + (tB.f1c3*B.f3c3);
	tBxB.f1c4 = (tB.f1c1*B.f1c4) + (tB.f1c2*B.f2c4) + (tB.f1c3*B.f3c4);
	tBxB.f2c1 = (tB.f2c1*B.f1c1) + (tB.f2c2*B.f2c1) + (tB.f2c3*B.f2c1);
	tBxB.f2c2 = (tB.f2c1*B.f1c2) + (tB.f2c2*B.f2c2) + (tB.f2c3*B.f3c2);
	tBxB.f2c3 = (tB.f2c1*B.f1c3) + (tB.f2c2*B.f2c3) + (tB.f2c3*B.f3c3);
	tBxB.f2c4 = (tB.f2c1*B.f1c4) + (tB.f2c2*B.f2c4) + (tB.f2c3*B.f3c4);
	tBxB.f3c1 = (tB.f3c1*B.f1c1) + (tB.f3c2*B.f2c1) + (tB.f3c3*B.f2c1);
	tBxB.f3c2 = (tB.f3c1*B.f1c2) + (tB.f3c2*B.f2c2) + (tB.f3c3*B.f3c2);
	tBxB.f3c3 = (tB.f3c1*B.f1c3) + (tB.f3c2*B.f2c3) + (tB.f3c3*B.f3c3);
	tBxB.f3c4 = (tB.f3c1*B.f1c4) + (tB.f3c2*B.f2c4) + (tB.f3c3*B.f3c4);
	tBxB.f4c1 = (tB.f4c1*B.f1c1) + (tB.f4c2*B.f2c1) + (tB.f4c3*B.f2c1);
	tBxB.f4c2 = (tB.f4c1*B.f1c2) + (tB.f4c2*B.f2c2) + (tB.f4c3*B.f3c2);
	tBxB.f4c3 = (tB.f4c1*B.f1c3) + (tB.f4c2*B.f2c3) + (tB.f4c3*B.f3c3);
	tBxB.f4c4 = (tB.f4c1*B.f1c4) + (tB.f4c2*B.f2c4) + (tB.f4c3*B.f3c4);

	invB = inverse(tBxB);

	//4x4 * 4x3 = 4x3
	pinvB.f1c1 = (invB.f1c1*tB.f1c1) + (invB.f1c2*tB.f2c1) + (invB.f1c3*tB.f3c1) + (invB.f1c4*tB.f4c1);
	pinvB.f1c2 = (invB.f1c1*tB.f1c2) + (invB.f1c2*tB.f2c2) + (invB.f1c3*tB.f3c2) + (invB.f1c4*tB.f4c2);
	pinvB.f1c3 = (invB.f1c1*tB.f1c3) + (invB.f1c2*tB.f2c3) + (invB.f1c3*tB.f3c3) + (invB.f1c4*tB.f4c3);
	pinvB.f2c1 = (invB.f2c1*tB.f1c1) + (invB.f2c2*tB.f2c1) + (invB.f2c3*tB.f3c1) + (invB.f2c4*tB.f4c1);
	pinvB.f2c2 = (invB.f2c1*tB.f1c2) + (invB.f2c2*tB.f2c2) + (invB.f2c3*tB.f3c2) + (invB.f2c4*tB.f4c2);
	pinvB.f2c3 = (invB.f2c1*tB.f1c3) + (invB.f2c2*tB.f2c3) + (invB.f2c3*tB.f3c3) + (invB.f2c4*tB.f4c3);
	pinvB.f3c1 = (invB.f3c1*tB.f1c1) + (invB.f3c2*tB.f2c1) + (invB.f3c3*tB.f3c1) + (invB.f3c4*tB.f4c1);
	pinvB.f3c2 = (invB.f3c1*tB.f1c2) + (invB.f3c2*tB.f2c2) + (invB.f3c3*tB.f3c2) + (invB.f3c4*tB.f4c2);
	pinvB.f3c3 = (invB.f3c1*tB.f1c3) + (invB.f3c2*tB.f2c3) + (invB.f3c3*tB.f3c3) + (invB.f3c4*tB.f4c3);
	pinvB.f4c1 = (invB.f4c1*tB.f1c1) + (invB.f4c2*tB.f2c1) + (invB.f4c3*tB.f3c1) + (invB.f4c4*tB.f4c1);
	pinvB.f4c2 = (invB.f4c1*tB.f1c2) + (invB.f4c2*tB.f2c2) + (invB.f4c3*tB.f3c2) + (invB.f4c4*tB.f4c2);
	pinvB.f4c3 = (invB.f4c1*tB.f1c3) + (invB.f4c2*tB.f2c3) + (invB.f4c3*tB.f3c3) + (invB.f4c4*tB.f4c3);

	return pinvB;
}

// 4x3 * 3x1 = 4x1
controller_controlLawMatrix_t Controller_controlLaw(float dxRef, float dyRef, float dpsiRef, controller_states_t e, controller_matrix4_3_t pinvB)
{
	controller_controlLawMatrix_t U;
	controller_states_t matrix;
	matrix.x   = dxRef - (KE*e.x);
	matrix.y   = dyRef - (KE*e.y);
	matrix.psi = dpsiRef - (KE*e.psi);

	U.w1 = (pinvB.f1c1*matrix.x) + (pinvB.f1c2*matrix.y) + (pinvB.f1c3*matrix.psi);
	U.w2 = (pinvB.f2c1*matrix.x) + (pinvB.f2c2*matrix.y) + (pinvB.f2c3*matrix.psi);
	U.w3 = (pinvB.f3c1*matrix.x) + (pinvB.f3c2*matrix.y) + (pinvB.f3c3*matrix.psi);
	U.w4 = (pinvB.f4c1*matrix.x) + (pinvB.f4c2*matrix.y) + (pinvB.f4c3*matrix.psi);

	return U;
}

// 3x4 * 4x1 = 3x1
controller_states_t Controller_odes(controller_matrix3_4_t B, controller_controlLawMatrix_t U, controller_states_t currentState)
{
	controller_states_t nxtState;
	nxtState.x   = (B.f1c1*U.w1) + (B.f1c2*U.w2) + (B.f1c3*U.w3) + (B.f1c1*U.w4);
	nxtState.y   = (B.f2c1*U.w1) + (B.f2c2*U.w2) + (B.f2c3*U.w3) + (B.f2c1*U.w4);
	nxtState.psi = (B.f3c1*U.w1) + (B.f3c2*U.w2) + (B.f3c3*U.w3) + (B.f3c1*U.w4);

	return nxtState;
}


