/**
	\file
	\brief
		This is the source file for the Encoder of the DC Motor .
		It contains all the implementation for all the elements for the configuration and use of the driver.
		i.e., this is the application programming interface (API) for the Motor DC Encoder.
	\author Yadira Vanessa Bonifacio Benavides ie71445@iteso.mx
 */

#ifndef ENCODER_H_
#define ENCODER_H_

#include "stdint.h"
#include "GPIO.h"
#include "Bits.h"

/**Constant that represent the connection pins*/
/*Pin configuration encoder 1*/
#define ENC1_A_PORT GPIO_C
#define ENC1_A_PIN 	bit_7
#define ENC1_B_PORT GPIO_C
#define ENC1_B_PIN 	bit_0
/*Pin configuration encoder 2*/
#define ENC2_A_PORT GPIO_A
#define ENC2_A_PIN 	bit_2
#define ENC2_B_PORT GPIO_B
#define ENC2_B_PIN 	bit_23
/*Pin configuration encoder 3*/
#define ENC3_A_PORT GPIO_E
#define ENC3_A_PIN 	bit_25
#define ENC3_B_PORT GPIO_E
#define ENC3_B_PIN 	bit_26
/*Pin configuration encoder 4*/
#define ENC4_A_PORT GPIO_D
#define ENC4_A_PIN 	bit_1
#define ENC4_B_PORT GPIO_D
#define ENC4_B_PIN 	bit_3

/**Constant that represent the port masks to obtain the value of a specific bit*/
#define ENC1B_MASK 0x00040000
#define ENC2B_MASK 0x00000200
#define ENC3B_MASK 0x00000001
#define ENC4B_MASK 0x00000001

#define CPR 		48U
#define PI_ENCODER 	3.1416F
#define MAX_COUNTER 0xFFFFFFFF

/*!These constants are used to select an specific state of the encoder*/
typedef enum {LOW, HIGH} encoder_state_t;
/*!These constants are used to select an specific direction*/
typedef enum {NEGATIVE, POSITIVE} encoder_direction_t;
/*! These constants are used to select an specific encoder*/
typedef enum {ENC_1, ENC_2, ENC_3, ENC_4} encoder_name_t;

/*!These constants are used to save the encoders time per revolution and direction*/
typedef struct {
	/*Internal state encoder 1*/
	uint32_t enc1_time;
	uint32_t enc1_freq;
	encoder_direction_t enc1_dir;
	/*Internal state encoder 2*/
	uint32_t enc2_time;
	uint32_t enc2_freq;
	encoder_direction_t enc2_dir;
	/*Internal state encoder 3*/
	uint32_t enc3_time;
	uint32_t enc3_freq;
	encoder_direction_t enc3_dir;
	/*Internal state encoder 4*/
	uint32_t enc4_time;
	uint32_t enc4_freq;
	encoder_direction_t enc4_dir;
} Encoder_internal_state_t;

void Encoder_init(void);
void Encoder1_ISR(void);
void Encoder2_ISR(void);
void Encoder3_ISR(void);
void Encoder4_ISR(void);
void Encoder_counter(void);
encoder_state_t Encoder_bState(encoder_name_t encoder);
float Encoder_getAngularSpeed(encoder_name_t encoder);

#endif /* ENCODER_H_ */
