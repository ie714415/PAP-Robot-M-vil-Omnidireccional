/**
	\file
	\brief
		This is the source file for the Encoder of the DC Motor .
		It contains all the implementation for all the elements for the configuration and use of the driver.
		i.e., this is the application programming interface (API) for the Motor DC Encoder.
	\author Yadira Vanessa Bonifacio Benavides ie71445@iteso.mx
 */

#include "Encoder.h"
#include "GPIO.h"

/**Variables that indicates which bits to turn on in the control register*/
const gpio_pin_control_register_t g_pinControlRegisterA = GPIO_MUX1 | INTR_RISING_EDGE;
const gpio_pin_control_register_t g_pinControlRegisterB = GPIO_MUX1;
/**Variables that indicates the frequency, time per rising edge of A and direction of the encoder*/
static Encoder_internal_state_t g_encodersInternalState = {0, 0, NEGATIVE, 0, 0, NEGATIVE, 0, 0, NEGATIVE};
/**Variable that stores the elapsed nx10^-5 seconds*/
static uint32_t g_time = 0;

void Encoder_init(void)
{
	/*Pin configuration encoder 1*/
	//ENC1 A
	/**Pin control configuration*/
	GPIO_pin_control_register(ENC1_A_PORT, ENC1_A_PIN, &g_pinControlRegisterA);
	/**Configures pin as input*/
	GPIO_data_direction_pin(ENC1_A_PORT, GPIO_INPUT, ENC1_A_PIN);
	//ENC1 B
	GPIO_pin_control_register(ENC1_B_PORT, ENC1_B_PIN, &g_pinControlRegisterB);
	GPIO_data_direction_pin(ENC1_B_PORT, GPIO_INPUT, ENC1_B_PIN);

	/*Pin configuration encoder 2*/
	//ENC2 A
	GPIO_pin_control_register(ENC2_A_PORT, ENC2_A_PIN, &g_pinControlRegisterA);
	GPIO_data_direction_pin(ENC2_A_PORT, GPIO_INPUT, ENC2_A_PIN);
	//ENC2 B
	GPIO_pin_control_register(ENC2_B_PORT, ENC2_B_PIN, &g_pinControlRegisterB);
	GPIO_data_direction_pin(ENC2_B_PORT, GPIO_INPUT, ENC2_B_PIN);

	/*Pin configuration encoder 3*/
	//ENC3 A
	GPIO_pin_control_register(ENC3_A_PORT, ENC3_A_PIN, &g_pinControlRegisterA);
	GPIO_data_direction_pin(ENC3_A_PORT, GPIO_INPUT, ENC3_A_PIN);
	//ENC3 B
	GPIO_pin_control_register(ENC3_B_PORT, ENC3_B_PIN, &g_pinControlRegisterB);
	GPIO_data_direction_pin(ENC3_B_PORT, GPIO_INPUT, ENC3_B_PIN);

	/*Pin configuration encoder 4*/
	//ENC4 A
	GPIO_pin_control_register(ENC4_A_PORT, ENC4_A_PIN, &g_pinControlRegisterA);
	GPIO_data_direction_pin(ENC4_A_PORT, GPIO_INPUT, ENC4_A_PIN);
	//ENC4 B
	GPIO_pin_control_register(ENC4_B_PORT, ENC4_B_PIN, &g_pinControlRegisterB);
	GPIO_data_direction_pin(ENC4_B_PORT, GPIO_INPUT, ENC4_B_PIN);
}

//                        _______         _______
//               A ______|       |_______|       |______ A
// negative <---      _______         _______         __   ---> positive
//               B __|       |_______|       |_______|   B

void Encoder1_ISR(void)
{
	uint32_t time;
	uint32_t currentTime;
	encoder_state_t state;

	currentTime = g_time;
	time = currentTime - g_encodersInternalState.enc1_time;
	g_encodersInternalState.enc1_time = currentTime;
	g_encodersInternalState.enc1_freq = (uint32_t)(100000/time);	//frequency = 1/time

	state = Encoder_bState(ENC_1);
	if(HIGH == state)
		g_encodersInternalState.enc1_dir = NEGATIVE;

	else
		g_encodersInternalState.enc1_dir = POSITIVE;
}

void Encoder2_ISR(void)
{
	uint32_t time;
	uint32_t currentTime;
	encoder_state_t state;

	currentTime = g_time;
	time = currentTime - g_encodersInternalState.enc2_time;
	g_encodersInternalState.enc2_time = currentTime;
	g_encodersInternalState.enc2_freq = (uint32_t)(100000/time);	//frequency = 1/time

	state = Encoder_bState(ENC_2);
	if(HIGH == state)
		g_encodersInternalState.enc2_dir = NEGATIVE;

	else
		g_encodersInternalState.enc2_dir = POSITIVE;
}

void Encoder3_ISR(void)
{
	uint32_t time;
	uint32_t currentTime;
	encoder_state_t state;

	currentTime = g_time;
	time = currentTime - g_encodersInternalState.enc3_time;
	g_encodersInternalState.enc3_time = currentTime;
	g_encodersInternalState.enc3_freq = (uint32_t)(100000/time);	//frequency = 1/time

	state = Encoder_bState(ENC_3);
	if(HIGH == state)
		g_encodersInternalState.enc3_dir = NEGATIVE;

	else
		g_encodersInternalState.enc3_dir = POSITIVE;
}

void Encoder4_ISR(void)
{
	uint32_t time;
	uint32_t currentTime;
	encoder_state_t state;

	currentTime = g_time;
	time = currentTime - g_encodersInternalState.enc4_time;
	g_encodersInternalState.enc4_time = currentTime;
	g_encodersInternalState.enc4_freq = (uint32_t)(100000/time);	//frequency = 1/time

	state = Encoder_bState(ENC_4);
	if(HIGH == state)
		g_encodersInternalState.enc4_dir = NEGATIVE;

	else
		g_encodersInternalState.enc4_dir = POSITIVE;
}

void Encoder_counter(void)
{
	g_time++;
}

encoder_state_t Encoder_bState(encoder_name_t encoder)
{
	encoder_state_t state;
	uint32_t portValue;

	switch(encoder)
	{
	case ENC_1:
	{
		portValue = GPIO_read_port(ENC1_B_PORT) & ENC1B_MASK;

		if(ENC1B_MASK == portValue)
			state = HIGH;
		else
			state = LOW;
		break;
	}
	case ENC_2:
	{
		portValue = GPIO_read_port(ENC2_B_PORT) & ENC2B_MASK;

		if(ENC2B_MASK == portValue)
			state = HIGH;
		else
			state = LOW;
		break;
	}
	case ENC_3:
	{
		portValue = GPIO_read_port(ENC3_B_PORT) & ENC3B_MASK;

		if(ENC3B_MASK == portValue)
			state = HIGH;
		else
			state = LOW;
		break;
	}
	case ENC_4:
	{
		portValue = GPIO_read_port(ENC4_B_PORT) & ENC4B_MASK;

		if(ENC4B_MASK == portValue)
			state = HIGH;
		else
			state = LOW;
		break;
	}
	default:
		break;
	}
	return state;
}

float Encoder_getAngularSpeed(encoder_name_t encoder)
{
	float speed;

	switch(encoder)
	{
	case ENC_1:
		speed = (2* PI_ENCODER * g_encodersInternalState.enc1_freq)/CPR; break;
	case ENC_2:
		speed = (2* PI_ENCODER * g_encodersInternalState.enc2_freq)/CPR; break;
	case ENC_3:
		speed = (2* PI_ENCODER * g_encodersInternalState.enc3_freq)/CPR; break;
	case ENC_4:
		speed = (2* PI_ENCODER * g_encodersInternalState.enc4_freq)/CPR; break;
	}
	return speed;
}

