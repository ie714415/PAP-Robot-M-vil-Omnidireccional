/**
	\file
	\brief
		This is the source file for the Dual VNH5019 Motor Driver Shield.
		It contains all the implementation for all the elements for the configuration and use of the driver.
		i.e., this is the application programming interface (API) for the Dual VNH5019 Motor Driver Shield.
	\author Yadira Vanessa Bonifacio Benavides ie71445@iteso.mx
 */

#include "MK64F12.h"
#include "DualVNH5019MotorShield.h"
#include "GPIO.h"
#include "FlexTimer.h"
#include "fsl_debug_console.h"

/**Variables that indicate which bits to turn on in the control register*/
const gpio_pin_control_register_t g_pinControlRegister = GPIO_MUX1;
const gpio_pin_control_register_t g_pinControlRegisterPWM = GPIO_MUX4;

void initMotorDriverShield(void)
{
	/**Pin control configuration*/
	GPIO_pin_control_register(ENABLE_PORT, ENABLE_PIN, &g_pinControlRegister);
	/**Configures pin as output*/
	GPIO_data_direction_pin(ENABLE_PORT, GPIO_OUTPUT, ENABLE_PIN);

	/*Pin configuration motor 1*/
	//M1INA
	GPIO_pin_control_register(M1INA_PORT, M1INA_PIN, &g_pinControlRegister);
	GPIO_data_direction_pin(M1INA_PORT, GPIO_OUTPUT, M1INA_PIN);
	//M1INB
	GPIO_pin_control_register(M1INB_PORT, M1INB_PIN, &g_pinControlRegister);
	GPIO_data_direction_pin(M1INB_PORT, GPIO_OUTPUT, M1INB_PIN);
	//M1PWM (channel 0)
	GPIO_pin_control_register(M1PWM_PORT, M1PWM_PIN, &g_pinControlRegisterPWM);

	/*Pin configuration motor 2*/
	//M2INA
	GPIO_pin_control_register(M2INA_PORT, M2INA_PIN, &g_pinControlRegister);
	GPIO_data_direction_pin(M2INA_PORT, GPIO_OUTPUT, M2INA_PIN);
	//M2INB
	GPIO_pin_control_register(M2INB_PORT, M2INB_PIN, &g_pinControlRegister);
	GPIO_data_direction_pin(M2INB_PORT, GPIO_OUTPUT, M2INB_PIN);
	//M2PWM (channel 1)
	GPIO_pin_control_register(M2PWM_PORT, M2PWM_PIN, &g_pinControlRegisterPWM);

	/*Pin configuration motor 3*/
	//M2INA
	GPIO_pin_control_register(M3INA_PORT, M3INA_PIN, &g_pinControlRegister);
	GPIO_data_direction_pin(M3INA_PORT, GPIO_OUTPUT, M3INA_PIN);
	//M3INB
	GPIO_pin_control_register(M3INB_PORT, M3INB_PIN, &g_pinControlRegister);
	GPIO_data_direction_pin(M3INB_PORT, GPIO_OUTPUT, M3INB_PIN);
	//M3PWM (channel 2)
	GPIO_pin_control_register(M3PWM_PORT, M3PWM_PIN, &g_pinControlRegisterPWM);

	//M2INA
	GPIO_pin_control_register(M4INA_PORT, M4INA_PIN, &g_pinControlRegister);
	GPIO_data_direction_pin(M4INA_PORT, GPIO_OUTPUT, M4INA_PIN);
	//M4INB
	GPIO_pin_control_register(M4INB_PORT, M4INB_PIN, &g_pinControlRegister);
	GPIO_data_direction_pin(M4INB_PORT, GPIO_OUTPUT, M4INB_PIN);
	//M4PWM (channel 3)
	GPIO_pin_control_register(M4PWM_PORT, M4PWM_PIN, &g_pinControlRegisterPWM);

	/*!Initialization FlexTimer*/
	FlexTimer_init();
}

// Set voltage for motor, where voltage is a number between -255 and 255 witch denotes the value for the PWM
// and the direction of the motor
void setM1Voltage(int16_t voltage)
{
	uint8_t reverse = 0;
	uint16_t dutyCycle;

	if(MIN_DUTY_CYCLE > voltage)
	{
		dutyCycle = -voltage;  // Make voltage a positive quantity
		reverse = 1;  // Preserve the direction
	}
	else if(MAX_DUTY_CYCLE < voltage)  // Max PWM duty cycle
		dutyCycle = 255;
	else
		dutyCycle = voltage;

	if(MIN_DUTY_CYCLE == dutyCycle)
	{
		GPIO_clear_pin(M1INA_PORT, M1INA_PIN);   // Make the motor coast no
		GPIO_clear_pin(M1INB_PORT, M1INB_PIN);   // matter which direction it is spinning.
	}
	else if(reverse)
	{
		GPIO_clear_pin(M1INA_PORT, M1INA_PIN);
		GPIO_set_pin(M1INB_PORT, M1INB_PIN);
	}
	else
	{
		GPIO_set_pin(M1INA_PORT, M1INA_PIN);
		GPIO_clear_pin(M1INB_PORT, M1INB_PIN);
	}

	FlexTimer_update_channel_value(dutyCycle, channel_0);
}

void setM2Voltage(int16_t voltage)
{
	uint8_t reverse = 0;
	uint16_t dutyCycle;

	if(MIN_DUTY_CYCLE > voltage)
	{
		dutyCycle = -voltage;  // Make voltage a positive quantity
		reverse = 1;  // Preserve the direction
	}
	else if(MAX_DUTY_CYCLE < voltage)  // Max PWM duty cycle
		dutyCycle = 255;
	else
		dutyCycle = voltage;

	if(MIN_DUTY_CYCLE == dutyCycle)
	{
		GPIO_clear_pin(M2INA_PORT, M2INA_PIN);   // Make the motor coast no
		GPIO_clear_pin(M2INB_PORT, M2INB_PIN);   // matter which direction it is spinning.
	}
	else if(reverse)
	{
		GPIO_clear_pin(M2INA_PORT, M2INA_PIN);
		GPIO_set_pin(M2INB_PORT, M2INB_PIN);
	}
	else
	{
		GPIO_set_pin(M2INA_PORT, M2INA_PIN);
		GPIO_clear_pin(M2INB_PORT, M2INB_PIN);
	}

	FlexTimer_update_channel_value(dutyCycle, channel_1);
}

void setM3Voltage(int16_t voltage)
{
	uint8_t reverse = 0;
	uint16_t dutyCycle;

	if(MIN_DUTY_CYCLE > voltage)
	{
		dutyCycle = -voltage;  // Make voltage a positive quantity
		reverse = 1;  // Preserve the direction
	}
	else if(MAX_DUTY_CYCLE < voltage)  // Max PWM duty cycle
		dutyCycle = 255;
	else
		dutyCycle = voltage;

	if(MIN_DUTY_CYCLE == dutyCycle)
	{
		GPIO_clear_pin(M3INA_PORT, M3INA_PIN);   // Make the motor coast no
		GPIO_clear_pin(M3INB_PORT, M3INB_PIN);   // matter which direction it is spinning.
	}
	else if(reverse)
	{
		GPIO_clear_pin(M3INA_PORT, M3INA_PIN);
		GPIO_set_pin(M3INB_PORT, M3INB_PIN);
	}
	else
	{
		GPIO_set_pin(M3INA_PORT, M3INA_PIN);
		GPIO_clear_pin(M3INB_PORT, M3INB_PIN);
	}

	FlexTimer_update_channel_value(dutyCycle, channel_2);
}

void setM4Voltage(int16_t voltage)
{
	uint8_t reverse = 0;
	uint16_t dutyCycle;

	if(MIN_DUTY_CYCLE > voltage)
	{
		dutyCycle = -voltage;  // Make voltage a positive quantity
		reverse = 1;  // Preserve the direction
	}
	else if(MAX_DUTY_CYCLE < voltage)  // Max PWM duty cycle
		dutyCycle = 255;
	else
		dutyCycle = voltage;

	if(MIN_DUTY_CYCLE == dutyCycle)
	{
		GPIO_clear_pin(M4INA_PORT, M4INA_PIN);   // Make the motor coast no
		GPIO_clear_pin(M4INB_PORT, M4INB_PIN);   // matter which direction it is spinning.
	}
	else if(reverse)
	{
		GPIO_clear_pin(M4INA_PORT, M4INA_PIN);
		GPIO_set_pin(M4INB_PORT, M4INB_PIN);
	}
	else
	{
		GPIO_set_pin(M4INA_PORT, M4INA_PIN);
		GPIO_clear_pin(M4INB_PORT, M4INB_PIN);
	}

	FlexTimer_update_channel_value(dutyCycle, channel_3);
}
void setVoltages(int16_t m1Voltage, int16_t m2Voltage, int16_t m3Voltage, int16_t m4Voltage)
{
	setM1Voltage(m1Voltage);
	setM2Voltage(m2Voltage);
	setM3Voltage(m3Voltage);
	setM4Voltage(m4Voltage);
}

void setM1Brake(int16_t brake)
{
	uint16_t dutyCycle;
	// normalize brake
	if(MIN_DUTY_CYCLE > brake)
		dutyCycle = -brake;
	else if(MAX_DUTY_CYCLE < brake)  // Max brake
		dutyCycle = 255;
	else
		dutyCycle = brake;

	GPIO_clear_pin(M1INA_PORT, M1INA_PIN);
	GPIO_clear_pin(M1INB_PORT, M1INB_PIN);

	FlexTimer_update_channel_value(dutyCycle, channel_0);
}

void setM2Brake(int16_t brake)
{
	uint16_t dutyCycle;
	// normalize brake
	if(MIN_DUTY_CYCLE > brake)
		dutyCycle = -brake;
	else if(MAX_DUTY_CYCLE < brake)  // Max brake
		dutyCycle = 255;
	else
		dutyCycle = brake;

	GPIO_clear_pin(M2INA_PORT, M2INA_PIN);
	GPIO_clear_pin(M2INB_PORT, M2INB_PIN);

	FlexTimer_update_channel_value(dutyCycle, channel_1);
}

void setM3Brake(int16_t brake)
{
	uint16_t dutyCycle;
	// normalize brake
	if(MIN_DUTY_CYCLE > brake)
		dutyCycle = -brake;
	else if(MAX_DUTY_CYCLE < brake)  // Max brake
		dutyCycle = 255;
	else
		dutyCycle = brake;

	GPIO_clear_pin(M3INA_PORT, M3INA_PIN);
	GPIO_clear_pin(M3INB_PORT, M3INB_PIN);

	FlexTimer_update_channel_value(dutyCycle, channel_2);
}

void setM4Brake(int16_t brake)
{
	uint16_t dutyCycle;
	// normalize brake
	if(MIN_DUTY_CYCLE > brake)
		dutyCycle = -brake;
	else if(MAX_DUTY_CYCLE < brake)  // Max brake
		dutyCycle = 255;
	else
		dutyCycle = brake;

	GPIO_clear_pin(M4INA_PORT, M4INA_PIN);
	GPIO_clear_pin(M4INB_PORT, M4INB_PIN);

	FlexTimer_update_channel_value(dutyCycle, channel_3);
}

void setBrakes(int16_t m1Brake, int16_t m2Brake, int16_t m3Brake, int16_t m4Brake)
{
	setM1Brake(m1Brake);
	setM2Brake(m2Brake);
	setM3Brake(m3Brake);
	setM4Brake(m4Brake);
}

void setMotorDirectionAndVoltage(motor_direction_t direction, int16_t m1Voltage, int16_t m2Voltage, int16_t m3Voltage, int16_t m4Voltage)
{
	/*The maximum forward Voltage is 255, the maximum reverse Voltage is -255 and the minimum Voltage 0*/
	switch(direction)
	{
	case FORWARD:
		setVoltages(m1Voltage, m1Voltage, m1Voltage, m1Voltage);						break;
	case BACKWARD:
		setVoltages(-m1Voltage, -m1Voltage, -m1Voltage, -m1Voltage);					break;
	case LEFT:
		setVoltages(m1Voltage, -m1Voltage, -m1Voltage, m1Voltage);						break;
	case RIGHT:
		setVoltages(-m1Voltage, m1Voltage, m1Voltage, -m1Voltage);						break;
	case TOP_LEFT:
		setVoltages(m1Voltage, MIN_DUTY_CYCLE, MIN_DUTY_CYCLE, m1Voltage);				break;
	case TOP_RIGHT:
		setVoltages(MIN_DUTY_CYCLE, m1Voltage, m1Voltage, MIN_DUTY_CYCLE);				break;
	case BOTTOM_LEFT:
		setVoltages(MIN_DUTY_CYCLE, -m1Voltage, -m1Voltage, MIN_DUTY_CYCLE);			break;
	case BOTTOM_RIGHT:
		setVoltages(-m1Voltage, MIN_DUTY_CYCLE, MIN_DUTY_CYCLE, -m1Voltage);			break;
	case CLOCK_WISE:
		setVoltages(-m1Voltage, -m1Voltage, m1Voltage, m1Voltage);						break;
	case COUNTER_CLOCK_WISE:
		setVoltages(m1Voltage, m1Voltage, -m1Voltage, -m1Voltage);						break;
	default:
		setVoltages(MIN_DUTY_CYCLE, MIN_DUTY_CYCLE, MIN_DUTY_CYCLE, MIN_DUTY_CYCLE);	break;
	}
}
