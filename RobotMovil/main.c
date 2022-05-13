/*Include files*/
#include "board.h"
#include "stdint.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK64F12.h"
#include "fsl_debug_console.h"
#include "GPIO.h"
#include "NVIC.h"
#include "PIT.h"
#include "DualVNH5019MotorShield.h"
#include "Encoder.h"
#include "ADC.h"
#include "MCG.h"
#include "PID.h"
#include "PsiController.h"

#define DEBUG

#ifdef DEBUG
	#include "stdio.h"
#endif

/*Definitions and declarations*/
#define MAX_S    	   360		 /*Constant that represent the maximun value for 1x10^-2 seconds count*/
#define DELAY_S		   0.0125 	 /*Constant that represent 1x10^-2 second*/
#define DELAY 		   0.0000125 /*Constant that represent 1x10^-5 second*/
#define CLK_FREQ_HZ    50000000  /*CLKIN0 frequency*/
#define SLOW_IRC_FREQ  32768	 /*This is the approximate value for the slow irc*/
#define FAST_IRC_FREQ  4000000 	 /*This is the approximate value for the fast irc*/
#define EXTERNAL_CLOCK 0 		 /*It defines an external clock*/
#define PLL_ENABLE 	   1 		 /*PLL is enabled*/
#define PLL_DISABLE    0 		 /*PLL is disabled*/
#define CRYSTAL_OSC    1  		 /*It defines an crystal oscillator*/
#define LOW_POWER 	   0    	 /*Set the oscillator for low power mode */
#define SLOW_IRC 	   0 		 /*Set the slow IRC */
#define CLK0_TYPE 	   0     	 /*Crystal or canned oscillator clock input */
#define PLL0_PRDIV 	   25    	 /*PLL predivider value */
#define PLL0_VDIV 	   30    	 /*PLL multiplier value*/

/*!These constants are used to save the angular speed (reference and actual) and voltage of each motor*/
typedef struct {
	/*Motor 1*/
	float m1_wAcc;
	float m1_wRef;
	int16_t m1_voltage;
	/*Motor 2*/
	float m2_wAcc;
	float m2_wRef;
	int16_t m2_voltage;
	/*Motor 3*/
	float m3_wAcc;
	float m3_wRef;
	int16_t m3_voltage;
	/*Motor 4*/
	float m4_wAcc;
	float m4_wRef;
	int16_t m4_voltage;
} internal_parameters_t;

static internal_parameters_t g_motorsParam = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
/**Variables that receive the ADC value*/
static float currentMilliAmp = 0;
/**Variable that stores the elapsed milliseconds*/
static uint16_t g_time = 0;

void SeconsCounter(void)
{
	g_time++;
	if(MAX_S < g_time)
		g_time = 0;
}

int main(void)
{
    /*Init board hardware*/
	int mcg_clk_hz;

#ifndef PLL_DIRECT_INIT
	mcg_clk_hz = fei_fbi(SLOW_IRC_FREQ,SLOW_IRC);				// 64 Hz ---> 32768
	mcg_clk_hz = fbi_fbe(CLK_FREQ_HZ,LOW_POWER,EXTERNAL_CLOCK); // 97.656KHz ---> 50000000
	mcg_clk_hz = fbe_pbe(CLK_FREQ_HZ,PLL0_PRDIV,PLL0_VDIV);		// 97.656KHz ---> 50000000 and PLL is configured to generate 60000000
	mcg_clk_hz =  pbe_pee(CLK_FREQ_HZ);							// 117.18 KHz ---> 60000000
#else
	mcg_clk_hz = pll_init(CLK_FREQ_HZ, LOW_POWER, EXTERNAL_CLOCK, PLL0_PRDIV, PLL0_VDIV, PLL_ENABLE);
#endif

	BOARD_InitBootPins();
	BOARD_InitBootClocks();
	BOARD_InitBootPeripherals();
#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
	/* Init FSL debug console. */
	BOARD_InitDebugConsole();
#endif

	/**Activating the GPIOA and GPIC clock gating*/
	GPIO_clock_gating(GPIO_A);
	GPIO_clock_gating(GPIO_B);
	GPIO_clock_gating(GPIO_C);
	GPIO_clock_gating(GPIO_D);
	GPIO_clock_gating(GPIO_E);

	/**Init Dual VNH5019 Motor Driver Shield*/
	initMotorDriverShield();
	/*!Init Analog to Digital Convert*/
	//ADC_init();
	/**Init Motor DC Encoder*/
	Encoder_init();
	/**Activating the PIT clock gating*/
	PIT_clock_gating();
	/**Activating the PIT clock*/
	PIT_enable();

	/**Callback for GPIO*/
	GPIO_callback_init(GPIO_A, Encoder2_ISR);
	GPIO_callback_init(GPIO_B, Encoder1_ISR);
	GPIO_callback_init(GPIO_C, Encoder3_ISR);
	GPIO_callback_init(GPIO_D, Encoder4_ISR);
	/**Callback for PIT*/
	PIT_callback_init(PIT_0, Encoder_counter);
	PIT_callback_init(PIT_1, SeconsCounter);

	/**Enables and sets a PORTs interrupts and its priority*/
	NVIC_enable_interrupt_and_priotity(PORTA_IRQ, PRIORITY_8);
	NVIC_enable_interrupt_and_priotity(PORTB_IRQ, PRIORITY_8);
	NVIC_enable_interrupt_and_priotity(PORTC_IRQ, PRIORITY_8);
	NVIC_enable_interrupt_and_priotity(PORTD_IRQ, PRIORITY_8);
	/**Enables and sets a PIT interrupt and its priority*/
	NVIC_enable_interrupt_and_priotity(PIT_CH0_IRQ, PRIORITY_10);
	NVIC_enable_interrupt_and_priotity(PIT_CH1_IRQ, PRIORITY_7);
	NVIC_global_enable_interrupts;

	GPIO_set_pin(ENABLE_PORT, ENABLE_PIN);

	/** PIT start for one microsecond*/
	PIT_delay(PIT_0, CLK_FREQ_HZ, DELAY);
	PIT_delay(PIT_1, CLK_FREQ_HZ, DELAY_S);

     while(1)
    {
    	//3.3V / 255 ADC counts / 0.14 V per A = 92.4 mA per count
    	//currentMilliAmp = ADC_result() * 92.4;
    	 setM1Voltage(255);
    	 setM2Voltage(255);
    	 setM3Voltage(-255);
    	 setM4Voltage(-255);

    	/*g_motorsParam.m1_wRef = 20.94;//340.34;
    	g_motorsParam.m2_wRef = 20.94;
    	g_motorsParam.m3_wRef = 20.94;
    	g_motorsParam.m4_wRef = 20.94;
    	for(int indx = 0; 4900 >= indx; indx++)
    	{
    		g_motorsParam.m1_wAcc = Encoder_getAngularSpeed(ENC_1);
    		g_motorsParam.m1_voltage = PID_controller(MOTOR_1, g_motorsParam.m1_wRef, g_motorsParam.m1_wAcc);
    		setM1Voltage(g_motorsParam.m1_voltage);
    		g_motorsParam.m2_wAcc = Encoder_getAngularSpeed(ENC_2);
    		g_motorsParam.m2_voltage = PID_controller(MOTOR_2, g_motorsParam.m2_wRef, g_motorsParam.m2_wAcc);
    		setM2Voltage(g_motorsParam.m2_voltage);
    		g_motorsParam.m3_wAcc = Encoder_getAngularSpeed(ENC_3);
    		g_motorsParam.m3_voltage = PID_controller(MOTOR_3, g_motorsParam.m3_wRef, g_motorsParam.m3_wAcc);
    		setM3Voltage(g_motorsParam.m3_voltage);
    		g_motorsParam.m4_wAcc = Encoder_getAngularSpeed(ENC_4);
    		g_motorsParam.m4_voltage = PID_controller(MOTOR_4, g_motorsParam.m4_wRef, g_motorsParam.m4_wAcc);
    		setM4Voltage(g_motorsParam.m4_voltage);
    	}*/
    	break;
    }
    return 0 ;
}
