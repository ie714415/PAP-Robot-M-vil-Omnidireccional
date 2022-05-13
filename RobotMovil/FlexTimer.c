/**
	\file
	\brief
		This is the source file for the FlexTimer device driver for Kinetis K64.
		It contains some of the implementation for configuration functions.
		i.e., this is the application programming interface (API) for the FlexTimer.
	\authors Sofia Arias ie708531@iteso.mx, Yadira Bonifacio ie71445@iteso.mx
 */
#include "FlexTimer.h"
#include "MK64F12.h"

void FlexTimer_update_channel_value(int16_t channel_value, FlexTimer_channel_t channel)
{
	/**Assigns a new value for the duty cycle*/
	FTM0->CONTROLS[channel].CnV = channel_value;
}

void FlexTimer_init()
{
	/**Clock gating for FlexTimer*/
	FlexTimer_clock_gating();
	/**When write protection is enabled (WPDIS = 0), write protected bits cannot be written.
	* When write protection is disabled (WPDIS = 1), write protected bits can be written.*/
	FlexTimer_enable();
	/**Assigning a default value for modulo register*/
	FlexTimer_mod();
	/**Selects the Edge-Aligned PWM mode mode*/
	FlexTimer_mode(HIGH_EA, channel_0);
	FlexTimer_mode(HIGH_EA, channel_1);
	FlexTimer_mode(HIGH_EA, channel_2);
	FlexTimer_mode(HIGH_EA, channel_3);

	/**Assign a duty cycle of 0%*/
	FlexTimer_channel_value(channel_0);
	FlexTimer_channel_value(channel_1);
	FlexTimer_channel_value(channel_2);
	FlexTimer_channel_value(channel_3);
	/**Configure the times*/
	FlexTimer_clock_source();

}

void FlexTimer_clock_gating(void)
{
	/**Clock gating for FlexTimer 0*/
	SIM->SCGC6 |= SIM_SCGC6_FTM0_MASK;
}

void FlexTimer_enable(void)
{
	/**When write protection is enabled or disabled*/
	FTM0->MODE |= FTM_MODE_WPDIS_MASK;
	/**Enables the writing over all registers*/
	FTM0->MODE &= ~ FTM_MODE_FTMEN_MASK;
}

void FlexTimer_select_behavior(void)
{
	/**Selects the FTM behavior in BDM mode. In this case in functional mode*/
	FTM0->CONF |= FTM_CONF_BDMMODE(3);

}

void FlexTimer_mode(FlexTimer_mode_t mode, FlexTimer_channel_t channel)
{
	switch(mode)
	{
		case TOOGLE:
			/**Select FlexTimer in output compare in toggle mode*/
			FTM0->CONTROLS[channel].CnSC = FTM_CnSC_MSA(1) | FTM_CnSC_ELSA(1);
			break;
		case SET:
			/**Select FlexTimer in output compare in set mode*/
			FTM0->CONTROLS[channel].CnSC = FTM_CnSC_MSA(1) | FTM_CnSC_ELSA(1) | FTM_CnSC_ELSB(1);
			break;
		case CLEAR:
			/**Select FlexTimer in output compare in clear mode*/
			FTM0->CONTROLS[channel].CnSC = FTM_CnSC_MSA(1) | FTM_CnSC_ELSB(1);
			break;
		case HIGH_EA:
			/**Selects FlexTimer in Edge-Aligned PWM in High-true pulses mode*/
			FTM0->CONTROLS[channel].CnSC = FTM_CnSC_MSB(1) | FTM_CnSC_ELSB(1);
			break;
		case LOW_EA:
			/**Selects FlexTimer in Edge-Aligned PWM in Low-true pulses mode*/
			FTM0->CONTROLS[channel].CnSC = FTM_CnSC_MSB(1) | FTM_CnSC_ELSA(1);
			break;
	}
}

void FlexTimer_mod(void)
{
	/**Assign modulo register with a predefined value*/
	FTM0->MOD = DUTYCYCLE100;
}

void FlexTimer_channel_value(FlexTimer_channel_t channel)
{
	/**Assign a duty cycle of 0%*/
	FTM0->CONTROLS[channel].CnV = 0;

}

void FlexTimer_clock_source(void)
{
	/**Select clock source and prescaler*/
	FTM0->SC |= FTM_SC_CLKS (FLEX_TIMER_CLKS_1)| FTM_SC_PS(FLEX_TIMER_PS_8);
}


