/**
	\file
	\brief
		This is the source file for the ADC driver for Kinetis K64.
		It contains some of the implementation for configuration functions.
		i.e., this is the application programming interface (API) for the ADC.
	\authors Sofia Arias ie708531@iteso.mx, Yadira Bonifacio ie71445@iteso.mx
 */
#include "MK64F12.h"
#include "GPIO.h"
#include "Bits.h"
#include "ADC.h"

void ADC_clock_gating(void)
{
	/** Bit 27 of SIM_SCGC6 is  set*/
	SIM->SCGC6 |= SIM_SCGC6_ADC0_MASK;
}

void ADC_configuration_register(void)
{
	/**ADxxb channels are selected*/
	ADC0->CFG1 = ADC_CFG1_ADIV(3) | ADC_CFG1_ADLSMP_MASK | ADC_CFG1_MODE(0) | ADC_CFG1_ADICLK(0);
	ADC0->CFG2 = FALSE;
}

void ADC_status_and_control(void)
{
	ADC0->SC2 = FALSE;
	/**Continuous conversions is enabled*/
	ADC0->SC3 = ADC_SC3_AVGE_MASK | ADC_SC3_AVGS(0);
}

void ADC_init(void)
{
	/**Activating the ADC clock gating*/
	ADC_clock_gating();
	/**Changes the ADC mux setting*/
	ADC_configuration_register();
	/**Enables continuous conversions*/
	ADC_status_and_control();
}

uint8_t ADC_result(void)
{
	uint16_t adc_result;
	ADC0->SC1[0] = ADC_SC1_ADCH(12);
	while ((ADC0->SC1[0] & ADC_SC1_COCO_MASK) == 0);
	adc_result = ADC0->R[0];;
	return (adc_result);
}

