/**
	\file
	\brief
		This is the source file for the ADC driver for Kinetis K64.
		It contains some of the implementation for configuration functions.
		i.e., this is the application programming interface (API) for the ADC.
	\authors Sofia Arias ie708531@iteso.mx, Yadira Bonifacio ie71445@iteso.mx
 */
#ifndef ADC_H_
#define ADC_H_

#include <stdint.h>

/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief	 This function set the clock gating for the ADC 0.

 	 \param[in]  void
 	 \return void
 */
void ADC_clock_gating(void);
/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief	 This function set the configuration of the register.

 	 \param[in]  void
 	 \return void
 */
void ADC_configuration_register(void);
/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief	 This function enable the continuous conversions.

 	 \param[in]  void
 	 \return void
 */
void ADC_status_and_control(void);
/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief	 This function initialize the ADC 0.

 	 \param[in]  void
 	 \return void
 */
void ADC_init(void);
/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief	 This function make the conversions of the signal we obtain from the hardware and
 	 	 converts it into a software number.

 	 \param[in]  void
 	 \return uint16_t
 */
uint8_t ADC_result(void);

#endif /* ADC_H_ */
