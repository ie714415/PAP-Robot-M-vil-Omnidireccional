/**
	\file
	\brief
		This is the source file for the FlexTimer device driver for Kinetis K64.
		It contains some of the implementation for configuration functions.
		i.e., this is the application programming interface (API) for the FlexTimer.
	\authors Sofia Arias ie708531@iteso.mx, Yadira Bonifacio ie71445@iteso.mx
 */
#ifndef FLEXTIMER_H_
#define FLEXTIMER_H_

#include "MK64F12.h"

/**Constants of clock source values uses in FlexTimer.*/
#define FLEX_TIMER_CLKS_0  (0U)
#define FLEX_TIMER_CLKS_1  (1U)
#define FLEX_TIMER_CLKS_2  (2U)
#define FLEX_TIMER_CLKS_3  (3U)
/**Constants that represents the different values for the prescaler*/
#define FLEX_TIMER_PS_1    (0U)
#define FLEX_TIMER_PS_2    (1U)
#define FLEX_TIMER_PS_4    (2U)
#define FLEX_TIMER_PS_8    (3U)
#define FLEX_TIMER_PS_16    (4U)
#define FLEX_TIMER_PS_32    (5U)
#define FLEX_TIMER_PS_64    (6U)
#define FLEX_TIMER_PS_128    (7U)
/**Constants that represents the different status for the mode*/
#define  FLEX_TIMER_DMA   0x01
#define  FLEX_TIMER_ELSA  0x04
#define  FLEX_TIMER_ELSB  0x08
#define  FLEX_TIMER_MSA   0x10
#define  FLEX_TIMER_MSB   0x20
#define  FLEX_TIMER_CHIE  0x40
#define  FLEX_TIMER_CHF   0x80
/**Constant that represents 100% duty cycle*/
#define DUTYCYCLE100 0x0FF
//#define PORCENTAGE100 100

/*! These constants are used to select an specific channel*/
typedef enum {channel_0, channel_1, channel_2, channel_3}FlexTimer_channel_t;
/*! These constants are used to select an specific mode*/
typedef enum {TOOGLE, /*!< Definition to select Toggle Output on match*/
			  SET, 	  /*!< Definition to select Set Output on match*/
			  CLEAR,  /*!< Definition to select Clear Output on match*/
			  HIGH_EA,/*!< Definition to select Edge-Aligned PWM High-true pulses*/
			  LOW_EA  /*!< Definition to select Edge-Aligned PWM Low-true pulses*/
}FlexTimer_mode_t;

/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief	 This function assigns the new value for the duty cycle.

 	 \param[in]  channel_value Value for the new duty cycle.
 	 \param[in]  channel  Channel for the FlexTimer.
 	 \return void
 */
void FlexTimer_update_channel_value(int16_t channel_value, FlexTimer_channel_t channel);
/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief	 This function initialize the FlexTimer.

 	 \param[in]  void
 	 \return void
 */
void FlexTimer_init(void);
/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief	 This function initialize the clock gating of the FlexTimer,

 	 \param[in]  void
 	 \return void
 */
void FlexTimer_clock_gating(void);
/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief	 This function enables writing over all the registers of the FlexTimer.

 	 \param[in]  void
 	 \return void
 */
void FlexTimer_enable(void);
/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief	 This function selects the FlexTimer in BDM.

 	 \param[in]  void
 	 \return void
 */
void FlexTimer_select_behavior(void);
/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief	 This function select the FlexTimer and selects the mode and the channel we are
 	 	 going to use.

 	 \param[in]  mode Mode that FlexTimer is going to work.
 	 \param[in]  channel Channel for the FlexTimer.
 	 \return void
 */
void FlexTimer_mode(FlexTimer_mode_t mode, FlexTimer_channel_t channel);
/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief	 This function assigns the modulo register with a predefined value of the FlexTime.
 	 	 (frequency of the signal)

 	 \param[in]  void
 	 \return void
 */
void FlexTimer_mod(void);
/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief	 This function assigns the start duty cycle in 0 of the FlexTimer.

 	 \param[in]  channel Channel for the FlexTimer.
 	 \return void
 */
void FlexTimer_channel_value(FlexTimer_channel_t channel);
/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief	 This function selects the clock source of the FlexTimer.

 	 \param[in]  void
 	 \return void
 */
void FlexTimer_clock_source(void);

#endif /* FLEXTIMER_H_ */
