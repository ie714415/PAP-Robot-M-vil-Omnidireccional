#include "MK64F12.h"
#include "PIT.h"
#include "Bits.h"

static void (*pit_0_callback)(void) = 0;
static void (*pit_1_callback)(void) = 0;
static void (*pit_2_callback)(void) = 0;
static void (*pit_3_callback)(void) = 0;

/** Global static variable that represents the interrupt activation flag */
static pit_interrupt_flags_t g_interrupt_flags_t = {FALSE};

volatile uint8_t g_dummyRead = FALSE;

void PIT_callback_init(PIT_timer_t pit_timer, void (*handler)(void))
{
	switch(pit_timer)
	{
		case PIT_0: /** PIT 0 is selected*/
			pit_0_callback = handler;
			break;
		case PIT_1: /** PIT 1 is selected*/
			pit_1_callback = handler;
			break;
		case PIT_2: /** PIT 2 is selected*/
			pit_2_callback = handler;
			break;
		case PIT_3: /** PIT 3 is selected*/
			pit_3_callback = handler;
			break;
	}
}

void PIT0_IRQHandler(void)
{
	g_interrupt_flags_t.flag_pit_0 = TRUE;

	if(pit_0_callback)
	{
		pit_0_callback();
	}

	PIT->CHANNEL[PIT_0].TFLG |= PIT_TFLG_TIF_MASK;
	g_dummyRead = PIT->CHANNEL[PIT_0].TCTRL; //read control register for clear PIT flag, this is silicon bug
}

void PIT1_IRQHandler(void)
{
	g_interrupt_flags_t.flag_pit_1 = TRUE;

	if(pit_1_callback)
	{
		pit_1_callback();
	}

	PIT->CHANNEL[PIT_1].TFLG |= PIT_TFLG_TIF_MASK;
	g_dummyRead = PIT->CHANNEL[PIT_1].TCTRL; //read control register for clear PIT flag, this is silicon bug
}

void PIT2_IRQHandler(void)
{
	g_interrupt_flags_t.flag_pit_2 = TRUE;

	if(pit_2_callback)
	{
		pit_2_callback();
	}

	PIT->CHANNEL[PIT_2].TFLG |= PIT_TFLG_TIF_MASK;
	g_dummyRead = PIT->CHANNEL[PIT_2].TCTRL; //read control register for clear PIT flag, this is silicon bug
}

void PIT3_IRQHandler(void)
{
	g_interrupt_flags_t.flag_pit_3 = TRUE;

	if(pit_3_callback)
	{
		pit_3_callback();
	}

	PIT->CHANNEL[PIT_3].TFLG |= PIT_TFLG_TIF_MASK;
	g_dummyRead = PIT->CHANNEL[PIT_3].TCTRL; //read control register for clear PIT flag, this is silicon bug
}

void PIT_delay(PIT_timer_t pit_timer, uint32_t system_clock, My_float_pit_t delay)
{
	My_float_pit_t pit_clock = system_clock / 2; //PIT operation frequency

	uint32_t vald_delay = (uint32_t)(system_clock*delay);
	/** Indicate the value of the delay*/
	PIT->CHANNEL[pit_timer].LDVAL = vald_delay;
	/** PIT #  interrupt is enabled*/
	PIT_enable_interrupt(pit_timer);
	/**Timer Enable of PIT # is enabled*/
	PIT_start(pit_timer);
}

void PIT_clock_gating(void)
{
	/** Bit 23 of SIM_SCGC6 is  set*/
	SIM->SCGC6 |= PIT_CLOCK_GATING;
}

uint8_t PIT_get_interrupt_flag_status(PIT_timer_t pit_timer)
{
	uint8_t status = FALSE;

	switch(pit_timer)
	{
		case PIT_0: /** PIT 0 is selected*/
			status = g_interrupt_flags_t.flag_pit_0;
			break;
		case PIT_1: /** PIT 1 is selected*/
			status = g_interrupt_flags_t.flag_pit_1;
			break;
		case PIT_2: /** PIT 2 is selected*/
			status = g_interrupt_flags_t.flag_pit_2;
			break;
		case PIT_3: /** PIT 3 is selected*/
			status = g_interrupt_flags_t.flag_pit_3;
			break;
	}
	return status;
}

void PIT_clear_interrupt_flag(PIT_timer_t pit_timer)
{
	/**Clean the programmer's flag (flag = 0)*/
	switch(pit_timer)
	{
		case PIT_0: /** PIT 0 is selected*/
			g_interrupt_flags_t.flag_pit_0 = FALSE;
			break;
		case PIT_1: /** PIT 1 is selected*/
			g_interrupt_flags_t.flag_pit_1 = FALSE;
			break;
		case PIT_2: /** PIT 2 is selected*/
			g_interrupt_flags_t.flag_pit_2 = FALSE;
			break;
		case PIT_3: /** PIT 3 is selected*/
			g_interrupt_flags_t.flag_pit_3 = FALSE;
			break;
	}
}

void PIT_enable(void)
{
	/**Timer activated debug mode*/
	PIT->MCR |= PIT_MCR_FRZ_MASK;
	/**Enable the PIT timer clocks*/
	PIT->MCR &= ~(PIT_MCR_MDIS_MASK);
}

void PIT_enable_interrupt(PIT_timer_t pit)
{
	/**Timer Interrupt Enable of PIT # is enabled*/
	PIT->CHANNEL[pit].TCTRL |= PIT_TCTRL_TIE_MASK;
}

void PIT_stop(uint8_t channel)
{
	/**Timer Enable of PIT_# is disabled*/
	PIT->CHANNEL[channel].TCTRL = FALSE;
}

void PIT_start(uint8_t channel)
{
	/**Timer Enable of PIT_# is enabled*/
	PIT->CHANNEL[channel].TCTRL |= PIT_TCTRL_TEN_MASK;
}

