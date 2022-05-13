/**
	\file
	\brief
		This is the source file for the GPIO device driver for Kinetis K64.
		It contains all the implementation for configuration functions, runtime functions and interrupts.
		i.e., this is the application programming interface (API) for the GPIO peripheral.
	\author Yadira Vanessa Bonifacio Benavides ie71445@iteso.mx
 */
#include "MK64F12.h"
#include "GPIO.h"
#include "Bits.h"

static void (*gpio_A_callback)(void) = 0;
static void (*gpio_B_callback)(void) = 0;
static void (*gpio_C_callback)(void) = 0;
static void (*gpio_D_callback)(void) = 0;
static void (*gpio_E_callback)(void) = 0;

static gpio_interrupt_flags_t g_intr_status_flag = {0};

void GPIO_callback_init(gpio_port_name_t port_name,void (*handler)(void))
{
	switch(port_name)
	{
		case GPIO_A: /** GPIO A is selected*/
			gpio_A_callback = handler;
			break;
		case GPIO_B: /** GPIO B is selected*/
			gpio_B_callback = handler;
			break;
		case GPIO_C: /** GPIO C is selected*/
			gpio_C_callback = handler;
			break;
		case GPIO_D: /** GPIO D is selected*/
			gpio_D_callback = handler;
			break;
		case GPIO_E: /** GPIO E is selected*/
			gpio_E_callback = handler;
			break;
	}
}

void PORTA_IRQHandler(void)
{
	g_intr_status_flag.flag_port_a = TRUE;

	if(gpio_A_callback)
	{
		gpio_A_callback();
	}

	GPIO_clear_interrupt(GPIO_A);
}

void PORTB_IRQHandler(void)
{
	g_intr_status_flag.flag_port_b = TRUE;

	if(gpio_B_callback)
	{
		gpio_B_callback();
	}

	GPIO_clear_interrupt(GPIO_B);
}

void PORTC_IRQHandler(void)
{
	g_intr_status_flag.flag_port_c = TRUE;

	if(gpio_C_callback)
	{
		gpio_C_callback();
	}

	GPIO_clear_interrupt(GPIO_C);
}

void PORTD_IRQHandler(void)
{
	g_intr_status_flag.flag_port_d= TRUE;

	if(gpio_D_callback)
	{
		gpio_D_callback();
	}

	GPIO_clear_interrupt(GPIO_D);
}

void PORTE_IRQHandler(void)
{
	g_intr_status_flag.flag_port_e = TRUE;

	if(gpio_E_callback)
	{
		gpio_E_callback();
	}

	GPIO_clear_interrupt(GPIO_E);
}

void GPIO_clear_irq_status(gpio_port_name_t gpio)
{
	switch(gpio)
	{
		case GPIO_A: /** GPIO A is selected*/
			g_intr_status_flag.flag_port_a = FALSE;
			break;
		case GPIO_B: /** GPIO B is selected*/
			g_intr_status_flag.flag_port_b = FALSE;
			break;
		case GPIO_C: /** GPIO C is selected*/
			g_intr_status_flag.flag_port_c = FALSE;
			break;
		case GPIO_D: /** GPIO D is selected*/
			g_intr_status_flag.flag_port_d = FALSE;
			break;
		case GPIO_E: /** GPIO E is selected*/
			g_intr_status_flag.flag_port_e = FALSE;
			break;
	}
}

uint8_t GPIO_get_irq_status(gpio_port_name_t gpio)
{
	uint8_t status = 0;

	switch(gpio)
	{
		case GPIO_A: /** GPIO A is selected*/
			status = g_intr_status_flag.flag_port_a;
			break;
		case GPIO_B: /** GPIO B is selected*/
			status = g_intr_status_flag.flag_port_b;
			break;
		case GPIO_C: /** GPIO C is selected*/
			status = g_intr_status_flag.flag_port_c;
			break;
		case GPIO_D: /** GPIO D is selected*/
			status = g_intr_status_flag.flag_port_d;
			break;
		case GPIO_E: /** GPIO E is selected*/
			status = g_intr_status_flag.flag_port_e;
			break;
	}
	return(status);
}

void GPIO_clear_interrupt(gpio_port_name_t port_name)
{
	switch(port_name)/** Selecting the GPIO for clock enabling*/
	{
		case GPIO_A: /** GPIO A is selected*/
			PORTA->ISFR=0xFFFFFFFF;
			break;
		case GPIO_B: /** GPIO B is selected*/
			PORTB->ISFR=0xFFFFFFFF;
			break;
		case GPIO_C: /** GPIO C is selected*/
			PORTC->ISFR = 0xFFFFFFFF;
			break;
		case GPIO_D: /** GPIO D is selected*/
			PORTD->ISFR=0xFFFFFFFF;
			break;
		default: /** GPIO E is selected*/
			PORTE->ISFR=0xFFFFFFFF;
			break;
	}// end switch
}

uint8_t GPIO_clock_gating(gpio_port_name_t port_name)
{
	switch(port_name)/** Selecting the GPIO for clock enabling*/
	{
		case GPIO_A: /** GPIO A is selected*/
			SIM->SCGC5 |= GPIO_CLOCK_GATING_PORTA; /** Bit 9 of SIM_SCGC5 is  set*/
			break;
		case GPIO_B: /** GPIO B is selected*/
			SIM->SCGC5 |= GPIO_CLOCK_GATING_PORTB; /** Bit 10 of SIM_SCGC5 is set*/
			break;
		case GPIO_C: /** GPIO C is selected*/
			SIM->SCGC5 |= GPIO_CLOCK_GATING_PORTC; /** Bit 11 of SIM_SCGC5 is set*/
			break;
		case GPIO_D: /** GPIO D is selected*/
			SIM->SCGC5 |= GPIO_CLOCK_GATING_PORTD; /** Bit 12 of SIM_SCGC5 is set*/
			break;
		case GPIO_E: /** GPIO E is selected*/
			SIM->SCGC5 |= GPIO_CLOCK_GATING_PORTE; /** Bit 13 of SIM_SCGC5 is set*/
			break;
		default: /**If doesn't exist the option*/
			return(FALSE);
	}// end switch
	/**Successful configuration*/
	return(TRUE);
}// end function

uint8_t GPIO_pin_control_register(gpio_port_name_t port_name, uint8_t pin, gpio_pin_control_register_t* pinControlRegister)
{
	switch(port_name)
		{
		case GPIO_A:/** GPIO A is selected*/
			PORTA->PCR[pin] = *pinControlRegister;
			break;
		case GPIO_B:/** GPIO B is selected*/
			PORTB->PCR[pin] = *pinControlRegister;
			break;
		case GPIO_C:/** GPIO C is selected*/
			PORTC->PCR[pin] = *pinControlRegister;
			break;
		case GPIO_D:/** GPIO D is selected*/
			PORTD->PCR[pin] = *pinControlRegister;
			break;
		case GPIO_E: /** GPIO E is selected*/
			PORTE->PCR[pin] = *pinControlRegister;
			break;
		default:/**If doesn't exist the option*/
			return(FALSE);
		break;
		}
	/**Successful configuration*/
	return(TRUE);
}

void GPIO_write_port(gpio_port_name_t port_name, uint32_t data)
{
	switch(port_name)
	{
		case GPIO_A:/** GPIO A is selected*/
			GPIOA->PSOR = data;
			break;
		case GPIO_B:/** GPIO B is selected*/
			GPIOB->PSOR = data;
			break;
		case GPIO_C:/** GPIO C is selected*/
			GPIOC->PSOR = data;
			break;
		case GPIO_D:/** GPIO D is selected*/
			GPIOD->PSOR = data;
			break;
		case GPIO_E: /** GPIO E is selected*/
			GPIOE->PSOR = data;
			break;
	}
}

uint32_t GPIO_read_port(gpio_port_name_t port_name)
{
	uint32_t data;
	switch(port_name)/** Selecting the port*/
	{
		case GPIO_A:/** GPIO A is selected*/
			data = GPIOA->PDIR;
			break;
		case GPIO_B:/** GPIO B is selected*/
			data = GPIOB->PDIR;
			break;
		case GPIO_C:/** GPIO C is selected*/
			data = GPIOC->PDIR;
			break;
		case GPIO_D:/** GPIO D is selected*/
			data = GPIOD->PDIR;
			break;
		case GPIO_E: /** GPIO E is selected*/
			data = GPIOE->PDIR;
	}
	/** Return data from the port*/
	return data;
}

uint8_t GPIO_read_pin(gpio_port_name_t port_name, uint8_t pin)
{
	uint8_t pin_value = 0;

	switch(port_name)
	{
		case GPIO_A:/**GPIO A is selected*/
			pin_value = GPIOA->PDIR & (1 << pin);
			break;
		case GPIO_B:/**GPIO B is selected*/
			pin_value = GPIOB->PDIR & (1 << pin);
			break;
		case GPIO_C:/**GPIO C is selected*/
			pin_value = GPIOC->PDIR & (1 << pin);
			break;
		case GPIO_D:/**GPIO D is selected*/
			pin_value = GPIOD->PDIR & (1 << pin);
			break;
		case GPIO_E: /**GPIO E is selected*/
			pin_value = GPIOE->PDIR & (1 << pin);
			break;
	}

	return(pin_value >> pin);
}

void GPIO_set_pin(gpio_port_name_t port_name, uint8_t pin)
{
	switch(port_name)/** Selecting the port*/
	{
		case GPIO_A:/** GPIO A is selected*/
			GPIOA->PSOR = 1 << pin;
			break;
		case GPIO_B:/** GPIO B is selected*/
			GPIOB->PSOR = 1 << pin;
			break;
		case GPIO_C:/** GPIO C is selected*/
			GPIOC->PSOR = 1 << pin;
			break;
		case GPIO_D:/** GPIO D is selected*/
			GPIOD->PSOR = 1 << pin;
			break;
		case GPIO_E: /** GPIO E is selected*/
			GPIOE->PSOR = 1 << pin;
	}
}

void GPIO_clear_pin(gpio_port_name_t port_name, uint8_t pin)
{
	switch(port_name)/** Selecting the port*/
	{
		case GPIO_A:/** GPIO A is selected*/
			GPIOA->PCOR |= 1 << pin;
			break;
		case GPIO_B:/** GPIO B is selected*/
			GPIOB->PCOR |= 1 << pin;
			break;
		case GPIO_C:/** GPIO C is selected*/
			GPIOC->PCOR |= 1 << pin;
			break;
		case GPIO_D:/** GPIO D is selected*/
			GPIOD->PCOR |= 1 << pin;
			break;
		case GPIO_E: /** GPIO E is selected*/
			GPIOE->PCOR |= 1 << pin;
	}
}

void GPIO_toogle_pin(gpio_port_name_t port_name, uint8_t pin)
{
	switch(port_name)/** Selecting the port*/
	{
		case GPIO_A:/** GPIO A is selected*/
			GPIOA->PTOR = 1 << pin;
			break;
		case GPIO_B:/** GPIO B is selected*/
			GPIOB->PTOR = 1 << pin;
			break;
		case GPIO_C:/** GPIO C is selected*/
			GPIOC->PTOR = 1 << pin;
			break;
		case GPIO_D:/** GPIO D is selected*/
			GPIOD->PTOR = 1 << pin;
			break;
		case GPIO_E: /** GPIO E is selected*/
			GPIOE->PTOR = 1 << pin;
	}
}

void GPIO_data_direction_port(gpio_port_name_t port_name ,uint32_t direction)
{
	switch(port_name)/** Selecting the port*/
	{
		case GPIO_A:/** GPIO A is selected*/
			GPIOA->PDDR = direction;
			break;
		case GPIO_B:/** GPIO B is selected*/
			GPIOB->PDDR = direction;
			break;
		case GPIO_C:/** GPIO C is selected*/
			GPIOC->PDDR = direction;
			break;
		case GPIO_D:/** GPIO D is selected*/
			GPIOD->PDDR = direction;
			break;
		case GPIO_E: /** GPIO E is selected*/
			GPIOE->PDDR = direction;
	}
}

void GPIO_data_direction_pin(gpio_port_name_t port_name, uint8_t state, uint8_t pin)
{
	switch(port_name)
	{
		case GPIO_A:/** GPIO A is selected*/
			GPIOA->PDDR &= ~(TRUE << pin);
			GPIOA->PDDR |= state << pin;
			break;
		case GPIO_B:/** GPIO B is selected*/
			GPIOB->PDDR &= ~(TRUE << pin);
			GPIOB->PDDR |= state << pin;
			break;
		case GPIO_C:/** GPIO C is selected*/
			GPIOC->PDDR &= ~(TRUE << pin);
			GPIOC->PDDR |= state << pin;
			break;
		case GPIO_D:/** GPIO D is selected*/
			GPIOD->PDDR &= ~(TRUE << pin);
			GPIOD->PDDR |= state << pin;
			break;
		case GPIO_E: /** GPIO E is selected*/
			GPIOE->PDDR &= ~(TRUE << pin);
			GPIOE->PDDR |= state << pin;
			break;
	}
}



