/*
===============================================================================
 Name        : 32_Day68_25112021_TIMER003.c

 Description : Program to demonstrate the state machine using button and timer

 TODO        :
 Layered Architecture used for this project
 ************************************
 Application layer-32_Day68_25112021_TIMER003.c
 ************************************
 Board layer -  configboard.h, led.c/.h
 ************************************
 Low level drivers or chip level - pinmux.c/.h,timer.c/.h, gpio.c/.h
 ************************************
 Hardware
 ************************************
===============================================================================
*/

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
/* Private includes ----------------------------------------------------------*/
#include "pinmux.h"
#include "timer.h"
#include "led.h"
#include "buttonint.h"

/* Private typedef -----------------------------------------------------------*/
typedef enum {LEDOFF=0,LEDON,LEDBLINK}states_t;
typedef enum {NONE=0,TIMEOUT,BUTTONPRESS}inputs_t;
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
volatile states_t currentstate =LEDOFF;
volatile inputs_t currentinput =NONE;
/* Private function prototypes -----------------------------------------------*/

/* Private user code ---------------------------------------------------------*/
void vTimeoutHandler(uint32_t interruptsource)
{
		switch (interruptsource) {
		case MR0INT:
			currentinput = TIMEOUT;
			break;
		case MR1INT:
			/* DO NOTHING*/
			break;
		case MR2INT:
			/* DO NOTHING*/
			break;
		case MR3INT:
			/* DO NOTHING*/
			break;
		case CAP0INT:
			/* DO NOTHING*/
			break;
		case CAP1INT:
			/* DO NOTHING*/
			break;
		}
}

void vLedBlinkHandler(uint32_t interruptsource)
{
		switch (interruptsource) {
		case MR0INT:
			vLedToggle(LED_0);
			break;
		case MR1INT:
			/* DO NOTHING*/
			break;
		case MR2INT:
			/* DO NOTHING*/
			break;
		case MR3INT:
			/* DO NOTHING*/
			break;
		case CAP0INT:
			/* DO NOTHING*/
			break;
		case CAP1INT:
			/* DO NOTHING*/
			break;
		}
}

/**
  * @brief  Initialize all the hardware connected
  * @retval none
  */
void vAppHardwareInit(void)
{
	vPinmuxInitialize();
	vLedInitialize();
	vButtonIntInitialize();

	/********************Configuring TIMER0 for Timeout Handler**************/
	/* Basic configurations of Timer */
	vTimerInitialize(TIMER_0);
	/* Configuring the timer in Timer Mode */
	vTimerCountControl(TIMER0,TIMER_MODE);
	/* Setting the prescalar for making TC count every 1ms */
	vTimerPrescalarSel(TIMER0,PRESCALAR_MS);
	/* Load the respective match registers with required delay in ms */
	vTimerLoadMatchRegister(TIMER0,MATCH0,10000);
	/* Enabling to Reset the timer when TC matches with MR0 register value */
	vTimerMatchReset(TIMER0,MATCH0);

	/* Call back registration of vTimeoutHandler with Timer IRQ Handler */
	vTimerIntAttachCallback(TIMER_0,vTimeoutHandler);
	/* Enabling interrupt for all match register 0 */
	vTimerInterruptEnable(TIMER0,MATCH0);
	/* Enabling the Timer0 interrupt in NVIC */
	NVIC_EnableIRQ(TIMER0_IRQn);
	/********************************************************************/
	/********************Configuring TIMER1 for Led Blink Handler**************/
	/* Basic configurations of Timer */
	vTimerInitialize(TIMER_1);
	/* Configuring the timer in Timer Mode */
	vTimerCountControl(TIMER1,TIMER_MODE);
	/* Setting the prescalar for making TC count every 1ms */
	vTimerPrescalarSel(TIMER1,PRESCALAR_MS);
	/* Load the respective match registers with required delay in ms */
	vTimerLoadMatchRegister(TIMER1,MATCH0,500);
	/* Enabling to Stop the timer when TC matches with MR0 register value */
	vTimerMatchReset(TIMER1,MATCH0);

	/* Call back registration of vLedBlinkHandler with Timer IRQ Handler */
	vTimerIntAttachCallback(TIMER_1,vLedBlinkHandler);
	/* Enabling interrupt for all match register 0 */
	vTimerInterruptEnable(TIMER1,MATCH0);
	/* Enabling the Timer0 interrupt in NVIC */
	NVIC_EnableIRQ(TIMER1_IRQn);
	/********************************************************************/
}

/**
  * @brief  Button interrupt Handler
  * @retval none
  */
void vButtonIntHandler(void)
{
		currentinput = BUTTONPRESS;

}

/**
  * @brief  Starting the timer for Timeout input
  * @retval none
  */
void vTimeoutTimerStart(void)
{
	/* Resetting the Timer */
    vTimerControlReset(TIMER0);
	/* Enabling the timer and counting starts */
	vTimerControl(TIMER0,TIMER_COUNTER_ENABLE);
}
/**
  * @brief  Stopping the timer of Timeout
  * @retval none
  */
void vTimeoutTimerStop(void)
{
	/* Resetting the Timer */
	vTimerControlReset(TIMER0);
	/* Disabling the timer and counting starts */
	vTimerControl(TIMER0,TIMER_COUNTER_DISABLE);
}


/**
  * @brief  Starting the timer for Led Blinking
  * @retval none
  */
void vLedBlinkTimerStart(void)
{
	/* Resetting the Timer */
	vTimerControlReset(TIMER1);
    /* Enabling the timer and counting starts */
	vTimerControl(TIMER1,TIMER_COUNTER_ENABLE);
}
/**
  * @brief  Starting the timer for Led Blinking
  * @retval none
  */
void vLedBlinkTimerStop(void)
{
	/* Resetting the Timer */
	vTimerControlReset(TIMER1);
    /* Disabling the timer and counting stops */
	vTimerControl(TIMER1,TIMER_COUNTER_DISABLE);
}



/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/

  /* Initialize all configured peripherals */
   vAppHardwareInit();
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)  // for(;;)
  {
	  switch(currentstate)
	  {
	  case LEDOFF:
		  switch(currentinput)
		  {
		  case NONE:
			  /* DO NOTHING */
			  break;
		  case TIMEOUT:
			  /* DO NOTHING */
			  break;
		  case BUTTONPRESS:
			  vLedOn(LED_0);
			  vTimeoutTimerStart();
			  currentstate = LEDON;
			  currentinput = NONE;
			  break;
		  default:
			  /* DO NOTHING */
			  break;
		  }

		  break;
	  case LEDON:
		  switch(currentinput)
			  {
			  case NONE:
				  /* DO NOTHING */
				  break;
			  case TIMEOUT:
				  vLedOff(LED_0);
				  currentstate = LEDOFF;
				  currentinput = NONE;
				  break;
			  case BUTTONPRESS:
				  vTimeoutTimerStart();
				  vLedBlinkTimerStart();
				  currentstate = LEDBLINK;
				  currentinput = NONE;
				  break;
			  default:
				  /* DO NOTHING */
				  break;
			  }

		  break;
	  case LEDBLINK:
		  switch(currentinput)
			  {
			  case NONE:
				  /* DO NOTHING */
				  break;
			  case TIMEOUT:
				  vLedBlinkTimerStop();
  				  vLedOff(LED_0);
  				  currentstate=LEDOFF;
  				  currentinput = NONE;
				  break;
			  case BUTTONPRESS:
				  vTimeoutTimerStop();
				  vLedBlinkTimerStop();
				  vLedOff(LED_0);
				  currentstate=LEDOFF;
				  currentinput = NONE;
				  break;
			  default:
				  /* DO NOTHING */
				  break;
			  }

		  break;
	  default:
		  break;
	  }
  }
  /* End of Application entry point */
}



