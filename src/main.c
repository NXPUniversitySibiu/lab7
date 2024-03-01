/*
 * Copyright 2020 NXP
 * All rights reserved.
 *
 * NXP Confidential. This software is owned or controlled by NXP and may only be
 * used strictly in accordance with the applicable license terms. By expressly
 * accepting such terms or by downloading, installing, activating and/or otherwise
 * using the software, you are agreeing that you have read, and that you agree to
 * comply with and are bound by, such license terms. If you do not agree to be
 * bound by the applicable license terms, then you may not retain, install,
 * activate or otherwise use the software. The production use license in
 * Section 2.3 is expressly granted for this software.
 */


#include "sdk_project_config.h"
#include "osif.h"
#include <stdio.h>
#include <string.h>


/********* DEFINE *******************/
#define PCC_CLOCK	PCC_PORTD_CLOCK

#define LED0_PORT PTD
#define LED0_PIN  15
#define LED1_PORT PTD
#define LED1_PIN  16

#define BTN_GPIO        PTC
#define BTN1_PIN        13U
#define BTN2_PIN        12U
#define BTN_PORT        PORTC
#define BTN_PORT_IRQn   PORTC_IRQn

#define ADC_INSTANCE    0UL
#define ADC_CHN         ADC_INPUTCHAN_EXT12
#define ADC_VREFH       5.0f
#define ADC_VREFL       0.0f

#define PTE0 0

//#define LED_RED_TOGGLE

/* Timeout in ms for blocking operations on LPUART */
#define TIMEOUT     500U


/*command for increase/decrease brightness */
typedef enum
{
    LED_BRIGHTNESS_INCREASE_REQUESTED = 0x00U,
	LED_BRIGHTNESS_DECREASE_REQUESTED = 0x01U
} btn_commands_list;


/* ******** VARIABLES ***************** */
volatile int exit_code = 0;

/* End command for Nextion display. Every command should end with this */
uint8_t Cmd_End[3]={0xFF,0xFF,0xFF};

/* store led request button value */
uint8_t ledRequested = (uint8_t)LED_BRIGHTNESS_INCREASE_REQUESTED;

/* LED PWM duty cycle */
int Led_Red_dutyCycle = 0U;
int Led_Green_dutyCycle = 0U;
int Led_Blue_dutyCycle = 0U;

/* ECU identification */
bool isHMI=false;

/*  ***** FUNCTIONS  ******** */

/** * Button interrupt handler - used to increase/ decrease duty for GREEN LED */
void buttonISR(void)
{
    /* Check if one of the buttons (SW3/SW2) was pressed */
    uint32_t buttonsPressed = PINS_DRV_GetPortIntFlag(BTN_PORT) &
                                           ((1 << BTN1_PIN) | (1 << BTN2_PIN));


    if(buttonsPressed != 0)
    {
        /* Set LED PWM duty value according to the button pressed */
        switch (buttonsPressed)
        {
            case (1 << BTN1_PIN):
                ledRequested = LED_BRIGHTNESS_INCREASE_REQUESTED;
                Led_Blue_dutyCycle=Led_Blue_dutyCycle+100;
                if(Led_Blue_dutyCycle>32768)
                {
                	Led_Blue_dutyCycle=32760;
                }
                /* Clear interrupt flag */
                PINS_DRV_ClearPinIntFlagCmd(BTN_PORT, BTN1_PIN);
                break;
            case (1 << BTN2_PIN):
                ledRequested = LED_BRIGHTNESS_DECREASE_REQUESTED;
                Led_Blue_dutyCycle=Led_Blue_dutyCycle-100;
                if(Led_Blue_dutyCycle<0)
                {
                	Led_Blue_dutyCycle=0;
                }
                /* Clear interrupt flag */
                PINS_DRV_ClearPinIntFlagCmd(BTN_PORT, BTN2_PIN);
                break;
            default:
                PINS_DRV_ClearPortIntFlagCmd(BTN_PORT);
                break;
        }
    }
}


/* Update the gauge on display with value sent on UART  */
void UpdateGauge(char *obj, uint16_t value)
{
	char buf[30];

	/*minimum value fr position 0 */
	value+=315;

	/*scaled the value upper 0  */
	if(value>=360)
		{
			value=value-360;
		}
	int len=sprintf(buf, "%s=%u", obj,value);

	/*  send value of gauge (z0.value) on UART    */
	LPUART_DRV_SendDataBlocking(INST_LPUART_LPUART_1, (uint8_t *)buf, len, TIMEOUT);
	//delay(10U);
	/* send the end command to display  */
	LPUART_DRV_SendDataBlocking(INST_LPUART_LPUART_1, (uint8_t *)Cmd_End, 3, TIMEOUT);

}

/* set delay in cycles */
void delay(volatile int cycles)
{
    /* Delay function - do nothing for a number of cycles */
    while(cycles--);
}

/* main function */
int main(void)
{
	/* Write your local variable definition here */
    ftm_state_t ftmStateStruct;

	/* Write your local variable definition here */

	/* Variables in which we store data from ADC */
	uint16_t adcRawValue;
	uint16_t adcMax;
	uint16_t adcValue;


  /* Initialize clock module */  
  CLOCK_SYS_Init(g_clockManConfigsArr, CLOCK_MANAGER_CONFIG_CNT, g_clockManCallbacksArr, CLOCK_MANAGER_CALLBACK_CNT);
  CLOCK_SYS_UpdateConfiguration(0U, CLOCK_MANAGER_POLICY_AGREEMENT);

  /* Initialize pins
   *    -    See PinSettings component for more info
   */
  PINS_DRV_Init(NUM_OF_CONFIGURED_PINS0, g_pin_mux_InitConfigArr0);

  /* Get ADC max value from the resolution */
  	if (ADC_0_ConvConfig0.resolution == ADC_RESOLUTION_8BIT)
  		adcMax = (uint16_t) (1 << 8);
  	else if (ADC_0_ConvConfig0.resolution == ADC_RESOLUTION_10BIT)
  		adcMax = (uint16_t) (1 << 10);
  	else
  		adcMax = (uint16_t) (1 << 12);

	/* Configure and calibrate the ADC converter
	 *  -   See ADC component for the configuration details
	 */
	DEV_ASSERT(ADC_0_ChnConfig0.channel == ADC_CHN);

	ADC_DRV_ConfigConverter(ADC_INSTANCE, &ADC_0_ConvConfig0);
	ADC_DRV_AutoCalibration(ADC_INSTANCE);

    /* Initialize LPUART instance */
    LPUART_DRV_Init(INST_LPUART_LPUART_1, &lpUartState1, &lpUartInitConfig1);

    /* Initialize FTM instance */
    FTM_DRV_Init(INST_FLEXTIMER_PWM_1, &flexTimer_pwm_1_InitConfig, &ftmStateStruct);

    /* Initialize FTM PWM */
    FTM_DRV_InitPwm(INST_FLEXTIMER_PWM_1, &flexTimer_pwm_1_PwmConfig);

    /* Setup button pins */
    PINS_DRV_SetPinsDirection(BTN_GPIO, ~((1 << BTN1_PIN)|(1 << BTN2_PIN)));

    /* Setup button pins interrupt */
    PINS_DRV_SetPinIntSel(BTN_PORT, BTN1_PIN, PORT_INT_RISING_EDGE);
    PINS_DRV_SetPinIntSel(BTN_PORT, BTN2_PIN, PORT_INT_RISING_EDGE);

    /* Install buttons ISR */
       INT_SYS_InstallHandler(BTN_PORT_IRQn, &buttonISR, NULL);

       /* Enable buttons interrupt */
       INT_SYS_EnableIRQ(BTN_PORT_IRQn);


/* check if ECU is HMI  */
       if (PTE->PDIR & (1<<PTE0))
       {
    	   isHMI=true;
       }
       else
       {
    	   isHMI=false;
       }
       /*set brightness to 0 on LED_RED */
	    FTM_DRV_UpdatePwmChannel(INST_FLEXTIMER_PWM_1,
	                                         flexTimer_pwm_1_IndependentChannelsConfig[0].hwChannelId,
	                                         FTM_PWM_UPDATE_IN_TICKS, (uint16_t)0,
	                                         0U,
	                                         true);

    /* Infinite loop */
while (1)
{
	/* Configure ADC channel and software trigger a conversion */
	ADC_DRV_ConfigChan(ADC_INSTANCE, 0U, &ADC_0_ChnConfig0);

	/* set the LED GREEN ON at mid brightness*/
	FTM_DRV_UpdatePwmChannel(INST_FLEXTIMER_PWM_1,
	                             flexTimer_pwm_1_IndependentChannelsConfig[1].hwChannelId,
	                             FTM_PWM_UPDATE_IN_TICKS, (uint16_t)Led_Green_dutyCycle,
	                             0U,
	                             true);

	/* set the LED GREEN ON at minimum brightness*/
	FTM_DRV_UpdatePwmChannel(INST_FLEXTIMER_PWM_1,
	                             flexTimer_pwm_1_IndependentChannelsConfig[2].hwChannelId,
	                             FTM_PWM_UPDATE_IN_TICKS, (uint16_t)Led_Blue_dutyCycle,
	                             0U,
	                             true);


  	

#ifdef LED_RED_TOGGLE
	/* Increase the brightness of LED RED*/
    for (Led_Red_dutyCycle = 0; Led_Red_dutyCycle < 32768; Led_Red_dutyCycle += 50)
    {
        FTM_DRV_UpdatePwmChannel(INST_FLEXTIMER_PWM_1,
                                     flexTimer_pwm_1_IndependentChannelsConfig[0].hwChannelId,
                                     FTM_PWM_UPDATE_IN_TICKS, (uint16_t)Led_Red_dutyCycle,
                                     0U,
                                     true);
        OSIF_TimeDelay(1);
    }
    OSIF_TimeDelay(10);
#endif
	ADC_DRV_WaitConvDone(ADC_INSTANCE);
	/* Store the channel result into a local variable */
	ADC_DRV_GetChanResult(ADC_INSTANCE, 0U, &adcRawValue);


/* if ECU is HMI then info to display to be sent */
	if(isHMI)
	{  
		for (int value=0;value< 54;value++) //270
		{
		  UpdateGauge("z0.val",value*5);

		}
	}


#ifdef LED_RED_TOGGLE
	/* Decrease the brightness */
    for (Led_Red_dutyCycle = 32768; Led_Red_dutyCycle > 0; Led_Red_dutyCycle -= 50)
    {
        FTM_DRV_UpdatePwmChannel(INST_FLEXTIMER_PWM_1,
                                     flexTimer_pwm_1_IndependentChannelsConfig[0].hwChannelId,
                                     FTM_PWM_UPDATE_IN_TICKS, (uint16_t)Led_Red_dutyCycle,
                                     0U,
                                     true);
        OSIF_TimeDelay(1);
    }
    OSIF_TimeDelay(10);
#endif
	/* Process the result to get the value in volts */
	adcValue = (( adcRawValue * 5000) / adcMax) ;// (ADC_VREFH - ADC_VREFL);
	Led_Green_dutyCycle= adcValue*2;
}

return exit_code;	  
	  


}
