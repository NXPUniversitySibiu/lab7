#ifndef _PIN_MUX_H_
#define _PIN_MUX_H_

#include "pins_driver.h"

/***********************************************************************************************************************
 * Definitions
 **********************************************************************************************************************/

/*!
 * @addtogroup pin_mux
 * @{
 */

/***********************************************************************************************************************
 * API
 **********************************************************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif


/*! @brief Definitions/Declarations for BOARD_InitPins Functional Group */
/*! @brief User definition pins */
#define LED_RED_PWM_PORT    PTD
#define LED_RED_PWM_PIN     15U
#define LED_GREEN_PWM_PORT    PTD
#define LED_GREEN_PWM_PIN     16U
#define LED_BLUE_PWM_PORT    PTD
#define LED_BLUE_PWM_PIN     0U
#define BTN2_SW3_PIN_PORT    PTC
#define BTN2_SW3_PIN_PIN     12U
#define BTN3_SW2_PIN_PORT    PTC
#define BTN3_SW2_PIN_PIN     13U
#define ECU_HMI_PORT    PTE
#define ECU_HMI_PIN     0U
/*! @brief User number of configured pins */
#define NUM_OF_CONFIGURED_PINS0 9
/*! @brief User configuration structure */
extern pin_settings_config_t g_pin_mux_InitConfigArr0[NUM_OF_CONFIGURED_PINS0];


#if defined(__cplusplus)
}
#endif

/*!
 * @}
 */
#endif /* _PIN_MUX_H_ */

/***********************************************************************************************************************
 * EOF
 **********************************************************************************************************************/

