/*
 * stm32f103xx_gpio_drivers.h
 *
 *  Created on: Sep 14, 2025
 *      Author: stellarbeing22
 *
 * Driver API requirements:
 * => GPIO initialization
 * => En/Dis GPIO port clock
 * => Read form GPIO pin
 * => Write to GPIO pin
 * => Configure Alternate function
 * => Interrupt handling
 *
 **********************************************************************************************************************
 *                                               FILE INDEX
 *                                    stm32f103xx_gpio_drivers.h
 *
 * 1. Data Structures
 *    1.1 GPIO_PinConfig_t
 *    1.2 GPIO_Handler_t
 *
 * 2. Pin Mode Definitions
 *    2.1 Input Modes
 *    2.2 Output Modes
 *    `	  2.2.1 Output Modes 10MHz
 *        2.2.2 Output Modes 2MHz
 *        2.2.3 Output Modes 50MHz
 *    2.3 Interrupt Modes
 *
 * 3. Pull-up / Pull-down Macros
 * 4. GPIO Pin Numbers
 * 5. AFIO Remap Bit Positions, Masks, and Options
 *
 * 6. Generic Functions
 *    6.1 GPIO_Port_Code()
 *
 * 7. Driver API
 *    7.1 Clock Control
 *        → GPIO_PeriClockControl()
 *
 *    7.2 Initialization and Deinitialization
 *        → GPIO_init()
 *        → GPIO_EXTI_init()
 *        → GPIO_Deinit()
 *        → GPIO_PinRemap()
 *
 *    7.3 Data Read/Write
 *        → GPIO_ReadFromInputPin()
 *        → GPIO_ReadFromInputPort()
 *        → GPIO_WriteToOutputPin()
 *        → GPIO_WriteToOutputPort()
 *        → GPIO_ToggleOutputPin()
 *
 *    7.4 Interrupt Configuration and Handling
 *        → GPIO_IRQInterruptConfig()
 *        → GPIO_IRQPriorityConfig()
 *        → GPIO_IRQHandling()
 *
 ***********************************************************************************************************************
 */



#ifndef INC_STM32F103XX_GPIO_DRIVERS_H_
#define INC_STM32F103XX_GPIO_DRIVERS_H_

#include"stm32f103xx.h"



/* =========================================================================
 * 1. Data Structures
 * =========================================================================*/

//1.1
//GPIO_PinConfig_t
typedef struct
{
	 uint8_t GPIO_PinNumber;    /* 0..15 */
	 uint8_t GPIO_PinMode;      /* Input, Output, Analog, Alternate Function, possible values form @GPIO_PIN_MODES */
	 uint8_t GPIO_PU_PD;        /* Alternate Function identifier (if mode = AF) */
//   uint8_t GPIO_Speed;
}GPIO_PinConfig_t;

//1.2
//GPIO_Handler_t
typedef struct
{
	GPIO_RegDef_t *pGPIOBx;                   /*This holds the base address of the GPIO port to which the pin belongs*/
	GPIO_PinConfig_t GPIO_PinConfig;          /*This holds GPIO pin configuration settings*/
}GPIO_Handler_t;




/* =========================================================================
 * 2. Pin Mode Definitions
 * =========================================================================*/


/*In input mode (MODE = 00):
 * 00: Analog mode
 * 01: Floating input (reset state)
 * 10: Input with pull-up / pull-down
 * 11: Reserved In output mode
 *
 * In output mode (MODE > 00):
 *  00: General purpose output push-pull
 *  01: General purpose output Open-drain
 *  10: Alternate function output Push-pull
 *  11: Alternate function output Open-drain
 *
 *  MODE:
 *  00: Input mode (reset state)
 *  01: Output mode, max speed 10 MHz.
 *  10: Output mode, max speed 2 MHz.
 *  11: Output mode, max speed 50 MHz.
 *   */

//2.1
//Input modes                                 CNF MODE
#define GPIO_IN_ANALOG                0x0   //00  00
#define GPIO_IN_FLOATING              0x4   //01  00
#define GPIO_IN_PU_PD                 0x8   //10  00

//2.2
//output modes


//2.2.1
//output modes(10MHZ)
#define GPIO_OUT_PP_10                0x1   //00  01
#define GPIO_OUT_OD_10                0x5   //01  01
#define GPIO_OUT_ALTFN_PP_10          0x9   //10  01
#define GPIO_OUT_ALTFN_OD_10          0xD   //11  01

//2.2.2
//output modes(2MHZ)
#define GPIO_OUT_PP_2                 0x2   //00  10
#define GPIO_OUT_OD_2                 0x6   //01  10
#define GPIO_OUT_ALTFN_PP_2           0xA   //10  10
#define GPIO_OUT_ALTFN_OD_2           0xE   //11  10

//2.2.3
//output modes(50MHZ)
#define GPIO_OUT_PP_50                0x3   //00  11
#define GPIO_OUT_OD_50                0x7   //01  11
#define GPIO_OUT_ALTFN_PP_50          0xB   //10  11
#define GPIO_OUT_ALTFN_OD_50          0xF   //11  11

//2.3
//Interrupt modes(custom)
#define GPIO_IT_RT                    0x1   //Rising edge Trigger
#define GPIO_IT_FT                    0x2   //Falling edge Trigger
#define GPIO_IT_RFT                   0x3   //Both edge trigger


/* =========================================================================
* 3. Pull-up / Pull-down Macros
 * =========================================================================*/
#define PULLDOWN                      0
#define PULLUP                        1
#define NO_PU_PD                      2


/* =========================================================================
* 4. GPIO Pin Numbers
 * =========================================================================*/

#define GPIO_Pin_0                    0
#define GPIO_Pin_1                    1
#define GPIO_Pin_2                    2
#define GPIO_Pin_3                    3
#define GPIO_Pin_4                    4
#define GPIO_Pin_5                    5
#define GPIO_Pin_6                    6
#define GPIO_Pin_7                    7
#define GPIO_Pin_8                    8
#define GPIO_Pin_9                    9
#define GPIO_Pin_10                   10
#define GPIO_Pin_11                   11
#define GPIO_Pin_12                   12
#define GPIO_Pin_13                   13
#define GPIO_Pin_14                   14
#define GPIO_Pin_15                   15



/* =========================================================================
 * 5. AFIO Remap Bit Positions, Masks, and Options
 * =========================================================================*/

/* Remap bit positions (masks) */
#define AFIO_MAPR_SPI1_REMAP_Pos      0
#define AFIO_MAPR_I2C1_REMAP_Pos      1
#define AFIO_MAPR_USART1_REMAP_Pos    2
#define AFIO_MAPR_USART2_REMAP_Pos    3
#define AFIO_MAPR_USART3_REMAP_Pos    4   // 2-bit field [5:4]

/* Masks */
#define AFIO_MAPR_SPI1_REMAP_Msk    (1U << AFIO_MAPR_SPI1_REMAP_Pos)
#define AFIO_MAPR_I2C1_REMAP_Msk    (1U << AFIO_MAPR_I2C1_REMAP_Pos)
#define AFIO_MAPR_USART1_REMAP_Msk  (1U << AFIO_MAPR_USART1_REMAP_Pos)
#define AFIO_MAPR_USART2_REMAP_Msk  (1U << AFIO_MAPR_USART2_REMAP_Pos)
#define AFIO_MAPR_USART3_REMAP_Msk  (3U << AFIO_MAPR_USART3_REMAP_Pos)

/* Options (values to write into mask region) */
#define SPI1_REMAP                  (1U << AFIO_MAPR_SPI1_MAPR_Pos)
#define I2C1_REMAP                  (1U << AFIO_MAPR_I2C1_MAPR_Pos)
#define USART3_REMAP_NONE           (0U << AFIO_MAPR_USART3_REMAP_Pos)
#define USART3_REMAP_PARTIAL        (1U << AFIO_MAPR_USART3_REMAP_Pos)
#define USART3_REMAP_FULL           (3U << AFIO_MAPR_USART3_REMAP_Pos)



/* =========================================================================
 * 6. Generic Functions
 * =========================================================================*/
uint8_t GPIO_Port_Code(GPIO_RegDef_t *pGPIOx);



/***********************************************************************************************************************
 *7.                                         API SUPPORTED BY THE DRIVER
 *                          FOR MORE INFORMATION ABOUT THE APIs CHECK THE FUNCTION DEFINITION
 ***********************************************************************************************************************/

//7.1
/* GPIO peripheral clock */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t En_or_Di);

//7.2
/* Init and Deinit */
void GPIO_init(GPIO_Handler_t *pGPIOHandle);
void GPIO_EXTI_init(GPIO_Handler_t *pGPIOHandle);
void GPIO_Deinit(GPIO_RegDef_t *pGPIOx);
void GPIO_PinRemap(uint32_t RemapMask, uint32_t Value);

//7.3
/* Data read and write */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint8_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber);

//7.4
/* IQR config and ISR handling */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t En_or_Di);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);


#endif /* INC_STM32F103XX_GPIO_DRIVERS_H_ */
