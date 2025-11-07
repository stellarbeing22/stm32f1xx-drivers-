/*
 * stm32f103xx_gpio_drivers.c
 *
 *  Created on: Sep 14, 2025
 *      Author: stellarbeing22
 ******************************************************************************
 *                              FUNCTION INDEX
 * ----------------------------------------------------------------------------
 *  1.  GPIO_Port_Code()                   - Returns port code (A=0, B=1, etc.) for EXTI configuration
 *
 *  2.  GPIO_PeriClockControl()            - Enable or disable GPIO peripheral clock
 *
 *  3.  GPIO_init()                        - Initialize GPIO pin in input/output/alternate/analog mode
 *  4.  GPIO_EXTI_init()                   - Initialize GPIO pin for external interrupt functionality
 *  5.  GPIO_Deinit()                      - Reset GPIO peripheral registers to default state
 *  6.  GPIO_PinRemap()                    - Configure or remap alternate function pins
 *
 *  7.  GPIO_ReadFromInputPin()            - Read logic level from a specific input pin
 *  8.  GPIO_ReadFromInputPort()           - Read 16-bit data from entire GPIO port
 *  9.  GPIO_WriteToOutputPin()            - Write logic value to a specific output pin
 * 10.  GPIO_WriteToOutputPort()           - Write 8-bit/16-bit value to entire GPIO port
 * 11.  GPIO_ToggleOutputPin()             - Toggle logic level of a specific output pin
 *
 * 12.  GPIO_IRQInterruptConfig()          - Configure NVIC to enable or disable GPIO interrupt line
 * 13.  GPIO_IRQPriorityConfig()           - Configure NVIC interrupt priority for GPIO
 * 14.  GPIO_IRQHandling()                 - Handle GPIO interrupt request for a given pin
 ******************************************************************************/




#include "stm32f103xx_gpio_drivers.h"

/* GPIO peripheral clock */

/* GPIO peripheral clock control */

/**
 * @fn                        - GPIO_PeriClockControl
 *
 * @brief                     - Enables or disables the peripheral clock for the specified GPIO port.
 *
 * @param[in]                 - pGPIOx  Base address of the GPIO peripheral (e.g., GPIOA, GPIOB).
 * @param[in]                 - En_or_Di ENABLE to turn on the clock, DISABLE to turn it off.(uses macros)
 *
 * @return                    - None
 *
 * @note                      - This must be called before initializing the GPIO pins.
 */

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t En_or_Di)
{
	if(En_or_Di == ENABLE)
	{
		if((pGPIOx == GPIOA))
		{
			GPIOA_PCLK_EN;
		}else if((pGPIOx == GPIOB))
		{
			GPIOB_PCLK_EN;
		}else if((pGPIOx == GPIOC))
		{
			GPIOC_PCLK_EN;
		}else if((pGPIOx == GPIOD))
		{
			GPIOD_PCLK_EN;
		}else if((pGPIOx == GPIOE))
		{
			GPIOE_PCLK_EN;
		}
	}else
	{
		if((pGPIOx = GPIOA))
		{
			GPIOA_PCLK_DI;
		}else if((pGPIOx == GPIOB))
		{
			GPIOB_PCLK_DI;
		}else if((pGPIOx == GPIOC))
		{
			GPIOC_PCLK_DI;
		}else if((pGPIOx == GPIOD))
		{
			GPIOD_PCLK_DI;
		}else if((pGPIOx == GPIOE))
		{
			GPIOE_PCLK_DI;
		}
	}
}


/* Init and Deinit */

/**
 * @fn                        - GPIO_init
 *
 * @brief                     - Initializes the GPIO pin according to the specified configuration.
 *
 * @param[in]                 - pGPIOHandle Pointer to a GPIO_Handler_t structure containing pin configuration.
 *
 * @return                    - None
 *
 * @note                      - The peripheral clock for the GPIO port is enabled within this function, no need
 * 		                        for external call.
 */

void GPIO_init(GPIO_Handler_t *pGPIOHandle)
{
	//Configure pin mode(without EXTI)
	//Configure Pin speed
	//configure pull-up/pull-down/no pull
	//Configure output type
	uint32_t temp = 0;

	//Enalbe peripheral clock
	GPIO_PeriClockControl(pGPIOHandle->pGPIOBx, ENABLE);

	// NOTE: GPIO_PinMode must already contain full [CNF:MODE] 4-bit value.
	// Example: 0b1001 = Output, AF push-pull, 10 MHz
	// Otherwise this function will misconfigure the pin.

	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber<8)
	{
		temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (4*(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOBx->CRL &= ~(0xF<<(4* (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)));//Clear the Bit-field
		pGPIOHandle->pGPIOBx->CRL |= temp;
	}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber>=8)
	{
		temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (4*(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber-8));
		pGPIOHandle->pGPIOBx->CRH &= ~(0xF<<(4* (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber-8)));//Clear the Bit-field
		pGPIOHandle->pGPIOBx->CRH |= temp;
	}
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_IN_PU_PD)// If the input mode is Pull up/ pull down,
	{

		if(pGPIOHandle->GPIO_PinConfig.GPIO_PU_PD == PULLUP)//ODR's nth bit set as 1 will set pull up
		{
			pGPIOHandle->pGPIOBx->ODR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);//Enable pullup

		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PU_PD == PULLDOWN)//ODR's nth bit reset as 0 will set pull down
		{
			pGPIOHandle->pGPIOBx->ODR &= ~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
	}
	// Alternate functionality is not configured in this function, A separate function @GPIO_PinRemap is implemented for the Remaping
}


/**
 * @fn                        - GPIO_EXTI_init
 *
 * @brief                     - nitializes the GPIO pin according to the specified configuration.
 *
 * @param[in]                 - pGPIOx Base address of the GPIO peripheral to reset.
 *
 * @return                    - None
 *
 * @note                      - Make sure the peripheral clock for the GPIO port is enabled before calling this function.
 */

void GPIO_EXTI_init(GPIO_Handler_t *pGPIOHandle)
{
	//1. Configure the detection type: RT/FT/RFT
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_IT_RT)
	{
		//Configure RTSR
		EXTI->RTSR |= (1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		//Reset the corresponding bit in the FTSR
		EXTI->FTSR &= ~(1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

	}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_IT_FT)
	{
		//Configure FTSR
		EXTI->FTSR |= (1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		//Reset the corresponding bit in the Rising trigger selection register
		EXTI->RTSR &= ~(1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

	}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_IT_RFT)
	{
		//Configure both RTSR and FTSR
		EXTI->FTSR |= (1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		EXTI->RTSR |= (1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

	}
		//2. configure the GPIO port selection in AFIO->EXTIx Register
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber/4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber%4;
		uint32_t portcode = GPIO_Port_Code(pGPIOHandle->pGPIOBx);
		AFIO_PCLK_EN;

			AFIO->EXTICR[temp1] &= ~(0xF<<temp2*4);
			AFIO->EXTICR[temp1] |= (portcode<<temp2*4);


		//3. Enable the EXTI interrupt delivery using IMR
		EXTI->IMR |= (1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
}

/**
 * @fn                        - GPIO_Deinit
 *
 * @brief                     - Resets the GPIO port registers to default state.
 *
 * @param[in]                 - pGPIOx Base address of the GPIO peripheral to reset.
 *
 * @return                    - None
 *
 * @note                      -  This will reset all pins of the port, use with caution.
 */
void GPIO_Deinit(GPIO_RegDef_t *pGPIOx)
{
	if((pGPIOx == GPIOA))
	{
		GPIOA_REG_RESET;
	}else if((pGPIOx == GPIOB))
	{
		GPIOB_REG_RESET;
	}else if((pGPIOx == GPIOC))
	{
		GPIOC_REG_RESET;
	}else if((pGPIOx == GPIOD))
	{
		GPIOD_REG_RESET;
	}else if((pGPIOx == GPIOE))
	{
		GPIOE_REG_RESET;
	}
}

/**
 * @fn                        - GPIO_PinRemap
 *
 * @brief                     - Configures alternate pin mapping using AFIO->MAPR.
 *
 * @param[in]                 - RemapMask  Bitmask for the remap field (e.g., AFIO_MAPR_USART3_REMAP_Msk).
 * @param[in]                 - Value New value to write into the field (use *_REMAP macros).
 *
 * @return                    - None
 *
 * @note                      - This overwrites only the bits covered by RemapMask.
 *
 */

void GPIO_PinRemap(uint32_t RemapMask, uint32_t Value)
{
	AFIO_PCLK_EN;
	AFIO->MAPR &= ~RemapMask;
	AFIO->MAPR |= (Value&RemapMask);
/*
* GPIO_PinRemap() – AFIO remap helper
*
* NOTE TO FUTURE ME:
* ------------------
* This function centralizes all pin remapping using AFIO->MAPR.
* Each remap option has a "mask" (the bits in MAPR that need clearing)
* and a "value" (what to set them to).
*
* Why (value & mask)?
* -------------------
* It's a safety guard: it makes sure only the intended field is touched,
* so garbage values or typos won’t spill over into unrelated remaps.
* If you remove the AND, then passing a wrong constant could corrupt
* other AFIO fields and cause hours of debugging pain.
*
* Usage:
*    GPIO_PinRemap(AFIO_MAPR_USART3_REMAP_Msk, USART3_REMAP_FULL);
*    GPIO_PinRemap(AFIO_MAPR_SPI1_REMAP_Msk, SPI1_REMAP);
*
*
* TL;DR: Don’t “optimize” this by removing the mask check.
* It’s here to protect you from yourself.
*/

}


/* Data read and write */

/**
 * @fn                        - GPIO_ReadFromInputPin
 *
 * @brief                     - Reads the value of a specific input pin.
 *
 * @param[in]                 - pGPIOx Base address of the GPIO peripheral.
 * @param[in]                 - PinNumber Pin number (0-15) to read.
 *
 * @return                    - uint8_t Logic level of the pin (0 or 1).
 *
 * @note                      -  None
 */



uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t Value;
	Value = (uint8_t)((pGPIOx->IDR >> PinNumber)&0x00000001);
	return Value;
}

/**
 * @fn                        - GPIO_ReadFromInputPort
 *
 * @brief                     - Reads the entire input data register of a GPIO port.
 *
 * @param[in]                 - pGPIOx Base address of the GPIO peripheral.
 *
 * @return                    - uint16_t Current logic levels of all pins in the port.
 *
 * @note                      - None
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t Value;
	Value = (uint16_t)pGPIOx->IDR;
	return Value;
}

/**
 * @fn                        - GPIO_WriteToOutputPin
 *
 * @brief                     - Writes a logic level to a specific output pin.
 * @param[in]                 - pGPIOx Base address of the GPIO peripheral.
 * @param[in]                 - PinNumber Pin number (0-15) to write to.
 * @param[in]                 - Value Logic level to write (0 or 1).
 *
 * @return                    - None
 * @note                      - None
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber, uint8_t Value)
{
	if(Value == GPIO_PIN_SET)
	{
		//Write 1 to the output data register at the bit field corresponding to the PinNumber
		pGPIOx->ODR |= (1<<PinNumber);
	}else
	{
		//Write 1 to the output data register at the bit field corresponding to the PinNumber
		pGPIOx->ODR &= ~(1<<PinNumber);
	}
}

/**
 * @fn                        - GPIO_WriteToOutputPort
 *
 * @brief                     - Writes a value to the entire output data register of a GPIO port.
 *
 * @param[in]                 - pGPIOx Base address of the GPIO peripheral.
 * @param[in]                 - Value 16-bit value to write to the port.
 *
 * @return                    - None
 *
 * @note                      - Writing affects all pins of the port.
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint8_t Value)
{
	pGPIOx->ODR = Value;
}

/**
 * @fn                         - GPIO_ToggleOutputPin
 *
 * @brief                      - Toggles the logic level of a specific output pin.
 *
 * @param[in]                  - pGPIOx Base address of the GPIO peripheral.
 * @param[in]                  - PinNumber Pin number (0-15) to toggle.
 *
 * @return                     - None
 *
 * @note                       - Useful for blinking LEDs or generating square waves.
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1<<PinNumber);
}


/* IQR config and ISR handling */
/**
 * @fn                        - GPIO_IRQInterrruptConfig
 *
 * @brief                     - Configures the interrupt for a specific GPIO pin.
 *
 * @param[in]                 - IRQNumber IRQ number corresponding to the GPIO pin.
 * @param[in]                 - En_or_Di ENABLE to activate, DISABLE to deactivate the interrupt.(uses macros)
 *
 * @return                    - None
 *
 * @note                      - Ensure the NVIC controller is properly configured for the given IRQ.
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t En_or_Di)
{
	//Totaly 59 interrupts + 13 system interrupts
	if(En_or_Di == ENABLE)
	{
		if(IRQNumber <=31)
		{
			//Configure ISER0 reg
			*NVIC_ISER0 |= (1 << IRQNumber);

		}else if(IRQNumber>32 && IRQNumber < 64)
		{
			//Configure ISER1 reg
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));

		}else if(IRQNumber>=64 && IRQNumber < 96)
		{
			//Configure ISER2 reg
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
		}
	}else
	{
		if(IRQNumber <=31)
		{
			//Configure ISER0 reg
			*NVIC_ICER0 |= (1 << IRQNumber);
		}else if(IRQNumber>=32 && IRQNumber < 64)
		{
			//Configure ISER1 reg
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		}else if(IRQNumber>=64 && IRQNumber < 96)
		{
			//Configure ISER2 reg
			*NVIC_ICER2 |= (1 << (IRQNumber % 64));
		}
	}
}


/**
 * @fn                        - GPIO_IRQPriorityConfig
 *
 * @brief                     - Configures the priority for a specific interrupt.
 *
 * @param[in]                 - IRQNumber IRQ number corresponding to the GPIO pin.
 * @param[in]                 - IRQPrority Priority of the interrupt (lower number = higher priority).
 *
 * @return                    - None
 *
 * @note                      - Ensure the NVIC controller is properly configured for the given IRQ.
 */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	//First check the IPR register
	uint8_t IPRx = IRQNumber/4;
	uint8_t IPRx_section = IRQNumber%4;
	uint8_t Shift_ammount = (8*IPRx_section) + (8-PRIORITY_BITS);
//	*(NVIC_IPR_BASEADDR + (IPRx)) &= ~(0xF<<Shift_ammount);
	*(NVIC_IPR_BASEADDR + (IPRx)) |= (IRQPriority<<Shift_ammount);

	// NVIC_IPR_BASEADDR is a uint32_t* (each step = 4 bytes).
	// Adding IPRx already advances by 4*IPRx bytes to reach the correct IPR register.
	// If you also multiply IPRx by 4, you overshoot (4*4 = 16 bytes per step), causing wrong offsets and hard-to-debug errors.

}

/*
 * if there is an interrupt inside the PR AND the processor is not executing any ISR,
 * it'll go to the fixed vector address but IF the processor is already executing an
 * existing ISR and another interrupt comes in the PR reg, then the priority is compared.
 *
 * 1. Implement the ISR function
 * 2. Store the addr. of your ISR at the vector addr. location corresponding to the IRQ
 *    number for which you have written the ISR
 *
 * NOTE1:- The ISR isn't implemented inside the driver layer, its application specific
 * NOTE2:- The Startup file already contains the .weak handlers, in main.c, call the
 *         IQRhandling fn inside your IQR handler inside the main
 */


/**
 * @fn                        - GPIO_IRQHandling
 * @brief                     - Handles the interrupt triggered by a GPIO pin by clearing its pending bit.
 * @param[in]                 - PinNumber Pin number (0-15) that generated the interrupt.
 * @return                    - None
 * @note                      - Should be called inside the EXTI IRQ handler.
 */
void GPIO_IRQHandling(uint8_t PinNumber)
{
	//Clear the EXTI OR register corresponding to the Pin number
	if(EXTI->PR & ( 1<< PinNumber))
	{
		//clear
		EXTI->PR |= (1 << PinNumber);
	}

}



/**
 * @fn                        - GPIO_Port_Code
 *
 * @brief                     - Returns the Port code(0-4) for the EXTIx GPIO mapping
 *
 * @param[in]                 - *pGPIOx The port at which the EXTIx will be mapped
 *
 * @return                    - Returns the Port code(0-4) to select the source for EXTIx interrupt
 *
 * @note                      - Refer to section 9.4.3 - 9.4.6 of RM0008
 *
 */
uint8_t GPIO_Port_Code(GPIO_RegDef_t *pGPIOx)
{
	     if(pGPIOx == GPIOA)return 0;
	else if(pGPIOx == GPIOB)return 1;
	else if(pGPIOx == GPIOC)return 2;
	else if(pGPIOx == GPIOD)return 3;
	else if(pGPIOx == GPIOE)return 4;
	else return 0;
}
