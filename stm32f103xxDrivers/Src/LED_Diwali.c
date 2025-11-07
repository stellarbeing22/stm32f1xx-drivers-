/*
 * LED_Diwali.c
 *
 *  Created on: Oct 6, 2025
 *      Author: stellarbeing22
 */

#include "stm32f103xx.h"

void delay();
void LED_init();

int main()
{
	LED_init();

	while(1)
	{
		for (int i = 0; i < 8; i++) {
		    GPIO_WriteToOutputPort(GPIOC, 1 << i);
		    delay();
		}
		for (int i = 6; i >= 1; i--) {
		    GPIO_WriteToOutputPort(GPIOC, 1 << i);
		    delay();
		}
	}
}

void LED_init()
{
    GPIO_Handler_t GPIO_DataPin;
    GPIO_DataPin.pGPIOBx = GPIOC;
    GPIO_DataPin.GPIO_PinConfig.GPIO_PinMode = GPIO_OUT_PP_10;     // Example mode: output push-pull

    for (uint8_t i = 0; i < 8; i++)
    {
        GPIO_DataPin.GPIO_PinConfig.GPIO_PinNumber = i;
        GPIO_init(&GPIO_DataPin);
    }
}

void delay()
{
	for(uint32_t i = 0; i<500000/2;i++);
}
