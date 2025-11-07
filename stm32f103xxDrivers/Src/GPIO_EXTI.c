/*
 * main.c
 *
 *  Created on: Sep 23, 2025
 *      Author: stellarbeing22
 */

#include "stm32f103xx.h"


void delay(void)
{
	for(uint32_t i = 0; i<500000/10;i++);
}
int main(void)
{
	GPIO_Handler_t GPIO_LED;
	GPIO_LED.pGPIOBx = GPIOA;
	GPIO_LED.GPIO_PinConfig.GPIO_PinNumber = GPIO_Pin_5;
	GPIO_LED.GPIO_PinConfig.GPIO_PinMode = GPIO_OUT_PP_10;

	GPIO_Handler_t GPIO_BTN;
	GPIO_BTN.pGPIOBx = GPIOC;
	GPIO_BTN.GPIO_PinConfig.GPIO_PinNumber = GPIO_Pin_13;
	GPIO_BTN.GPIO_PinConfig.GPIO_PinMode = GPIO_IN_FLOATING;

	GPIO_init(&GPIO_LED);
	GPIO_init(&GPIO_BTN);

	while(1)
	{
		if(!((GPIO_BTN.pGPIOBx->IDR) & (1<<GPIO_Pin_13)))
		{
		delay();
		GPIO_ToggleOutputPin(GPIOA, GPIO_Pin_5);
		}
	}
}



/*
Handler function for EXTI0
void EXTI0_IRQHandler(void)
{
	GPIO_ToggleOutputPin(GPIOA, GPIO_Pin_5);
	GPIO_IRQHandling(GPIO_Pin_0);   // clear EXTI pending

	buttonPressed = 1;
}

*/
