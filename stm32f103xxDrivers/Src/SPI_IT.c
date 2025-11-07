/*
 * SPI_IT.c
 *
 *  Created on: Oct 7, 2025
 *      Author: stellarbeing22
 */

#include <string.h>
#include "stm32f103xx.h"


void GPIO_For_SPI();
void SPI_Config();

SPI_Handler_t SPIA;

int main()
{
	//1. Initialise the GPIO and SPI
	GPIO_For_SPI();
	SPI_Config();
	SPI_SSOEConfig(SPI1,ENABLE);

	//2. Declare Tx buffer and buffer size
	uint8_t user_message[] = "Hello World!";
	uint8_t datalen = strlen((char*)user_message);

	//3. Enable NVIC interrupt
	SPI_IRQInterruptConfig(IRQ_SPI1_Global,ENABLE);

	//4. Enable the SPI peripheral
	SPI_PeripheralControl(SPI1,ENABLE);

	//5. Send the data
	SPI_SendData_IT(&SPIA, user_message, datalen);

	//6. Wait until transmission completes
	while(SPIA.TxState != SPI_READY);

	//7. Disable SPI peripheral (optional)
	SPI_PeripheralControl(SPI1, DISABLE);

	while(1);
}


void GPIO_For_SPI()
{

	GPIO_Handler_t SPI_NSS,SPI_MOSI,SPI_CLK; //As master MOSI, CLK and NSS are output
	GPIO_Handler_t SPI_MISO;                 // As master MISO is input

	SPI_NSS.pGPIOBx = GPIOA;
	SPI_NSS.GPIO_PinConfig.GPIO_PinMode = GPIO_OUT_ALTFN_PP_10;
	SPI_NSS.GPIO_PinConfig.GPIO_PinNumber = GPIO_Pin_4;

	SPI_MOSI.pGPIOBx = GPIOA;
	SPI_MOSI.GPIO_PinConfig.GPIO_PinMode = GPIO_OUT_ALTFN_PP_10;
	SPI_MOSI.GPIO_PinConfig.GPIO_PinNumber = GPIO_Pin_7;

	SPI_CLK.pGPIOBx = GPIOA;
	SPI_CLK.GPIO_PinConfig.GPIO_PinMode = GPIO_OUT_ALTFN_PP_10;
	SPI_CLK.GPIO_PinConfig.GPIO_PinNumber = GPIO_Pin_5;

	SPI_MISO.pGPIOBx = GPIOA;
	SPI_MISO.GPIO_PinConfig.GPIO_PinMode = GPIO_IN_FLOATING;
	SPI_MISO.GPIO_PinConfig.GPIO_PinNumber = GPIO_Pin_6;

	GPIO_init(&SPI_MISO);
	GPIO_init(&SPI_MOSI);
	GPIO_init(&SPI_CLK);
	GPIO_init(&SPI_NSS);

}

void SPI_Config()
{
	SPIA.pSPIx = SPI1;
	SPIA.SPI_config.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPIA.SPI_config.SPI_BusConfig = SPI_BUS_CONFIG_FDPX;
	SPIA.SPI_config.SPI_Speed = SPI_CLOCK_DIV_2;
	SPIA.SPI_config.SPI_CPOL = SPI_CPOL_0_IDLE;
	SPIA.SPI_config.SPI_CPHA = SPI_CPHA_1ST_TRANSIT;
	SPIA.SPI_config.SPI_SSM = SPI_SSM_DI;
	SPIA.SPI_config.SPI_DFF = SPI_DFF_8BITS;

	SPI_init(&SPIA);
}

void SPI1_IRQHandler()
{
	SPI_IRQHandling(&SPIA);
}
