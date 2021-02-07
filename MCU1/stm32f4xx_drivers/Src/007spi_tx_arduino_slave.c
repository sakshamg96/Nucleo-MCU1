/*
 * 007spi_tx_arduino_slave.c
 *
 *  Created on: 27-Nov-2020
 *      Author: saksh
 */

#include "stm32f46xx.h"
#include <string.h>

/*
 * SPI2 mapping on GPIO pins
 * PB12 --> SPI2_NSS
 * PB13 --> SPI2_SCK
 * PB14 --> SPI2_MISO
 * PB15 --> SPI2_MOSI
 * Alt fn mode: 5
 */

/*
 * SPI1 mapping on GPIO pins
 * PA4 --> SPI2_NSS
 * PB3 --> SPI2_SCK
 * PB4 --> SPI2_MISO
 * PB5 --> SPI2_MOSI
 * Alt fn mode: 5
 */

void delay(void) {
	for(int i=0; i<500000/2; i++);
}

void GPIO_SPI2config(GPIO_Handle_t* pGPIO_SPI) {

	pGPIO_SPI->pGPIOx = GPIOB;
	pGPIO_SPI->GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	pGPIO_SPI->GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	pGPIO_SPI->GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	pGPIO_SPI->GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU;
	pGPIO_SPI->GPIO_PinConfig.GPIO_PinSpeed = GPIO_OP_SPEED_FAST;

	//SPI2_NSS
	pGPIO_SPI->GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(pGPIO_SPI);

	//SPI2_SCK
	pGPIO_SPI->GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(pGPIO_SPI);

	//SPI2_MISO
	//pGPIO_SPI->GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	//GPIO_Init(pGPIO_SPI);

	//SPI2_MOSI
	pGPIO_SPI->GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(pGPIO_SPI);

}

void GPIO_SPI1config(GPIO_Handle_t* pGPIO_SPI) {

	pGPIO_SPI->pGPIOx = GPIOB;
	pGPIO_SPI->GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	pGPIO_SPI->GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	pGPIO_SPI->GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	pGPIO_SPI->GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU;
	pGPIO_SPI->GPIO_PinConfig.GPIO_PinSpeed = GPIO_OP_SPEED_FAST;


	//SPI1_SCK
	pGPIO_SPI->GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_3;
	GPIO_Init(pGPIO_SPI);

	//SPI2_MISO
	//pGPIO_SPI->GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	//GPIO_Init(pGPIO_SPI);

	//SPI2_MOSI
	pGPIO_SPI->GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GPIO_Init(pGPIO_SPI);

	//SPI1_NSS
	pGPIO_SPI->pGPIOx = GPIOA;
	pGPIO_SPI->GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	pGPIO_SPI->GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	pGPIO_SPI->GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	pGPIO_SPI->GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU;
	pGPIO_SPI->GPIO_PinConfig.GPIO_PinSpeed = GPIO_OP_SPEED_FAST;

	pGPIO_SPI->GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_4;
	GPIO_Init(pGPIO_SPI);

}

void GPIO_BtnConfig(GPIO_Handle_t *pGPIOBtn) {

	pGPIOBtn->pGPIOx = GPIOC;
	pGPIOBtn->GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	pGPIOBtn->GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	pGPIOBtn->GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	pGPIOBtn->GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	pGPIOBtn->GPIO_PinConfig.GPIO_PinSpeed = GPIO_OP_SPEED_FAST;

	GPIO_PeriClockControl(pGPIOBtn->pGPIOx, ENABLE);

	GPIO_Init(pGPIOBtn);
}

int main() {

	GPIO_Handle_t GPIO_SPI;
	GPIO_Handle_t GPIOBtn;
	SPI_Handle_t SPI2Master;

	char user_data[] = "Bow down to the king!!";

	GPIO_SPI2config(&GPIO_SPI);
	//GPIO_SPI1config(&GPIO_SPI);
	GPIO_BtnConfig(&GPIOBtn);

	SPI2Master.pSPIx = SPI2;
	//SPI2Master.pSPIx = SPI1;
	SPI2Master.SPI_Config.SPI_BusConfig = SPI_CONFIG_FD;
	SPI2Master.SPI_Config.SPI_CPHA = SPI_CPHA_LOW;
	SPI2Master.SPI_Config.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Master.SPI_Config.SPI_DFF = SPI_DFF_8BITS;
	SPI2Master.SPI_Config.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Master.SPI_Config.SPI_SSM = SPI_SSM_DI;
	SPI2Master.SPI_Config.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8;		//Generate 2MHz clock

	SPI_Init(&SPI2Master);

	while(1) {
		//Wait for button to be pressed
		while(GPIO_ReadFromInputPin(GPIOBtn.pGPIOx, 13));
		delay();								//debouncing

		SPI_SSOEConfig(SPI2Master.pSPIx, ENABLE);
		SPI_PeripheralControl(SPI2Master.pSPIx, ENABLE);

		uint8_t dataLen = strlen(user_data);

		SPI_SendData(SPI2Master.pSPIx, &dataLen, 1);

		SPI_SendData(SPI2Master.pSPIx, (uint8_t*)user_data, strlen(user_data));

		//Get busy flag status
		while((SPI2Master.pSPIx->SR & (1 << SPI_SR_BSY)));
		SPI_PeripheralControl(SPI2Master.pSPIx, DISABLE);
	}

	return 0;
}
