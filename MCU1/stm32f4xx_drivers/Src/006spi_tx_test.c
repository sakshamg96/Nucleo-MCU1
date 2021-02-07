/*
 * 006spi_tx_test.c
 *
 *  Created on: 15-Nov-2020
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

void GPIO_config(GPIO_Handle_t GPIO_SPI) {

	GPIO_SPI.pGPIOx = GPIOB;
	GPIO_SPI.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	GPIO_SPI.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	GPIO_SPI.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GPIO_SPI.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_SPI.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OP_SPEED_FAST;

	//SPI2_NSS
	//GPIO_SPI.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	//GPIO_Init(&GPIO_SPI);

	//SPI2_SCK
	GPIO_SPI.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&GPIO_SPI);

	//SPI2_MISO
	//GPIO_SPI.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	//GPIO_Init(&GPIO_SPI);

	//SPI2_MOSI
	GPIO_SPI.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&GPIO_SPI);

}

int main() {

	GPIO_Handle_t GPIO_SPI;
	SPI_Handle_t SPI2Master;

	char user_data[] = "Hello world";

	GPIO_config(GPIO_SPI);

	SPI2Master.pSPIx = SPI2;
	SPI2Master.SPI_Config.SPI_BusConfig = SPI_CONFIG_FD;
	SPI2Master.SPI_Config.SPI_CPHA = SPI_CPHA_LOW;
	SPI2Master.SPI_Config.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Master.SPI_Config.SPI_DFF = SPI_DFF_8BITS;
	SPI2Master.SPI_Config.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Master.SPI_Config.SPI_SSM = SPI_SSM_EN;
	SPI2Master.SPI_Config.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2;

	SPI_Init(&SPI2Master);

	//This removes MODF error by pullup up NSS pin internally
	SPI_SSIConfig(SPI2Master.pSPIx, ENABLE);
	SPI_SSOEConfig(SPI2Master.pSPIx, ENABLE);
	SPI_PeripheralControl(SPI2Master.pSPIx, ENABLE);

	SPI_SendData(SPI2Master.pSPIx, (uint8_t*)user_data, strlen(user_data));

	SPI_PeripheralControl(SPI2Master.pSPIx, DISABLE);

	while(1);

	return 0;
}
