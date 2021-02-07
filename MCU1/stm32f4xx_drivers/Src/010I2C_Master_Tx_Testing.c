/*
 * 010I2C_Master_Tx_Testing.c
 *
 *  Created on: 12-Dec-2020
 *      Author: saksh
 */

#include "stm32f46xx.h"
#include <string.h>

/*
 * I2C1 mapping on GPIO pins
 * PB6 --> I2C1_SCL
 * PB7 --> I2C1_SDA
 * Alt fn mode: 4
 */

void delay(void) {
	for(int i=0; i<500000/2; i++);
}

void GPIO_I2C1config(GPIO_Handle_t* pGPIO_I2C) {

	pGPIO_I2C->pGPIOx = GPIOB;
	pGPIO_I2C->GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	pGPIO_I2C->GPIO_PinConfig.GPIO_PinAltFunMode = 4;
	pGPIO_I2C->GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	pGPIO_I2C->GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU;
	pGPIO_I2C->GPIO_PinConfig.GPIO_PinSpeed = GPIO_OP_SPEED_FAST;

	//I2C1_SCL
	pGPIO_I2C->GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(pGPIO_I2C);

	//I2C1_SDA
	pGPIO_I2C->GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	GPIO_Init(pGPIO_I2C);
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

	GPIO_Handle_t GPIO_I2C;
	GPIO_Handle_t GPIOBtn;
	I2C_Handle_t I2C1Master;
	uint32_t devAddr = 0x68;

	char user_data[] = "Bow down to the king!!";

	GPIO_I2C1config(&GPIO_I2C);
	GPIO_BtnConfig(&GPIOBtn);

	I2C1Master.pI2Cx = I2C1;
	I2C1Master.I2C_Config.I2C_AckControl = I2C_ACK_ENABLE;
	I2C1Master.I2C_Config.I2C_DeviceAddr = devAddr;
	I2C1Master.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;


	I2C_Init(&I2C1Master);

	while(1) {
		//Wait for button to be pressed
		while(GPIO_ReadFromInputPin(GPIOBtn.pGPIOx, 13));
		delay();								//debouncing

		I2C_PeripheralControl(I2C1Master.pI2Cx, ENABLE);

		I2C_MasterSendData(&I2C1Master, (uint8_t*)user_data, strlen(user_data), devAddr, DISABLE);

		//wait until device is busy
		while(I2C_GetFlagStatus(I2C1Master.pI2Cx->SR[1], I2C_SR2_BUSY));
		I2C_PeripheralControl(I2C1Master.pI2Cx, DISABLE);
	}

	return 0;
}
