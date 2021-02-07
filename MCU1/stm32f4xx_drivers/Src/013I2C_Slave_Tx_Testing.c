/*
 * 010I2C_Master_Tx_Testing.c
 *
 *  Created on: 12-Dec-2020
 *      Author: saksh
 */

#include "stm32f46xx.h"
#include <string.h>
#include <stdio.h>

/*
 * I2C1 mapping on GPIO pins
 * PB6 --> I2C1_SCL
 * PB7 --> I2C1_SDA
 * Alt fn mode: 4
 */

#define DEV_ADDR		0x68

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

I2C_Handle_t I2C1Master;

int main() {

	GPIO_Handle_t GPIO_I2C;
	GPIO_Handle_t GPIOBtn;

	GPIO_I2C1config(&GPIO_I2C);
	GPIO_BtnConfig(&GPIOBtn);

	I2C1Master.pI2Cx = I2C1;
	I2C1Master.I2C_Config.I2C_AckControl = I2C_ACK_ENABLE;
	I2C1Master.I2C_Config.I2C_DeviceAddr = DEV_ADDR;
	I2C1Master.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

	/**************** IRQ configurations ************/
	I2C_IRQConfig(IRQ_NO_I2C1_EV, NVIC_IRQ_PRIO0, ENABLE);
	I2C_IRQConfig(IRQ_NO_I2C1_ER, NVIC_IRQ_PRIO0, ENABLE);

	I2C_Init(&I2C1Master);

	I2C_SlaveEnableCallbackEvent(I2C1Master.pI2Cx,ENABLE);

	I2C_PeripheralControl(I2C1Master.pI2Cx, ENABLE);

	I2C_ManageAcking(I2C1Master.pI2Cx, ENABLE);

	while(1);

	return 0;
}

void I2C1_EV_IRQHandler(void) {
	I2C_EV_IRQHandling(&I2C1Master);
}

void I2C1_ER_IRQHandler(void) {
	I2C_ER_IRQHandling(&I2C1Master);
}

void I2C_ApplicationEventCallback(I2C_Handle_t* pI2CHandle, uint8_t AppEvent) {

	static uint8_t command_code = 0;
	static uint8_t cnt = 0;
	char send_msg[32] = "STM32 Slave is replying";

	if(AppEvent == I2C_EV_DATA_REQ) {

		//Master wants to receive data. Slave has to send data.
		if(command_code == 0x51) {
			//Send length of send_msg
			I2C_SlaveSendData(pI2CHandle->pI2Cx, strlen(send_msg));
		}else if(command_code == 0x52) {
			//Send contentsof send_msg
			I2C_SlaveSendData(pI2CHandle->pI2Cx, send_msg[cnt++]);
		}

	} else if(AppEvent == I2C_EV_DATA_RECV) {

		//Master wants to send data. Save has to read data.
		command_code = I2C_SlaveReceiveData(pI2CHandle->pI2Cx);

	}else if(AppEvent == I2C_ERROR_AF) {

		//This is only applicable during slave txing. Master has sent NACK, slave should stop
		//sending data
		cnt = 0;
		command_code = 0;
	}
	else if(AppEvent == I2C_EV_STOPF) {

		//Happens only during slave reception
		//Master has ended the communication
	}

}
