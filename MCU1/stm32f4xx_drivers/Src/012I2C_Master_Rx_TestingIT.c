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

//#define DEV_ADDR		0x68
#define DEV_ADDR		0x66		//To test Acknowledgment failure

uint8_t recvComplete = RESET;

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

	uint8_t msg_len;
	char recv_msg[32];

	uint8_t cmd_code_len = 0x51;
	uint8_t cmd_code_data = 0x52;

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

	while(1) {
		//Wait for button to be pressed
		while(GPIO_ReadFromInputPin(GPIOBtn.pGPIOx, 13));
		delay();								//debouncing

		I2C_PeripheralControl(I2C1Master.pI2Cx, ENABLE);

		while(I2C_MasterSendDataIT(&I2C1Master, &cmd_code_len, 1, DEV_ADDR, ENABLE) != I2C_READY);

		while(I2C_MasterReceiveDataIT(&I2C1Master, &msg_len, 1, DEV_ADDR, ENABLE) != I2C_READY);
		while(recvComplete == RESET);
		recvComplete = RESET;

		while(I2C_MasterSendDataIT(&I2C1Master, &cmd_code_data, 1, DEV_ADDR, ENABLE) != I2C_READY);

		while(I2C_MasterReceiveDataIT(&I2C1Master, (uint8_t*)recv_msg, msg_len, DEV_ADDR, DISABLE) != I2C_READY);
		while(recvComplete == RESET);
		recvComplete = RESET;

		recv_msg[msg_len + 1] = '\0';

		printf("Received %d bytes: %s\n",msg_len,recv_msg);

		//wait until device is busy
		while(I2C_GetFlagStatus(I2C1Master.pI2Cx->SR[1], I2C_SR2_BUSY));
		I2C_PeripheralControl(I2C1Master.pI2Cx, DISABLE);
	}

	return 0;
}

void I2C1_EV_IRQHandler(void) {
	I2C_EV_IRQHandling(&I2C1Master);
}

void I2C1_ER_IRQHandler(void) {
	I2C_ER_IRQHandling(&I2C1Master);
}

void I2C_ApplicationEventCallback(I2C_Handle_t* pI2CHandle, uint8_t AppEvent) {

	if(AppEvent == I2C_EV_TX_DONE) {
		printf("Tx is completed \n");
	}else if(AppEvent == I2C_EV_RX_DONE) {
		printf("Rx is completed \n");
		recvComplete = SET;
	}else if(AppEvent == I2C_ERROR_AF) {
		printf("Acknowledgment failure \n");
		//Master sends data but slave fails to send acknowledgment

		//Close communication
		I2C_CloseSendData(pI2CHandle);

		//Generate STOP to release the bus
		I2C_GenerateStop(pI2CHandle->pI2Cx);

		//Hang in infinite loop
		while(1);
	}
}
