/*
 * 015_USART_Tx_testing.c
 *
 *  Created on: 17-Jan-2021
 *      Author: saksh
 */


#include "stm32f46xx.h"
#include <string.h>
#include <stdio.h>

/*
 * USART1 mapping on GPIO pins
 * PA9 --> USART_TX
 * PA10 --> USART_RX
 * Alt fn mode: 7
 */

void delay(void) {
	for(int i=0; i<500000/2; i++);
}

void GPIO_USART1config(GPIO_Handle_t* pGPIO_USART) {

	pGPIO_USART->pGPIOx = GPIOA;
	pGPIO_USART->GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	pGPIO_USART->GPIO_PinConfig.GPIO_PinAltFunMode = 7;
	pGPIO_USART->GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	pGPIO_USART->GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	pGPIO_USART->GPIO_PinConfig.GPIO_PinSpeed = GPIO_OP_SPEED_FAST;

	//USART1_TX
	pGPIO_USART->GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_9;
	GPIO_Init(pGPIO_USART);

	//USART1_RX
	pGPIO_USART->GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_10;
	GPIO_Init(pGPIO_USART);
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

	GPIO_Handle_t GPIO_USART;
	GPIO_Handle_t GPIOBtn;

	USART_Handle_t USART1Handle;

	char msg[]  = "Testing UART peripheral...";

	GPIO_USART1config(&GPIO_USART);
	GPIO_BtnConfig(&GPIOBtn);

	USART1Handle.pUSARTx = USART1;
	USART1Handle.USART_Config.USART_Baud = USART_STD_BAUD_115200;
	USART1Handle.USART_Config.USART_Mode = USART_MODE_ONLY_TX;
	USART1Handle.USART_Config.USART_NoOfStopBits = USART_STOPBITS_1;
	USART1Handle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;
	USART1Handle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;
	USART1Handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;

	USART_Init(&USART1Handle);

	USART_PeripheralControl(USART1Handle.pUSARTx, ENABLE);

	while(1) {
		while(GPIO_ReadFromInputPin(GPIOBtn.pGPIOx, 13));
		delay();								//debouncing

		USART_SendData(&USART1Handle, (uint8_t*)msg, strlen(msg));
	}

	return 0;
}
