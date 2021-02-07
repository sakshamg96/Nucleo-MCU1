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

GPIO_Handle_t GPIO_USART;
GPIO_Handle_t GPIOBtn;

USART_Handle_t USART1Handle;

/*** Sending multiple messages to arduino  ******/
char *msg[3]  = {"Testing UART peripheral...", "Receiving alternate cAsE !", "BOW down !"};

//Reply from arduino will be stored here
char rx_buf[1024];

//This flag indicates reception completion
uint8_t rxCmplt = RESET;

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

	uint32_t cnt = 0;

	GPIO_USART1config(&GPIO_USART);
	GPIO_BtnConfig(&GPIOBtn);

	USART1Handle.pUSARTx = USART1;
	USART1Handle.USART_Config.USART_Baud = USART_STD_BAUD_115200;
	USART1Handle.USART_Config.USART_Mode = USART_MODE_TXRX;
	USART1Handle.USART_Config.USART_NoOfStopBits = USART_STOPBITS_1;
	USART1Handle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;
	USART1Handle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;
	USART1Handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;

	USART_Init(&USART1Handle);

	USART_PeripheralControl(USART1Handle.pUSARTx, ENABLE);

	USART_IRQConfig(IRQ_NO_USART1, 0, ENABLE);

	printf("Application is running\n");

    //do forever
    while(1)
    {
		//wait till button is pressed
		while( GPIO_ReadFromInputPin(GPIOBtn.pGPIOx, 13) );

		//to avoid button de-bouncing related issues 200ms of delay
		delay();

		// Next message index ; make sure that cnt value doesn't cross 2
		cnt = cnt % 3;

		//First lets enable the reception in interrupt mode
		//this code enables the receive interrupt
		while ( USART_ReceiveDataIT(&USART1Handle,(uint8_t*)rx_buf,strlen(msg[cnt])) != USART_READY );

		//Send the msg indexed by cnt in blocking mode
    	USART_SendData(&USART1Handle,(uint8_t*)msg[cnt],strlen(msg[cnt]));

    	printf("Transmitted : %s\n",msg[cnt]);


    	//Now lets wait until all the bytes are received from the arduino .
    	//When all the bytes are received rxCmplt will be SET in application callback
    	while(rxCmplt != SET);

    	//just make sure that last byte should be null otherwise %s fails while printing
    	rx_buf[strlen(msg[cnt])] = '\0';

    	//Print what we received from the arduino
    	printf("Received    : %s\n",rx_buf);

    	//invalidate the flag
    	rxCmplt = RESET;

    	//move on to next message indexed in msg[]
    	cnt ++;
    }


	return 0;
}

void USART1_IRQHandler(void) {

	USART_IRQHandling(&USART1Handle);
}

void USART_ApplicationEventCallback( USART_Handle_t *pUSARTHandle,uint8_t ApEv)
{
   if(ApEv == USART_EV_RX_DONE)
   {
			rxCmplt = SET;

   }else if (ApEv == USART_EV_TX_DONE)
   {
	   ;
   }
}
