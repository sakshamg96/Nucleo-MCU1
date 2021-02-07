/*
 * 002led_toggle_btn.c
 *
 *  Created on: 26-Oct-2020
 *      Author: saksh
 */

#include "stm32f46xx.h"

void delay(void) {
	for(int i=0; i<500000/2; i++);
}

int main(void)
{
	GPIO_Handle_t GPIOLed;
	GPIO_Handle_t GPIOBtn;

	GPIOLed.pGPIOx = GPIOA;
	GPIOLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIOLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GPIOLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GPIOLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIOLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OP_SPEED_FAST;

	GPIOBtn.pGPIOx = GPIOC;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIOBtn.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OP_SPEED_FAST;


	GPIO_PeriClockControl(GPIOLed.pGPIOx, ENABLE);
	GPIO_PeriClockControl(GPIOBtn.pGPIOx, ENABLE);

	GPIO_Init(&GPIOLed);
	GPIO_Init(&GPIOBtn);

	while(1) {
		if(GPIO_ReadFromInputPin(GPIOBtn.pGPIOx, 13) == 0) {
			GPIO_ToggleOutputPin(GPIOLed.pGPIOx, 5);
			//while(GPIO_ReadFromInputPin(GPIOBtn.pGPIOx, 13) == 0);
			delay();													//btn debouncing
		}

	}
}
