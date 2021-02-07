/*
 * 002led_toggle_btn.c
 *
 *  Created on: 26-Oct-2020
 *      Author: saksh
 */

#include "stm32f46xx.h"
#include "string.h"

void delay(void) {
	for(int i=0; i<500000/2; i++);
}

int main(void)
{
	GPIO_Handle_t GPIOLed;
	GPIO_Handle_t GPIOBtn;
	memset(&GPIOLed,0,sizeof(GPIOLed));
	memset(&GPIOBtn,0,sizeof(GPIOBtn));

	GPIOLed.pGPIOx = GPIOA;
	GPIOLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIOLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GPIOLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GPIOLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIOLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OP_SPEED_FAST;

	GPIOBtn.pGPIOx = GPIOC;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIOBtn.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OP_SPEED_FAST;


	GPIO_PeriClockControl(GPIOLed.pGPIOx, ENABLE);
	GPIO_PeriClockControl(GPIOBtn.pGPIOx, ENABLE);

	GPIO_Init(&GPIOLed);
	GPIO_Init(&GPIOBtn);

	//IRQ Configurations
	GPIO_IRQConfig(IRQ_NO_EXTI15_10, NVIC_IRQ_PRIO15, ENABLE);
	GPIO_IRQConfig(IRQ_NO_EXTI9_5, NVIC_IRQ_PRIO15, ENABLE);

	while(1);

}

void EXTI15_10_IRQHandler(void) {
	GPIO_IRQHandling(GPIO_PIN_NO_13);
	GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);
}
