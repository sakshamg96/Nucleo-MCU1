/*
 * stm32f46xx_gpio.c
 *
 *  Created on: 25-Oct-2020
 *      Author: saksh
 */

#include "stm32f46xx_gpio_driver.h"

/*
 * Peripheral clock setup
 */

/********************************************
 * @fn			- GPIO_PeriClockControl
 *
 * @brief		- To enable/disable peripheral clock enable for given GPIO port
 *
 * @param[in]	- base address of GPIO peripheral
 * @param[in]	- ENABLE or DISABLE macros
 *
 * @return		- none
 *
 * @Note		- none
 *
 */

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if (pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}
		else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}
		else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}
		else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}
		else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
	}
	else
	{
		if (pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}
		else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}
		else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}
		else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}
		else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		}
		else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_DI();
		}
		else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_DI();
		}
		else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		}
	}
	//RCC->AHB1ENR |= (EnorDi << (pGPIOx-AHB1PERIPH_BASEADDR)/0x400 );
	//RCC->AHB1ENR &= ~(EnorDi << (pGPIOx-AHB1PERIPH_BASEADDR)/0x400 );
}

/*
 * Init and De-init
 */

/********************************************
 * @fn			- GPIO_Init
 *
 * @brief		- To initialize GPIO pin with appropriate configuration
 *
 * @param[in]	- GPIO Handle containing base address of GPIO pin and configuration settings
 *
 * @return		- none
 *
 * @Note		- none
 *
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle){

	//Enable peripheral clock
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	//1. configure mode of gpio pin
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG) {
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	}
	else {
		//. interrupt mode
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT) {
			//1. configure the FTSR
			EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
			//clearing the RTSR register
			EXTI->RTSR &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT) {
			//1. configure the RTSR
			EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
			//clearing the FTSR register
			EXTI->FTSR &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT) {
			//1. configure the RTSR and FTSR register
			EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
			//clearing the FTSR register
			EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
		}

		//2. Configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4);
		uint8_t temp2 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)/4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp2] &= ~( 0xf << (temp1 * 4));
		SYSCFG->EXTICR[temp2] |= ( portcode << (temp1 * 4));

		//3. enable the exti interrupt delivery using IMR
		EXTI->IMR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
	}

	//2. configure speed of gpio pin
	pGPIOHandle->pGPIOx->OSPEEDER &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDER |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));

	//3. configure pupd settings
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));

	//4. configure the optype
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x3 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));

	//5. configure the alt functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN) {
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber <= 7) {
			pGPIOHandle->pGPIOx->AFR[0] &= ~(0xf << (4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
			pGPIOHandle->pGPIOx->AFR[0] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)));
		}
		else {
			pGPIOHandle->pGPIOx->AFR[1] &= ~(0xf << (4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
			pGPIOHandle->pGPIOx->AFR[1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8)));
		}
	}

}

/********************************************
 * @fn			- GPIO_DeInit
 *
 * @brief		- To reset the value of GPIO port
 *
 * @param[in]	- base address of GPIO port
 *
 * @return		- none
 *
 * @Note		- none
 *
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx) {

	if (pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}
	else if (pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}
	else if (pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}
	else if (pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}
	else if (pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}
	else if (pGPIOx == GPIOF)
	{
		GPIOF_REG_RESET();
	}
	else if (pGPIOx == GPIOG)
	{
		GPIOG_REG_RESET();
	}
	else if (pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}
}

/*
 * Data read/write
 */

/********************************************
 * @fn			- GPIO_ReadFromInputPin
 *
 * @brief		- Read value from input pin
 *
 * @param[in]	- base address of GPIO port
 * @param[in]	- Pin index whose value needs to be read
 *
 * @return		- none
 *
 * @Note		- none
 *
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber) {
	return (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x1);
}

/********************************************
 * @fn			- GPIO_ReadFromInputPort
 *
 * @brief		- Read value from input pin
 *
 * @param[in]	- base address of GPIO port
 *
 * @return		- none
 *
 * @Note		- none
 *
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx) {
	return (uint16_t)(pGPIOx->IDR);
}

/********************************************
 * @fn			- GPIO_WriteToOutputPin
 *
 * @brief		- Read value from input pin
 *
 * @param[in]	- base address of GPIO port
 *
 * @return		- none
 *
 * @Note		- none
 *
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value) {
	pGPIOx->ODR &= ~((0x1) << PinNumber);
	pGPIOx->ODR |= ((Value & 0x1) << PinNumber);
}

/********************************************
 * @fn			- GPIO_WriteToOutputPort
 *
 * @brief		- Read value from input pin
 *
 * @param[in]	- base address of GPIO port
 * @param[in]	- value of output port
 *
 * @return		- none
 *
 * @Note		- none
 *
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value) {
	pGPIOx->ODR = Value;
}

/********************************************
 * @fn			- GPIO_ToggleOutputPin
 *
 * @brief		- Toggle output value of gpio pin
 *
 * @param[in]	- base address of GPIO port
 * @param[in]	- pin number of gpio port
 *
 * @return		- none
 *
 * @Note		- none
 *
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber) {
	pGPIOx->ODR ^= (1 << PinNumber);
}

/********************************************
 * @fn			- GPIO_IRQConfig
 *
 * @brief		- Configure interrupt pin on MCU side
 *
 * @param[in]	- interrupt priority
 * @param[in]	- Enable or disable interrupt
 *
 * @return		- none
 *
 * @Note		- none
 *
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi){
	if(EnorDi == ENABLE) {
		NVIC->ISER[IRQNumber/32] |= ( 1 << (IRQNumber%32) );
	}
	else {
		NVIC->ICER[IRQNumber/32] |= ( 1 << (IRQNumber%32) );
	}
	uint8_t shift_amount = (IRQNumber%4 * 8) + (8 - NO_PR_BITS_IMPLEMENTED);
	NVIC->IPR[IRQNumber/4] &= ~( 0xff << (IRQNumber%4 * 8));
	NVIC->IPR[IRQNumber/4] |= ( IRQPriority << shift_amount);
}
void GPIO_IRQHandling(uint8_t PinNumber){

	//clear the exti pending register corresponding to correct pin number
	if(EXTI->PR & ( 1 << PinNumber )) {
		EXTI->PR |= ( 1 << PinNumber );
	}
}

