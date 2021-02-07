/*
 * stm32f46xx_spi_driver.c
 *
 *  Created on: 07-Nov-2020
 *      Author: saksh
 */

#include "stm32f46xx_spi_driver.h"

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);

/*
 * Peripheral clock setup
 */
/********************************************
 * @fn			- GPIO_PeriClockControl
 *
 * @brief		- To enable/disable peripheral clock enable for given GPIO port
 *
 * @param[in]	- base address of SPI peripheral
 * @param[in]	- ENABLE or DISABLE macros
 *
 * @return		- none
 *
 * @Note		- none
 *
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi){
	if(EnorDi == ENABLE) {
		if (pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}
		else if (pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}
		else if (pSPIx == SPI3)
		{
			SPI3_PCLK_EN();
		}
		else if (pSPIx == SPI4)
		{
			SPI4_PCLK_EN();
		}
	}
	else {
		if (pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}
		else if (pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}
		else if (pSPIx == SPI3)
		{
			SPI3_PCLK_EN();
		}
		else if (pSPIx == SPI4)
		{
			SPI4_PCLK_EN();
		}
	}
}

/*
 * Init and De-init
 */
/********************************************
 * @fn			- SPI_Init
 *
 * @brief		- To initialize SPI with appropriate configuration
 *
 * @param[in]	- SPI Handle containing base address of SPI and configuration settings
 *
 * @return		- none
 *
 * @Note		- none
 *
 */

void SPI_Init(SPI_Handle_t *pSPIHandle){

	//Enable peripheral clock
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	//2. Configure device mode
	if(pSPIHandle->SPI_Config.SPI_DeviceMode == SPI_DEVICE_MODE_MASTER) {
		pSPIHandle->pSPIx->CR[0] |= ( 1 << SPI_CR1_MSTR );
	}
	else if(pSPIHandle->SPI_Config.SPI_DeviceMode == SPI_DEVICE_MODE_SLAVE) {
		pSPIHandle->pSPIx->CR[0] &= ~( 1 << SPI_CR1_MSTR );
	}

	//3. Configure bus config
	if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_CONFIG_FD) {
		pSPIHandle->pSPIx->CR[0] &= ~( 1 << SPI_CR1_BIDIMODE );
	}
	else if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_CONFIG_HD_TX) {
		pSPIHandle->pSPIx->CR[0] |= ( 1 << SPI_CR1_BIDIMODE );
		pSPIHandle->pSPIx->CR[0] |= ( 1 << SPI_CR1_BIDIOE );
	}
	else if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_CONFIG_HD_RX) {
		pSPIHandle->pSPIx->CR[0] |= ( 1 << SPI_CR1_BIDIMODE );
		pSPIHandle->pSPIx->CR[0] &= ~( 1 << SPI_CR1_BIDIOE );
	}
	else if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_CONFIG_SIMPLEX_RX) {
		pSPIHandle->pSPIx->CR[0] &= ~( 1 << SPI_CR1_BIDIMODE );
		pSPIHandle->pSPIx->CR[0] |= ( 1 << SPI_CR1_RXONLY );
	}

	//3. Configure data frame format
	if(pSPIHandle->SPI_Config.SPI_DFF == SPI_DFF_8BITS) {
		pSPIHandle->pSPIx->CR[0] &= ~( 1 << SPI_CR1_DFF );
	}
	else if(pSPIHandle->SPI_Config.SPI_DFF == SPI_DFF_16BITS) {
		pSPIHandle->pSPIx->CR[0] |= ( 1 << SPI_CR1_DFF );
	}

	//4. Configure CPHA
	if(pSPIHandle->SPI_Config.SPI_CPHA == SPI_CPHA_LOW) {
		pSPIHandle->pSPIx->CR[0] &= ~( 1 << SPI_CR1_CPHA );
	}
	else if(pSPIHandle->SPI_Config.SPI_CPHA == SPI_CPHA_HIGH) {
		pSPIHandle->pSPIx->CR[0] |= ( 1 << SPI_CR1_CPHA );
	}

	//5. configure CPOL
	if(pSPIHandle->SPI_Config.SPI_CPOL == SPI_CPOL_LOW) {
		pSPIHandle->pSPIx->CR[0] &= ~( 1 << SPI_CR1_CPOL );
	}
	else if(pSPIHandle->SPI_Config.SPI_CPOL == SPI_CPOL_HIGH) {
		pSPIHandle->pSPIx->CR[0] |= ( 1 << SPI_CR1_CPOL );
	}

	//6. Configure sclk speed
	pSPIHandle->pSPIx->CR[0] &= ~( 0x7 << SPI_CR1_BR );
	pSPIHandle->pSPIx->CR[0] |= ( pSPIHandle->SPI_Config.SPI_SclkSpeed << SPI_CR1_BR );

	//7. configure SSM
	pSPIHandle->pSPIx->CR[0] &= ~( 1 << SPI_CR1_SSM );
	pSPIHandle->pSPIx->CR[0] |= ( pSPIHandle->SPI_Config.SPI_SSM << SPI_CR1_SSM );


	//Enable SPI peripheral(Done through different API)
	//pSPIHandle->pSPIx->CR[0] |= ( 1 << SPI_CR1_SPE );

}

/********************************************
 * @fn			- SPI_DeInit
 *
 * @brief		- To de-initialize SPI peripheral
 *
 * @param[in]	- SPI Handle containing base address of SPI
 *
 * @return		- none
 *
 * @Note		- none
 *
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx){
	if (pSPIx == SPI1)
	{
		SPI1_REG_RESET();
	}
	else if (pSPIx == SPI2)
	{
		SPI2_REG_RESET();
	}
	else if (pSPIx == SPI3)
	{
		SPI3_REG_RESET();
	}
	else if (pSPIx == SPI4)
	{
		SPI4_REG_RESET();
	}
}


/*
 * Data send and receive
 */
/********************************************
 * @fn			- SPI_SendData
 *
 * @brief		- To send data through SPI peripheral
 *
 * @param[in]	- SPI Handle containing base address of SPI
 * @param[in]	- Tx buffer containing data to be sent
 * @param[in]	- Length of data block
 *
 * @return		- none
 *
 * @Note		- This is a blocking call
 *
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len){
	while(Len > 0) {
		//while( (pSPIx->SR >> SPI_SR_TXE & 0x1) == 0x0 );
		while( (pSPIx->SR & (1 << SPI_SR_TXE )) == 0x0 );
		//if( (pSPIx->CR[0] >> SPI_CR1_DFF & 0x1) == 0x1 ) {
		if( (pSPIx->CR[0] & ( 1 << SPI_CR1_DFF)) ) {
			pSPIx->DR = *(pTxBuffer+1)<<8 | *pTxBuffer;
			//pSPIx->DR = *((uint16_t*)pTxBuffer);			//another way of above statement
			pTxBuffer += 2;
			//(uint16_t*)pTxBuffer++;					//another way of above statement
			Len -= 2;
		}
		else {
			pSPIx->DR = *pTxBuffer;
			pTxBuffer++;
			Len--;
		}

	}
}



void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len) {
	while(Len > 0) {
		//Wait until Rx buffer is empty
		while( (pSPIx->SR & (1 << SPI_SR_RXNE )) == 0x0 );
		//16 bit data format
		if( (pSPIx->CR[0] & ( 1 << SPI_CR1_DFF)) ) {
			//Load data from data register into Rx buffer
			*((uint16_t*)pRxBuffer) = (uint16_t)(pSPIx->DR & 0x0000FFFF);
			//Increment RxBuffer address by 2(due to 2 bytes)
			pRxBuffer += 2;
			Len -= 2;
		}
		else {
			*pRxBuffer = pSPIx->DR & 0x000000FF;
			pRxBuffer++;
			Len--;
		}

	}
}

/********************************************
 * @fn			- SPI_SendDataIT
 *
 * @brief		- To send data through SPI peripheral
 *
 * @param[in]	- SPI Handle containing base address of SPI
 * @param[in]	- Tx buffer containing data to be sent
 * @param[in]	- Length of data block
 *
 * @return		- none
 *
 * @Note		- This is a non-blocking call
 *
 */
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len){

	uint8_t state = pSPIHandle->TxState;

	if(state != SPI_BUSY_TX) {
		pSPIHandle->TxLen = Len;
		pSPIHandle->pTxBuffer = pTxBuffer;

		//Setting SPI status to busy
		pSPIHandle->TxState = SPI_BUSY_TX;

		//Enabling the TXEIE flag to generate interrupt whenever TXE flag is set
		pSPIHandle->pSPIx->CR[1] |= ( 1 << SPI_CR2_TXEIE );
	}

	return state;
}

/********************************************
 * @fn			- SPI_ReceiveDataIT
 *
 * @brief		- To receive data through SPI peripheral
 *
 * @param[in]	- SPI Handle containing base address of SPI
 * @param[in]	- Rx buffer containing data to be received
 * @param[in]	- Length of data block
 *
 * @return		- none
 *
 * @Note		- This is a non-blocking call
 *
 */
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len){
	uint8_t state = pSPIHandle->RxState;

	if(state != SPI_BUSY_RX) {
		pSPIHandle->RxLen = Len;
		pSPIHandle->pRxBuffer = pRxBuffer;

		//Setting SPI status to busy
		pSPIHandle->RxState = SPI_BUSY_RX;

		//Enabling the RXNEIE flag to generate interrupt whenever RXNE flag is set
		pSPIHandle->pSPIx->CR[1] |= ( 1 << SPI_CR2_RXNEIE );
	}

	return state;
}

/*
 * IRQ configuration and ISR handling
 */
void SPI_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi){
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

void SPI_IRQHandling(SPI_Handle_t *pSPIHandle) {

	//Check if TXE flag is set
	if((pSPIHandle->pSPIx->SR & ( 1 << SPI_SR_TXE )) && (pSPIHandle->pSPIx->SR & ( 1 << SPI_CR2_TXEIE )) ) {
		//Tx ISR
		spi_txe_interrupt_handle(pSPIHandle);
	}
	else if((pSPIHandle->pSPIx->SR & ( 1 << SPI_SR_RXNE )) && (pSPIHandle->pSPIx->SR & ( 1 << SPI_CR2_RXNEIE )) ) {
		//Rx ISR
		spi_rxne_interrupt_handle(pSPIHandle);
	}
	else if((pSPIHandle->pSPIx->SR & ( 1 << SPI_SR_OVR )) && (pSPIHandle->pSPIx->SR & ( 1 << SPI_CR2_ERRIE )) ) {
		//Err ISR
		spi_ovr_err_interrupt_handle(pSPIHandle);
	}
}


/********************************************
 * @fn			- SPI_PeripheralControl
 *
 * @brief		- To enable SPI peripheral
 *
 * @param[in]	-
 * @param[in]	- Base address of SPI peripheral
 * @param[in]	- ENABLE or DISABLE macros
 *
 * @return		- none
 *
 * @Note		- none
 *
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi){
	if(EnOrDi == ENABLE) {
		pSPIx->CR[0] |= ( 1 << SPI_CR1_SPE);
	}
	else {
		pSPIx->CR[0] &= ~( 1 << SPI_CR1_SPE);
	}
}

/********************************************
 * @fn			- SPI_SSIConfig
 *
 * @brief		- To configure SSI bit of SPI peripheral
 *
 * @param[in]	-
 * @param[in]	- Base address of SPI peripheral
 * @param[in]	- ENABLE or DISABLE macros
 *
 * @return		- none
 *
 * @Note		- none
 *
 */
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi){
	if(EnOrDi == ENABLE) {
		pSPIx->CR[0] |= ( 1 << SPI_CR1_SSI);
	}
	else {
		pSPIx->CR[0] &= ~( 1 << SPI_CR1_SSI);
	}
}

/*********************************************************************
 * @fn      		  - SPI_SSOEConfig
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */
void  SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR[1] |=  (1 << SPI_CR2_SSOE);
	}else
	{
		pSPIx->CR[1] &=  ~(1 << SPI_CR2_SSOE);
	}


}



static void  spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle) {

	if( (pSPIHandle->pSPIx->CR[0] & ( 1 << SPI_CR1_DFF)) ) {
		pSPIHandle->pSPIx->DR = *(pSPIHandle->pTxBuffer+1)<<8 | *(pSPIHandle->pTxBuffer);
		pSPIHandle->pTxBuffer += 2;
		pSPIHandle->TxLen -= 2;
	}
	else {
		pSPIHandle->pSPIx->DR = *(pSPIHandle->pTxBuffer);
		pSPIHandle->pTxBuffer++;
		pSPIHandle->TxLen--;
	}

	if( !pSPIHandle->TxLen ){

		//TxLen is 0, stop SPI Tx transmission

		SPI_CloseTransmission(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_TX_CMPLT);
	}
}

static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle) {

	//16 bit data format
	if( (pSPIHandle->pSPIx->CR[0] & ( 1 << SPI_CR1_DFF)) ) {
		//Load data from data register into Rx buffer
		*((uint16_t*)pSPIHandle->pRxBuffer) = (uint16_t)(pSPIHandle->pSPIx->DR & 0x0000FFFF);
		pSPIHandle->pRxBuffer += 2;
		pSPIHandle->RxLen -= 2;
	}
	else {
		*(pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->DR & 0x000000FF;
		pSPIHandle->pRxBuffer++;
		pSPIHandle->RxLen--;
	}

	if( !pSPIHandle->RxLen ) {

		//Rxlen is 0, stop Rx transmission
		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_RX_CMPLT);
	}
}

static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle) {

	uint8_t temp;
	//Clear OVR flag
	if(pSPIHandle->RxState != SPI_BUSY_TX) {
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	(void)temp;
	SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_OVRERR);
}

void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle) {

	//Disable to TXEIE bit to stop receive any more interrupts
	pSPIHandle->pSPIx->CR[2] &= ~( 1 << SPI_CR2_TXEIE);
	//Reset all members related to SPI Tx transmission
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;
}

void SPI_CloseReception(SPI_Handle_t *pSPIHandle) {

	//Disable to RXNEIE bit to stop receive any more interrupts
	pSPIHandle->pSPIx->CR[1] &= ~( 1 << SPI_CR2_RXNEIE);
	//Reset all members related to SPI Rx transmission
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;
}

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx) {

	uint8_t temp;

	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;
}

__weak void SPI_ApplicationEventCallback(SPI_Handle_t* pSPIHandle, uint8_t AppEvent) {

	//This is the weak implementation, application may override this implementation
}
