/*
 * stm32f46xx_i2c_driver.C
 *
 *  Created on: 05-Dec-2020
 *      Author: saksh
 */


#include "stm32f46xx_i2c_driver.h"

static void i2c_stopf_interrupt_handle(I2C_Handle_t *pI2CHandle);
static void i2c_btf_interrupt_handle(I2C_Handle_t *pI2CHandle);
static void i2c_rxne_interrupt_handle(I2C_Handle_t *pI2CHandle);
static void i2c_txe_interrupt_handle(I2C_Handle_t *pI2CHandle);
static void i2c_addr_interrupt_handle(I2C_Handle_t *pI2CHandle);
static void i2c_sb_interrupt_handle(I2C_Handle_t *pI2CHandle);

static void I2C_GenerateStart(I2C_RegDef_t* pI2Cx) {
	pI2Cx->CR[0] |= (1 << I2C_CR1_START);
}

static void I2C_ExecuteAddrPhase(I2C_RegDef_t* pI2Cx, uint8_t SlaveAddr, uint8_t RWn) {

	SlaveAddr = (SlaveAddr & 0x7F) << 1;
	if(RWn) {
		//Set RWn bit for read
		SlaveAddr |= (1 << 0);
	}
	else {
		//Clear R/Wn bit for write
		SlaveAddr &= ~(1 << 0);
	}
	pI2Cx->DR = SlaveAddr;
}

void I2C_GenerateStop(I2C_RegDef_t* pI2Cx) {
	pI2Cx->CR[0] |= (1 << I2C_CR1_STOP);
}

uint8_t I2C_GetFlagStatus(uint32_t reg, uint32_t bit) {
	if(reg & (1 << bit)) {
		return SET;
	}
	else {
		return RESET;
	}
}

static void I2C_clearADDRFlag(I2C_RegDef_t* pI2Cx) {

	uint16_t dummyRead;
	dummyRead = pI2Cx->SR[0];
	dummyRead = pI2Cx->SR[1];
	(void)dummyRead;
}

/*
 * Peripheral clock setup
 */
/********************************************
 * @fn			- I2C_PeriClockControl
 *
 * @brief		- To enable/disable peripheral clock enable for given GPIO port
 *
 * @param[in]	- base address of I2C peripheral
 * @param[in]	- ENABLE or DISABLE macros
 *
 * @return		- none
 *
 * @Note		- none
 *
 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi){
	if(EnorDi == ENABLE) {
		if (pI2Cx == I2C1)
		{
			I2C1_PCLK_EN();
		}
		else if (pI2Cx == I2C2)
		{
			I2C2_PCLK_EN();
		}
		else if (pI2Cx == I2C3)
		{
			I2C3_PCLK_EN();
		}
	}
	else {
		if (pI2Cx == I2C1)
		{
			I2C1_PCLK_DI();
		}
		else if (pI2Cx == I2C2)
		{
			I2C2_PCLK_DI();
		}
		else if (pI2Cx == I2C3)
		{
			I2C3_PCLK_DI();
		}
	}
}

void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi) {

	if(EnOrDi == ENABLE) {
		pI2Cx->CR[0] |= ( 1 << I2C_CR1_ACK );
	}else {
		pI2Cx->CR[0] &= ~( 1 << I2C_CR1_ACK );
	}
}



/*
 * Init and De-init
 */
/********************************************
 * @fn			- I2C_Init
 *
 * @brief		- To initialize I2C with appropriate configuration
 *
 * @param[in]	- I2C Handle containing base address of I2C and configuration settings
 *
 * @return		- none
 *
 * @Note		- none
 *
 */

void I2C_Init(I2C_Handle_t *pI2CHandle){

	uint32_t pclk1_freq;
	uint8_t FREQ;
	uint16_t ccr_val;
	uint32_t trise = 0;

	//Enable peripheral clock
	I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);

	//1. Configure the mode
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM) {
		//Configure CCR register for SM
		pI2CHandle->pI2Cx->CCR &= ~( 1 << I2C_CCR_FS);
	}
	else {
		//Configure CCR register for FM
		pI2CHandle->pI2Cx->CCR |= ( 1 << I2C_CCR_FS);
	}

	//2. Configure speed of serial clk
	pclk1_freq = RCC_GetPclk1Val();
	FREQ = pclk1_freq/1000000U;				//Divide the frequency by 1M to get FREQ val

	//Configure FREQ parameter in CR2
	pI2CHandle->pI2Cx->CR[1] &= ~(0x3F);
	pI2CHandle->pI2Cx->CR[1] |= FREQ;

	//Configure CCR register
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM) {
		ccr_val = pclk1_freq/( 2 * pI2CHandle->I2C_Config.I2C_SCLSpeed );
		pI2CHandle->pI2Cx->CCR &= ~(0xFFF << I2C_CCR_CCR);
		pI2CHandle->pI2Cx->CCR |= (ccr_val << I2C_CCR_CCR);
	}
	else {
		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle) {
			ccr_val = pclk1_freq/(pI2CHandle->I2C_Config.I2C_SCLSpeed * 25);
		}
		else {
			ccr_val = pclk1_freq/(pI2CHandle->I2C_Config.I2C_SCLSpeed * 3);
		}
		pI2CHandle->pI2Cx->CCR &= ~(0xFFF << I2C_CCR_CCR);
		pI2CHandle->pI2Cx->CCR |= (ccr_val << I2C_CCR_CCR);
	}

	//3. Configure device address
	pI2CHandle->pI2Cx->OAR[0] &= ~(0x7F << 1);
	pI2CHandle->pI2Cx->OAR[0] |= (pI2CHandle->I2C_Config.I2C_DeviceAddr & 0x7F) << 1;
	pI2CHandle->pI2Cx->OAR[0] |= (1 << 14);					//Refer to RM: I2C_OAR1

	//4. Configure ack control
	pI2CHandle->pI2Cx->CR[0] &= ~(1 << I2C_CR1_ACK);
	pI2CHandle->pI2Cx->CR[0] |= (I2C_ACK_ENABLE << I2C_CR1_ACK);

	//5. Configure Duty Cycle
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed > I2C_SCL_SPEED_SM) {
		pI2CHandle->pI2Cx->CCR &= ~(1 << I2C_CCR_DUTY);
		pI2CHandle->pI2Cx->CCR |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << I2C_CCR_DUTY);
	}

	//6. Configure TRISE register
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM) {
		trise = (RCC_GetPclk1Val() / 1000000U) + 1;			//(1000ns * Fpclk1) + 1
	}
	else {
		trise = (RCC_GetPclk1Val() / 1000000000U * 300) + 1;			//(300ns * Fpclk1) + 1
	}
	pI2CHandle->pI2Cx->TRISE = trise & 0x3F;

}

/********************************************
 * @fn			- I2C_DeInit
 *
 * @brief		- To de-initialize I2C peripheral
 *
 * @param[in]	- I2C Handle containing base address of I2C
 *
 * @return		- none
 *
 * @Note		- none
 *
 */
void I2C_DeInit(I2C_RegDef_t *pI2Cx){
	if (pI2Cx == I2C1)
	{
		I2C1_REG_RESET();
	}
	else if (pI2Cx == I2C2)
	{
		I2C2_REG_RESET();
	}
	else if (pI2Cx == I2C3)
	{
		I2C3_REG_RESET();
	}
}


/********************************************
 * @fn			- I2C_PeripheralControl
 *
 * @brief		- To enable I2C peripheral
 *
 * @param[in]	-
 * @param[in]	- Base address of I2C peripheral
 * @param[in]	- ENABLE or DISABLE macros
 *
 * @return		- none
 *
 * @Note		- none
 *
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi){
	if(EnOrDi == ENABLE) {
		pI2Cx->CR[0] |= ( 1 << I2C_CR1_PE);
	}
	else {
		pI2Cx->CR[0] &= ~( 1 << I2C_CR1_PE);
	}
}

void I2C_SlaveEnableCallbackEvent(I2C_RegDef_t *pI2Cx, uint8_t EnorDi) {

	if(EnorDi == ENABLE) {
		//Implement the code to enable ITBUFEN Control Bit
		pI2Cx->CR[1] |= ( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		pI2Cx->CR[1] |= ( 1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2Cx->CR[1] |= ( 1 << I2C_CR2_ITERREN);
	}else {
		//Implement the code to enable ITBUFEN Control Bit
		pI2Cx->CR[1] &= ~( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		pI2Cx->CR[1] &= ~( 1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2Cx->CR[1] &= ~( 1 << I2C_CR2_ITERREN);
	}
}

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr) {

	uint16_t temp;

	//Generate START condition
	I2C_GenerateStart(pI2CHandle->pI2Cx);

	while(I2C_GetFlagStatus(pI2CHandle->pI2Cx->SR[0],I2C_SR1_SB) == RESET);

	//Clear EV5: clear SR1 register and write DR register
	temp = pI2CHandle->pI2Cx->SR[0];
	(void)temp;
	I2C_ExecuteAddrPhase(pI2CHandle->pI2Cx,SlaveAddr,0);			//Set RWn bit to 0

	while(I2C_GetFlagStatus(pI2CHandle->pI2Cx->SR[0],I2C_SR1_ADDR) == RESET);

	//Clear EV6: clear ADDR flag
	I2C_clearADDRFlag(pI2CHandle->pI2Cx);

	//Send data until len becomes 0
	while(Len > 0) {
		//Wait till TXE is set
		while(I2C_GetFlagStatus(pI2CHandle->pI2Cx->SR[0],I2C_SR1_TXE) == RESET);
		pI2CHandle->pI2Cx->DR = *pTxBuffer;
		pTxBuffer++;
		Len--;
	}

	//After Len becomes 0, wait TXE=1 and BTF=1 before sending STOP condition
	while(I2C_GetFlagStatus(pI2CHandle->pI2Cx->SR[0],I2C_SR1_TXE) == RESET);
	while(I2C_GetFlagStatus(pI2CHandle->pI2Cx->SR[0],I2C_SR1_BTF) == RESET);

	if(Sr == DISABLE)
		I2C_GenerateStop(pI2CHandle->pI2Cx);

}

void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr){

	uint16_t temp;

	//Generate START condition
	I2C_GenerateStart(pI2CHandle->pI2Cx);

	//Wait for Start Bit to be 1
	while(I2C_GetFlagStatus(pI2CHandle->pI2Cx->SR[0],I2C_SR1_SB) == RESET);

	//Clear EV5: clear SR1 register and write DR register
	temp = pI2CHandle->pI2Cx->SR[0];
	(void)temp;
	I2C_ExecuteAddrPhase(pI2CHandle->pI2Cx,SlaveAddr,1);		//Set RWn bit to 1 for read

	//Wait for ADDR to be 1
	while(I2C_GetFlagStatus(pI2CHandle->pI2Cx->SR[0],I2C_SR1_ADDR) == RESET);

	if(Len == 1) {

		//Disable ACK
		I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);

		//Clear EV6: clear ADDR flag
		I2C_clearADDRFlag(pI2CHandle->pI2Cx);

		//Wait for Rx Buffer to be non empty
		while(I2C_GetFlagStatus(pI2CHandle->pI2Cx->SR[0],I2C_SR1_RXNE) == RESET);

		//STOP=1
		if(Sr == DISABLE)
			I2C_GenerateStop(pI2CHandle->pI2Cx);

		*pRxBuffer = pI2CHandle->pI2Cx->DR;
		pRxBuffer++;
		Len--;
	}
	else {

		//Clear EV6: clear ADDR flag
		I2C_clearADDRFlag(pI2CHandle->pI2Cx);

		//Receive data until len becomes 0
		while(Len > 0) {

			//Wait for Rx Buffer to be non empty
			while(I2C_GetFlagStatus(pI2CHandle->pI2Cx->SR[0],I2C_SR1_RXNE) == RESET);

			//EV7_1: Clear ACK bit and program STOP for last data byte(this should be done before reading DATA(N-1))
			if(Len == 2) {
				//Disable Acking
				I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);
				//STOP=1
				if(Sr == DISABLE)
					I2C_GenerateStop(pI2CHandle->pI2Cx);
			}

			*pRxBuffer = pI2CHandle->pI2Cx->DR;
			pRxBuffer++;
			Len--;
		}
	}

	//Re-enable Acking
	if(pI2CHandle->I2C_Config.I2C_AckControl == I2C_ACK_ENABLE) {
		I2C_ManageAcking(pI2CHandle->pI2Cx, ENABLE);
	}

}

uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr) {

	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->TxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Implement code to Generate START Condition
		I2C_GenerateStart(pI2CHandle->pI2Cx);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR[1] |= ( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVTEN Control Bit
		pI2CHandle->pI2Cx->CR[1] |= ( 1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR[1] |= ( 1 << I2C_CR2_ITERREN);

	}

	return busystate;
}

uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr) {

	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pRxBuffer = pRxBuffer;
		pI2CHandle->RxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
		pI2CHandle->RxSize = Len; //Rxsize is used in the ISR code to manage the data reception
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Implement code to Generate START Condition
		I2C_GenerateStart(pI2CHandle->pI2Cx);


		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR[1] |= ( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR[1] |= ( 1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR[1] |= ( 1 << I2C_CR2_ITERREN);
	}

	return busystate;
}

/*
 * Slave mode APIs
 */

void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx, uint8_t data) {
	pI2Cx->DR = data;
}
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx) {
	return (uint8_t) pI2Cx->DR;
}

/*
 * IRQ configuration and ISR handling
 */
void I2C_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi){
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

static void i2c_sb_interrupt_handle(I2C_Handle_t *pI2CHandle) {

	uint32_t temp;

	//Clear EV5: clear SR1 register and write DR register
	temp = pI2CHandle->pI2Cx->SR[0];
	(void)temp;
	if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX) {
		I2C_ExecuteAddrPhase(pI2CHandle->pI2Cx,pI2CHandle->DevAddr,1);			//Set RWn bit to 0
	}
	else {
		I2C_ExecuteAddrPhase(pI2CHandle->pI2Cx,pI2CHandle->DevAddr,0);			//Set RWn bit to 0
	}

}

static void i2c_addr_interrupt_handle(I2C_Handle_t *pI2CHandle) {

	//Check for device mode
	if(I2C_GetFlagStatus(pI2CHandle->pI2Cx->SR[1], I2C_SR2_MSL) == SET) {

		//Master
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX) {
			//Clear EV6: clear ADDR flag
			I2C_clearADDRFlag(pI2CHandle->pI2Cx);
		}
		else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX) {
			if(pI2CHandle->RxLen == 1) {
				//Disable ACK
				I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);
			}

			//Clear EV6: clear ADDR flag
			I2C_clearADDRFlag(pI2CHandle->pI2Cx);
		}
	}else {

		//Slave
		I2C_clearADDRFlag(pI2CHandle->pI2Cx);
	}

}

static void i2c_txe_interrupt_handle(I2C_Handle_t *pI2CHandle) {

	//Check for Master mode
	if(I2C_GetFlagStatus(pI2CHandle->pI2Cx->SR[1], I2C_SR2_MSL) == SET) {
		//Check for Tx state
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX) {
			if(pI2CHandle->TxLen > 0) {
				pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);
				pI2CHandle->pTxBuffer++;
				pI2CHandle->TxLen--;
			}
		}
	}
	else {

		//Slave mode
		//check for transmitter mode
		if(I2C_GetFlagStatus(pI2CHandle->pI2Cx->SR[1], I2C_SR2_TRA) == SET) {
			I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_REQ);
		}
	}
}

static void i2c_rxne_interrupt_handle(I2C_Handle_t *pI2CHandle) {

	//Check for Master mode(Implementation for Master receive data)
	if(I2C_GetFlagStatus(pI2CHandle->pI2Cx->SR[1], I2C_SR2_MSL) == SET) {
		//Check for Rx state
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX) {
			if(pI2CHandle->RxSize == 1) {
				//STOP=1
				if(pI2CHandle->Sr == DISABLE)
					I2C_GenerateStop(pI2CHandle->pI2Cx);

				*(pI2CHandle->pRxBuffer) = pI2CHandle->pI2Cx->DR;
				pI2CHandle->pRxBuffer++;
				pI2CHandle->RxLen--;
			}
			else if (pI2CHandle->RxSize > 1) {
				if(pI2CHandle->RxLen > 0) {
					//EV7_1: Clear ACK bit and program STOP for last data byte(this should be done before reading DATA(N-1))
					if(pI2CHandle->RxLen == 2) {
						//Disable Acking
						I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);
						//STOP=1
						//if(pI2CHandle->Sr == DISABLE)
						//	I2C_GenerateStop(pI2CHandle->pI2Cx);						//If STOP is generated here, RxNE bit is not set for last data block
					}

					*(pI2CHandle->pRxBuffer) = pI2CHandle->pI2Cx->DR;
					pI2CHandle->pRxBuffer++;
					pI2CHandle->RxLen--;
				}
			}
			if(pI2CHandle->RxLen == 0) {

				//STOP=1
				if(pI2CHandle->Sr == DISABLE) {
					I2C_GenerateStop(pI2CHandle->pI2Cx);
				}

				//close communication
				I2C_CloseReceiveData(pI2CHandle);

				//Send acknowledgment to application
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_DONE);

			}
		}
	}
	else {

		//Slave
		//check for receiver mode
		if(I2C_GetFlagStatus(pI2CHandle->pI2Cx->SR[1], I2C_SR2_TRA) == RESET) {
			I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_RECV);
		}
	}
}

static void i2c_stopf_interrupt_handle(I2C_Handle_t *pI2CHandle) {

	uint32_t temp;
	if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX) {		//only applicable for slave mode
		//Clear SR register
		//1. Read SR register
		temp = pI2CHandle->pI2Cx->SR[0];
		(void) temp;

		//2. Write CR register
		pI2CHandle->pI2Cx->CR[0] |= 0x0;

		//3. Notify application
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOPF);
	}
}

static void i2c_btf_interrupt_handle(I2C_Handle_t *pI2CHandle) {

	if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX) {
		if(pI2CHandle->TxLen == 0) {
			//1. generate STOP
			if(pI2CHandle->Sr == DISABLE)
				I2C_GenerateStop(pI2CHandle->pI2Cx);

			//2. close communication
			I2C_CloseSendData(pI2CHandle);

			//3. Notify application layer of tx completion
			I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_DONE);
		}
	}

}

void I2C_CloseSendData(I2C_Handle_t *pI2CHandle) {

	//Disable interrupt control bits

	//Implement the code to enable ITBUFEN Control Bit
	pI2CHandle->pI2Cx->CR[1] &= ~( 1 << I2C_CR2_ITBUFEN);

	//Implement the code to enable ITEVTEN Control Bit
	pI2CHandle->pI2Cx->CR[1] &= ~( 1 << I2C_CR2_ITEVTEN);

	//Implement the code to enable ITERREN Control Bit
	pI2CHandle->pI2Cx->CR[1] &= ~( 1 << I2C_CR2_ITERREN);

	pI2CHandle->TxLen = 0;
	pI2CHandle->pTxBuffer = NULL;
	pI2CHandle->TxRxState = I2C_READY;
}

void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle) {

	//Disable interrupt control bits

	//Implement the code to enable ITBUFEN Control Bit
	pI2CHandle->pI2Cx->CR[1] &= ~( 1 << I2C_CR2_ITBUFEN);

	//Implement the code to enable ITEVTEN Control Bit
	pI2CHandle->pI2Cx->CR[1] &= ~( 1 << I2C_CR2_ITEVTEN);

	//Implement the code to enable ITERREN Control Bit
	pI2CHandle->pI2Cx->CR[1] &= ~( 1 << I2C_CR2_ITERREN);

	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->TxRxState = I2C_READY;

	//Re-enable Acking
	if(pI2CHandle->I2C_Config.I2C_AckControl == I2C_ACK_ENABLE) {
		I2C_ManageAcking(pI2CHandle->pI2Cx, ENABLE);
	}
}

void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle) {

	//Interrupt handling for both master and slave mode of device

	if(I2C_GetFlagStatus(pI2CHandle->pI2Cx->CR[1],I2C_CR2_ITEVTEN) == SET) {

		//1. Handle for event generated by SB event
			//Note: only applicable for master mode
		if(I2C_GetFlagStatus(pI2CHandle->pI2Cx->SR[0],I2C_SR1_SB) == SET) {
			i2c_sb_interrupt_handle(pI2CHandle);
		}

		//2. Handle for event: ADDR
			//Note: Master: address sent
			//		Slave: address matched
		if(I2C_GetFlagStatus(pI2CHandle->pI2Cx->SR[0],I2C_SR1_ADDR) == SET) {
			i2c_addr_interrupt_handle(pI2CHandle);
		}

		//3. STOPF event
		if(I2C_GetFlagStatus(pI2CHandle->pI2Cx->SR[0],I2C_SR1_STOPF) == SET) {
			i2c_stopf_interrupt_handle(pI2CHandle);
		}

		//4. BTF event
		if(I2C_GetFlagStatus(pI2CHandle->pI2Cx->SR[0],I2C_SR1_BTF) == SET) {
			i2c_btf_interrupt_handle(pI2CHandle);
		}

		if(I2C_GetFlagStatus(pI2CHandle->pI2Cx->CR[1],I2C_CR2_ITBUFEN) == SET) {

			//5. RxNE event
			if(I2C_GetFlagStatus(pI2CHandle->pI2Cx->SR[0],I2C_SR1_RXNE) == SET) {
				i2c_rxne_interrupt_handle(pI2CHandle);
			}

			//6. TXE event
			if(I2C_GetFlagStatus(pI2CHandle->pI2Cx->SR[0],I2C_SR1_TXE) == SET) {
				i2c_txe_interrupt_handle(pI2CHandle);
			}
		}
	}
}

void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle) {

	uint32_t temp1,temp2;

    //Know the status of  ITERREN control bit in the CR2
	temp2 = (pI2CHandle->pI2Cx->CR[1]) & ( 1 << I2C_CR2_ITERREN);


/***********************Check for Bus error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR[0]) & ( 1<< I2C_SR1_BERR);
	if(temp1  && temp2 )
	{
		//This is Bus error

		//Implement the code to clear the buss error flag
		pI2CHandle->pI2Cx->SR[0] &= ~( 1 << I2C_SR1_BERR);

		//Implement the code to notify the application about the error
	   I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_BERR);
	}

/***********************Check for arbitration lost error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR[0]) & ( 1 << I2C_SR1_ARLO );
	if(temp1  && temp2)
	{
		//This is arbitration lost error

		//Implement the code to clear the arbitration lost error flag
		pI2CHandle->pI2Cx->SR[0] &= ~( 1 << I2C_SR1_ARLO);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_ARLO);

	}

/***********************Check for ACK failure  error************************************/

	temp1 = (pI2CHandle->pI2Cx->SR[0]) & ( 1 << I2C_SR1_AF);
	if(temp1  && temp2)
	{
		//This is ACK failure error

	    //Implement the code to clear the ACK failure error flag
		pI2CHandle->pI2Cx->SR[0] &= ~( 1 << I2C_SR1_AF);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_AF);
	}

/***********************Check for Overrun/underrun error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR[0]) & ( 1 << I2C_SR1_OVR);
	if(temp1  && temp2)
	{
		//This is Overrun/underrun

	    //Implement the code to clear the Overrun/underrun error flag
		pI2CHandle->pI2Cx->SR[0] &= ~( 1 << I2C_SR1_OVR);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_OVR);
	}

/***********************Check for Time out error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR[0]) & ( 1 << I2C_SR1_TIMEOUT);
	if(temp1  && temp2)
	{
		//This is Time out error

	    //Implement the code to clear the Time out error flag
		pI2CHandle->pI2Cx->SR[0] &= ~( 1 << I2C_SR1_OVR);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_TIMEOUT);
	}
}
