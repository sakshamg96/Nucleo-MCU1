/*
 * stm32f46xx_i2c_driver.h
 *
 *  Created on: 05-Dec-2020
 *      Author: saksh
 */

#ifndef INC_STM32F46XX_I2C_DRIVER_H_
#define INC_STM32F46XX_I2C_DRIVER_H_

#include "stm32f46xx.h"

typedef struct
{
	uint32_t I2C_SCLSpeed;				/* |< Defines the speed of serial clock >*/
	uint8_t I2C_DeviceAddr;				/* |< Assigns device address in slave mode >*/
	uint8_t I2C_AckControl;				/* |< Enable/Disable acknowledge >*/
	uint8_t I2C_FMDutyCycle;			/* |< Sets duty cycle in fast mode >*/

}I2C_Config_t;

/*
 * Handle structure of I2C
 */

typedef struct {
	I2C_RegDef_t		*pI2Cx;			/* |< Hold the base address of I2Cx(x:0,1,2) peripheral >| */
	I2C_Config_t		I2C_Config;
	uint8_t				*pTxBuffer;		/* |< Hold the buffer address while sending data >| */
	uint8_t				*pRxBuffer;		/* |< Hold the buffer address while receiving data >| */
	uint32_t			TxLen;			/* |< Hold the remaining length of data during send >| */
	uint32_t			RxLen;			/* |< Hold the remaining length of data during receive >| */
	uint8_t				TxRxState;		/* |< Hold the I2C send/receive status >| */
	uint8_t				DevAddr;		/* |< Hold the device address >| */
	uint32_t			RxSize;			/* |< Hold the I2C receive size >| */
	uint8_t				Sr;				/* |< Hold the I2C repeated start status >| */
}I2C_Handle_t;

/*
 * @I2C_SCLSpeed
 */
#define I2C_SCL_SPEED_SM	100000U
#define I2C_SCL_SPEED_FM4K	400000U
#define I2C_SCL_SPEED_FM2K	200000U

/*
 * @I2C_AckControl
 */
#define I2C_ACK_ENABLE	1
#define I2C_ACK_DISABLE	0

/*
 * @I2C_FMDutyCycle
 */
#define I2C_FM_DUTY_2		0
#define I2C_FM_DUTY_16_9	1

/*
 * @TxRxState
 */
#define I2C_READY		0
#define I2C_BUSY_IN_TX	1
#define I2C_BUSY_IN_RX	2

/*
 * @I2C application callback event state
 */
#define I2C_EV_TX_DONE	0
#define I2C_EV_RX_DONE	1
#define I2C_EV_STOPF	2
#define I2C_ERROR_BERR  3
#define I2C_ERROR_ARLO  4
#define I2C_ERROR_AF    5
#define I2C_ERROR_OVR   6
#define I2C_ERROR_TIMEOUT 7
#define I2C_EV_DATA_REQ 8
#define I2C_EV_DATA_RECV 9

/************************************************************************************
 * 						APIs supported by this driver
 * 			For more information about the APIs check the function definitions
 ************************************************************************************/

/*
 * Peripheral clock setup
 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

/*
 * Init and De-init
 */
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);


/*
 * Data send and receive
 */

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);

void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx, uint8_t data);
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx);

uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);

void I2C_SlaveEnableCallbackEvent(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

/*
 * IRQ configuration and ISR handling
 */
void I2C_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi);
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle);

/*
 * Other configuration APIs
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);
uint8_t I2C_GetFlagStatus(uint32_t reg, uint32_t bit);
void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle);
void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle);
void I2C_GenerateStop(I2C_RegDef_t* pI2Cx);


/*
 * Application callback
 */
void I2C_ApplicationEventCallback(I2C_Handle_t* pI2CHandle, uint8_t AppEvent);

#endif /* INC_STM32F46XX_I2C_DRIVER_H_ */
