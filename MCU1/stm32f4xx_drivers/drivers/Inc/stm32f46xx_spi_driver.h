/*
 * stm32f46xx_spi_driver.h
 *
 *  Created on: 07-Nov-2020
 *      Author: saksh
 */

#ifndef INC_STM32F46XX_SPI_DRIVER_H_
#define INC_STM32F46XX_SPI_DRIVER_H_


#include "stm32f46xx.h"

typedef struct
{
	uint8_t SPI_DeviceMode;				/* |< possible values from @SPI_DEVICE_MODES >*/
	uint8_t SPI_BusConfig;				/* |< possible values from @SPI_BUS_CONFIG >*/
	uint8_t SPI_DFF;					/* |< possible values from @SPI_DFF >*/
	uint8_t SPI_CPHA;					/* |< possible values from @SPI_CPHA >*/
	uint8_t SPI_CPOL;					/* |< possible values from @SPI_CPOL >*/
	uint8_t SPI_SSM;					/* |< possible values from @SPI_SSM >*/
	uint8_t SPI_SclkSpeed;				/* |< possible values from @SPI_CLK_SPEED >*/

}SPI_Config_t;

/*
 * Handle structure of SPI
 */

typedef struct {
	SPI_RegDef_t		*pSPIx;			/* |< Hold the base address of SPIx(x:0,1,2) peripheral >| */
	SPI_Config_t		SPI_Config;
	uint8_t				*pTxBuffer;		/* |< Hold the buffer address while sending data >| */
	uint8_t				*pRxBuffer;		/* |< Hold the buffer address while receiving data >| */
	uint32_t			TxLen;			/* |< Hold the remaining length of data during send >| */
	uint32_t			RxLen;			/* |< Hold the remaining length of data during receive >| */
	uint32_t			TxState;		/* |< Hold the SPI send status >| */
	uint32_t			RxState;		/* |< Hold the SPI receive status >| */
}SPI_Handle_t;


/*
 * @SPI_DEVICE_MODES
 * SPI possible modes
 */
#define SPI_DEVICE_MODE_MASTER			1
#define SPI_DEVICE_MODE_SLAVE			0

/*
 * @SPI_BUS_CONFIG
 * SPI different bus configuration
 */
#define SPI_CONFIG_FD					0
#define SPI_CONFIG_HD_TX				1
#define SPI_CONFIG_HD_RX				2
#define SPI_CONFIG_SIMPLEX_RX			3

/*
 * @SPI_CLK_SPEED
 * SPI different clock speeds
 */
#define SPI_SCLK_SPEED_DIV2					0
#define SPI_SCLK_SPEED_DIV4					1
#define SPI_SCLK_SPEED_DIV8					2
#define SPI_SCLK_SPEED_DIV16				3
#define SPI_SCLK_SPEED_DIV32				4
#define SPI_SCLK_SPEED_DIV64				5
#define SPI_SCLK_SPEED_DIV128				6
#define SPI_SCLK_SPEED_DIV256				7

/*
 * @SPI_DFF
 */
#define SPI_DFF_8BITS	0
#define SPI_DFF_16BITS	1

/*
 * @SPI_CPHA
 */
#define SPI_CPHA_HIGH	1
#define SPI_CPHA_LOW	0

/*
 * @SPI_CPOL
 */
#define SPI_CPOL_HIGH	1
#define SPI_CPOL_LOW	0

/*
 * @SPI_SSM
 */
#define SPI_SSM_DI	0
#define SPI_SSM_EN	1

/*
 * @TxState
 */
#define	SPI_READY		0
#define	SPI_BUSY_RX		1
#define	SPI_BUSY_TX		2

/*
 * SPI possible events
 */
#define SPI_EVENT_TX_CMPLT	0
#define SPI_EVENT_RX_CMPLT	1
#define SPI_EVENT_OVRERR	2

/************************************************************************************
 * 						APIs supported by this driver
 * 			For more information about the APIs check the function definitions
 ************************************************************************************/

/*
 * Peripheral clock setup
 */

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

/*
 * Init and De-init
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);


/*
 * Data send and receive
 */

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len);

/*
 * IRQ configuration and ISR handling
 */
void SPI_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);

/*
 * Other configuration APIs
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);

/*
 * Application callback
 */
void SPI_ApplicationEventCallback(SPI_Handle_t* pSPIHandle, uint8_t AppEvent);

#endif /* INC_STM32F46XX_SPI_DRIVER_H_ */
