/*
 * spi_cmd_response.c
 *
 *  Created on: 29-Nov-2020
 *      Author: saksh
 */

#include "stm32f46xx.h"
#include <string.h>
#include <stdio.h>

/*
 * SPI2 mapping on GPIO pins
 * PB12 --> SPI2_NSS
 * PB13 --> SPI2_SCK
 * PB14 --> SPI2_MISO
 * PB15 --> SPI2_MOSI
 * Alt fn mode: 5
 */

//command codes
#define COMMAND_LED_CTRL 		0x50
#define COMMAND_SENSOR_READ 	0x51
#define COMMAND_LED_READ		0x52
#define COMMAND_PRINT			0x53
#define COMMAND_ID_READ			0x54

//LED status
#define LED_ON	1
#define LED_OFF	0

//Arduino analog pins
#define ANALOG_PIN0		0
#define ANALOG_PIN1		1
#define ANALOG_PIN2		2
#define ANALOG_PIN3		3
#define ANALOG_PIN4		4

//LED connection on Arduino
#define LED_PIN	9

//Acknowledge message
#define ACK		0xF5
#define NACK	0xA5

void delay(void) {
	for(int i=0; i<500000/2; i++);
}

uint8_t board_id[10];

extern void initialise_monitor_handles();

void GPIO_SPI2config(GPIO_Handle_t* pGPIO_SPI) {

	pGPIO_SPI->pGPIOx = GPIOB;
	pGPIO_SPI->GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	pGPIO_SPI->GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	pGPIO_SPI->GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	pGPIO_SPI->GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU;
	pGPIO_SPI->GPIO_PinConfig.GPIO_PinSpeed = GPIO_OP_SPEED_FAST;

	//SPI2_NSS
	pGPIO_SPI->GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(pGPIO_SPI);

	//SPI2_SCK
	pGPIO_SPI->GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(pGPIO_SPI);

	//SPI2_MISO
	pGPIO_SPI->GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(pGPIO_SPI);

	//SPI2_MOSI
	pGPIO_SPI->GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(pGPIO_SPI);

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

void CmdLedCtrl(SPI_Handle_t* pSPIHandle) {
	uint8_t cmdcode = COMMAND_LED_CTRL;
	uint8_t dummy_read;
	uint8_t dummy_write = 0xff;
	uint8_t ack;
	uint8_t args[2];

	//Send command code
	SPI_SendData(pSPIHandle->pSPIx, &cmdcode, 1);

	//Do dummy read to empty the Rx Buffer which got filled due to last send(clear off RXNE	)
	SPI_ReceiveData(pSPIHandle->pSPIx, &dummy_read, 1);

	//Send dummy data to get response of the slave in Rx Buffer
	SPI_SendData(pSPIHandle->pSPIx, &dummy_write, 1);

	//Read acknowledge
	SPI_ReceiveData(pSPIHandle->pSPIx, &ack, 1);

	if(ack == ACK) {
		//Slave has acknowledges the command, send command arguments
		args[0] = LED_PIN;
		args[1] = LED_ON;
		SPI_SendData(pSPIHandle->pSPIx, args, 2);
	}

}

uint8_t CmdSensorRead(SPI_Handle_t* pSPIHandle) {
	uint8_t cmdcode = COMMAND_SENSOR_READ;
	uint8_t dummy_read;
	uint8_t dummy_write = 0xff;
	uint8_t ack;
	uint8_t args[2];
	uint8_t sensor_val;

	//Send command code
	SPI_SendData(pSPIHandle->pSPIx, &cmdcode, 1);

	//Do dummy read to empty the Rx Buffer which got filled due to last send(clear off RXNE	)
	SPI_ReceiveData(pSPIHandle->pSPIx, &dummy_read, 1);

	//Send dummy data to get response of the slave in Rx Buffer
	SPI_SendData(pSPIHandle->pSPIx, &dummy_write, 1);

	//Read acknowledge
	SPI_ReceiveData(pSPIHandle->pSPIx, &ack, 1);

	if(ack == ACK) {

		//Slave has acknowledges the command, send command arguments
		args[0] = ANALOG_PIN0;
		SPI_SendData(pSPIHandle->pSPIx, args, 1);

		//Do dummy read to clear RXNE
		SPI_ReceiveData(pSPIHandle->pSPIx, &dummy_read, 1);

		//Add delay before reading analog value
		delay();

		//do dummy send to get analog pin value in Rx Buffer
		SPI_SendData(pSPIHandle->pSPIx, &dummy_write, 1);

		//Read analog pin value from Rx Buffer
		SPI_ReceiveData(pSPIHandle->pSPIx, &sensor_val, 1);

		return sensor_val;
	}

	return 0;
}

uint8_t CmdLedRead(SPI_Handle_t* pSPIHandle) {
	uint8_t cmdcode = COMMAND_LED_READ;
	uint8_t dummy_read;
	uint8_t dummy_write = 0xff;
	uint8_t ack;
	uint8_t args[2];
	uint8_t led_val;

	//Send command code
	SPI_SendData(pSPIHandle->pSPIx, &cmdcode, 1);

	//Do dummy read to empty the Rx Buffer which got filled due to last send(clear off RXNE	)
	SPI_ReceiveData(pSPIHandle->pSPIx, &dummy_read, 1);

	//Send dummy data to get response of the slave in Rx Buffer
	SPI_SendData(pSPIHandle->pSPIx, &dummy_write, 1);

	//Read acknowledge
	SPI_ReceiveData(pSPIHandle->pSPIx, &ack, 1);

	if(ack == ACK) {

		//Slave has acknowledges the command, send command arguments
		args[0] = LED_PIN;
		SPI_SendData(pSPIHandle->pSPIx, args, 1);

		//Do dummy read to clear RXNE
		SPI_ReceiveData(pSPIHandle->pSPIx, &dummy_read, 1);

		//Add delay before reading LED value
		delay();

		//do dummy send to get LED pin value in Rx Buffer
		SPI_SendData(pSPIHandle->pSPIx, &dummy_write, 1);

		//Read analog pin value from Rx Buffer
		SPI_ReceiveData(pSPIHandle->pSPIx, &led_val, 1);

		return led_val;
	}

	return 0;
}

void CmdPrint(SPI_Handle_t* pSPIHandle, char* user_str) {
	uint8_t cmdcode = COMMAND_PRINT;
	uint8_t dummy_read;
	uint8_t dummy_write = 0xff;
	uint8_t ack;
	uint8_t args[2];

	//Send command code
	SPI_SendData(pSPIHandle->pSPIx, &cmdcode, 1);

	//Do dummy read to empty the Rx Buffer which got filled due to last send(clear off RXNE	)
	SPI_ReceiveData(pSPIHandle->pSPIx, &dummy_read, 1);

	//Send dummy data to get response of the slave in Rx Buffer
	SPI_SendData(pSPIHandle->pSPIx, &dummy_write, 1);

	//Read acknowledge
	SPI_ReceiveData(pSPIHandle->pSPIx, &ack, 1);

	if(ack == ACK) {
		//Slave has acknowledges the command, send command arguments
		args[0] = (uint8_t)strlen(user_str);
		SPI_SendData(pSPIHandle->pSPIx, args, 1);

		SPI_SendData(pSPIHandle->pSPIx, (uint8_t*)user_str, args[0]);
	}

}

void CmdIDRead(SPI_Handle_t* pSPIHandle) {
	uint8_t cmdcode = COMMAND_ID_READ;
	uint8_t dummy_read;
	uint8_t dummy_write;
	uint8_t ack;

	//Send command code
	SPI_SendData(pSPIHandle->pSPIx, &cmdcode, 1);

	//Do dummy read to empty the Rx Buffer which got filled due to last send(clear off RXNE	)
	SPI_ReceiveData(pSPIHandle->pSPIx, &dummy_read, 1);

	//Send dummy data to get response of the slave in Rx Buffer
	SPI_SendData(pSPIHandle->pSPIx, &dummy_write, 1);

	//Read acknowledge
	SPI_ReceiveData(pSPIHandle->pSPIx, &ack, 1);

	if(ack == ACK) {
		//Get 10 bytes of data from slave
		for(int i=0; i<10; i++) {
			//Slave has acknowledges the command, do dummy send to get board ID
			SPI_SendData(pSPIHandle->pSPIx, &dummy_write, 1);

			SPI_ReceiveData(pSPIHandle->pSPIx, &board_id[i], 1);
		}
	}
}

int main() {

	GPIO_Handle_t GPIO_SPI;
	GPIO_Handle_t GPIOBtn;
	SPI_Handle_t SPI2Master;
	uint8_t sensor_val;
	uint8_t led_val;
	char user_str[] = "Testing CMD_PRINT successful !!";

	initialise_monitor_handles();

	GPIO_SPI2config(&GPIO_SPI);
	GPIO_BtnConfig(&GPIOBtn);

	SPI2Master.pSPIx = SPI2;
	SPI2Master.SPI_Config.SPI_BusConfig = SPI_CONFIG_FD;
	SPI2Master.SPI_Config.SPI_CPHA = SPI_CPHA_LOW;
	SPI2Master.SPI_Config.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Master.SPI_Config.SPI_DFF = SPI_DFF_8BITS;
	SPI2Master.SPI_Config.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Master.SPI_Config.SPI_SSM = SPI_SSM_DI;
	SPI2Master.SPI_Config.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8;		//Generate 2MHz clock

	SPI_Init(&SPI2Master);

	printf("Application is running\n");

	while(1) {

		SPI_SSOEConfig(SPI2Master.pSPIx, ENABLE);
//---------- 1. Command Led control ------------------------//
		//Wait for button to be pressed
		while(GPIO_ReadFromInputPin(GPIOBtn.pGPIOx, 13));
		delay();								//debouncing

		SPI_PeripheralControl(SPI2Master.pSPIx, ENABLE);

		printf("Executing CMD_LED_CTRL\n");
		//Execute CMD_LED_CTRL
		CmdLedCtrl(&SPI2Master);

		//Get busy flag status
		while((SPI2Master.pSPIx->SR & (1 << SPI_SR_BSY)));
		SPI_PeripheralControl(SPI2Master.pSPIx, DISABLE);

//---------------- 2. Command sensor read -------------------//

		//Wait for button to be pressed
		while(GPIO_ReadFromInputPin(GPIOBtn.pGPIOx, 13));
		delay();								//debouncing

		SPI_PeripheralControl(SPI2Master.pSPIx, ENABLE);

		printf("Executing CMD_SENSOR_READ\n");
		//Execute CMD_SENSOR_READ
		sensor_val = CmdSensorRead(&SPI2Master);
		//CmdSensorRead(&SPI2Master);
		printf("Value of analog pin: %d\n",sensor_val);

		//Get busy flag status
		while((SPI2Master.pSPIx->SR & (1 << SPI_SR_BSY)));
		SPI_PeripheralControl(SPI2Master.pSPIx, DISABLE);

//--------------- 3. Command led status read -----------------//

		//Wait for button to be pressed
		while(GPIO_ReadFromInputPin(GPIOBtn.pGPIOx, 13));
		delay();								//debouncing

		SPI_PeripheralControl(SPI2Master.pSPIx, ENABLE);

		printf("Executing CMD_LED_READ\n");
		//Execute CMD_LED_READ
		led_val = CmdLedRead(&SPI2Master);
		//CmdLedRead(&SPI2Master);
		printf("Value of LED pin: %d\n",led_val);

		//Get busy flag status
		while((SPI2Master.pSPIx->SR & (1 << SPI_SR_BSY)));
		SPI_PeripheralControl(SPI2Master.pSPIx, DISABLE);

//---------------- 4. Command print ---------------------------//

		//Wait for button to be pressed
		while(GPIO_ReadFromInputPin(GPIOBtn.pGPIOx, 13));
		delay();								//debouncing

		SPI_PeripheralControl(SPI2Master.pSPIx, ENABLE);

		printf("Executing CMD_PRINT\n");
		//Execute CMD_PRINT
		CmdPrint(&SPI2Master, user_str);

		//Get busy flag status
		while((SPI2Master.pSPIx->SR & (1 << SPI_SR_BSY)));
		SPI_PeripheralControl(SPI2Master.pSPIx, DISABLE);

//-------------- 5. Command ID read --------------------------//

		//Wait for button to be pressed
		while(GPIO_ReadFromInputPin(GPIOBtn.pGPIOx, 13));
		delay();								//debouncing

		SPI_PeripheralControl(SPI2Master.pSPIx, ENABLE);

		printf("Executing CMD_ID_READ\n");
		//Execute CMD_ID_READ
		CmdIDRead(&SPI2Master);
		printf("ID received: %s\n",board_id);

		//Get busy flag status
		while((SPI2Master.pSPIx->SR & (1 << SPI_SR_BSY)));
		SPI_PeripheralControl(SPI2Master.pSPIx, DISABLE);

		printf("Application ended\n");
	}

	return 0;
}
