/*
 * stm32f46xx_rcc_driver.c
 *
 *  Created on: 16-Jan-2021
 *      Author: saksh
 */

#include "stm32f46xx_rcc_driver.h"

uint16_t GetPLLOutput(void) {
	//TODO: To be implemented
	return 0;
}

uint32_t RCC_GetPclk1Val(void) {

	uint32_t pclk_val;
	uint8_t sysclk_src = (RCC->CFGR >> 2) & 0x3;
	uint32_t SysClk;
	uint16_t ahb_prescaler;
	uint16_t apb1_prescaler;
	switch(sysclk_src) {
		case 0x00:				//HSI oscillator
			SysClk = SYSCLK_SPEED_HSI;
			break;
		case 0x01:				//HSE oscillator
			SysClk = SYSCLK_SPEED_HSE;
			break;
		default:
			SysClk = GetPLLOutput();
	}

	//AHB prescaler calculate
	if((RCC->CFGR >> 7) & 0x1) {
		switch((RCC->CFGR >> RCC_CFGR_HPRE) & 0x7) {
			case 0x0:
				ahb_prescaler = 2;
				break;
			case 0x1:
				ahb_prescaler = 4;
				break;
			case 0x2:
				ahb_prescaler = 8;
				break;
			case 0x3:
				ahb_prescaler = 16;
				break;
			case 0x4:
				ahb_prescaler = 64;
				break;
			case 0x5:
				ahb_prescaler = 128;
				break;
			case 0x6:
				ahb_prescaler = 256;
				break;
			case 0x7:
				ahb_prescaler = 512;
				break;
		}
	}
	else {
		ahb_prescaler = 1;
	}

	//APB1 prescaler calculate
	if((RCC->CFGR >> 12) & 0x1) {
		switch((RCC->CFGR >> RCC_CFGR_PPRE1) & 0x3) {
			case 0x0:
				apb1_prescaler = 2;
				break;
			case 0x1:
				apb1_prescaler = 4;
				break;
			case 0x2:
				apb1_prescaler = 8;
				break;
			case 0x3:
				apb1_prescaler = 16;
				break;
		}
	}
	else {
		apb1_prescaler = 1;
	}

	pclk_val = (SysClk/ahb_prescaler)/apb1_prescaler;
	return pclk_val;
}

uint32_t RCC_GetPclk2Val(void) {

	uint32_t pclk_val;
	uint8_t sysclk_src = (RCC->CFGR >> 2) & 0x3;
	uint32_t SysClk;
	uint16_t ahb_prescaler;
	uint16_t apb2_prescaler;
	switch(sysclk_src) {
		case 0x00:				//HSI oscillator
			SysClk = SYSCLK_SPEED_HSI;
			break;
		case 0x01:				//HSE oscillator
			SysClk = SYSCLK_SPEED_HSE;
			break;
		default:
			SysClk = GetPLLOutput();
	}

	//AHB prescaler calculate
	if((RCC->CFGR >> 7) & 0x1) {
		switch((RCC->CFGR >> RCC_CFGR_HPRE) & 0x7) {
			case 0x0:
				ahb_prescaler = 2;
				break;
			case 0x1:
				ahb_prescaler = 4;
				break;
			case 0x2:
				ahb_prescaler = 8;
				break;
			case 0x3:
				ahb_prescaler = 16;
				break;
			case 0x4:
				ahb_prescaler = 64;
				break;
			case 0x5:
				ahb_prescaler = 128;
				break;
			case 0x6:
				ahb_prescaler = 256;
				break;
			case 0x7:
				ahb_prescaler = 512;
				break;
		}
	}
	else {
		ahb_prescaler = 1;
	}

	//APB2 prescaler calculate
	if((RCC->CFGR >> 15) & 0x1) {
		switch((RCC->CFGR >> RCC_CFGR_PPRE2) & 0x3) {
			case 0x0:
				apb2_prescaler = 2;
				break;
			case 0x1:
				apb2_prescaler = 4;
				break;
			case 0x2:
				apb2_prescaler = 8;
				break;
			case 0x3:
				apb2_prescaler = 16;
				break;
		}
	}
	else {
		apb2_prescaler = 1;
	}

	pclk_val = (SysClk/ahb_prescaler)/apb2_prescaler;
	return pclk_val;
}
