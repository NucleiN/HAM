/*
 * HAM_GPIO.h
 *
 *  Created on: Nov 25, 2016
 *      Author: Daniel
 */

#ifndef HAM_GPIO_H_
#define HAM_GPIO_H_

#include "stm32f7xx.h"

		//OTYPER
#define HAM_GPIO_TYPEPP						0x0
#define HAM_GPIO_TYPEOD						0x1
		//OSPEEDR
#define HAM_GPIO_LOWSPEED					0x0
#define HAM_GPIO_MEDIUMSPEED				0x1
#define HAM_GPIO_HIGHSPEED					0x2
#define HAM_GPIO_VHIGHSPEED					0x3
		//PUPDR
#define HAM_GPIO_NOPULL						0x0
#define HAM_GPIO_PULLUP						0x1
#define HAM_GPIO_PULLDOWN					0x2

#define xHAM_GPIO_MODER(GPIOx,PIN,VAL)	\
	GPIOx->MODER|= (VAL<<(PIN*0x2))

#define xHAM_GPIO_OSPEEDR(GPIOx,PIN,VAL)	\
	GPIOx->OSPEEDR|= (VAL<<(PIN*0x2))

#define xHAM_GPIO_OTYPER(GPIOx,PIN,VAL)	\
	GPIOx->OTYPER|= (VAL<<PIN)

#define xHAM_GPIO_PUPDR(GPIOx,PIN,VAL)	\
	GPIOx->PUPDR|= (VAL<<(PIN*0x2))

#define xHAM_GPIO_AFR(GPIOx,PIN,VAL)	\
	GPIOx->AFR[0]|= (PIN < 0x8) ? (VAL<<(PIN*0x4)) : 0;\
	GPIOx->AFR[1]|= (PIN < 0x8) ? 0 : (VAL<<((PIN-0x8)*0x4))

		//Init GPIO input
#define HAM_GPIO_INPUT_init(GPIOx,PIN,PULL)	\
	xGPIO_PUPDR(GPIOx,PIN,PULL)
		//Init GPIO output
#define HAM_GPIO_OUTPUT_init(GPIOx,PIN,SPEED,TYPE,PULL)	\
	xHAM_GPIO_MODER(GPIOx,PIN,0x1);\
	xHAM_GPIO_OSPEEDR(GPIOx,PIN,SPEED);\
	xHAM_GPIO_OTYPER(GPIOx,PIN,TYPE);\
	xHAM_GPIO_PUPDR(GPIOx,PIN,PULL)
		//Init GPIO alternate function
#define HAM_GPIO_AF_init(GPIOx,PIN,AF,SPEED,TYPE,PULL)	\
	xHAM_GPIO_MODER(GPIOx,PIN,0x2);\
	xHAM_GPIO_AFR(GPIOx,PIN,AF);\
	xHAM_GPIO_OSPEEDR(GPIOx,PIN,SPEED);\
	xHAM_GPIO_OTYPER(GPIOx,PIN,TYPE);\
	xHAM_GPIO_PUPDR(GPIOx,PIN,PULL)
		//Init GPIO analog
#define HAM_GPIO_ANALOG_init(GPIOx,PIN)	\
	xHAM_GPIO_MODER(GPIOx,PIN,0x3)
		//Deinit GPIO
#define HAM_GPIO_deinit(GPIOx,PIN)	\
	GPIOx->MODER&= ~(0x3<<(PIN*0x2));\
	GPIOx->AFR[0]&= (PIN < 0x8) ? ~(0xFF<<(PIN*0x4)) : 0xFFFFFFFF;\
	GPIOx->AFR[1]&= (PIN < 0x8) ? 0xFFFFFFFF : ~(0xFF<<((PIN-0x8)*0x4));\
	GPIOx->OSPEEDR&= ~(0x3<<(PIN*0x2));\
	GPIOx->OTYPER&= ~PIN;\
	GPIOx->PUPDR&= ~(0x3<<(PIN*0x2))

		//Get GPIO pin input value
#define HAM_GPIO_getInput(GPIOx,PIN)	\
	(GPIOx->IDR & PIN)
		//Get GPIO pin current output value
#define HAM_GPIO_getOutput(GPIOx,PIN)	\
	(GPIOx->ODR & PIN)
		//Set GPIO pin output
#define HAM_GPIO_setOutput(GPIOx,PIN,SET)	\
	GPIOx->BSRR|= (SET) ? PIN : PIN+0x10

		//Enable GPIO clocks
#define HAM_GPIOA_CLK()	\
	RCC->AHB1ENR|= RCC_AHB1ENR_GPIOAEN
#define HAM_GPIOB_CLK()	\
	RCC->AHB1ENR|= RCC_AHB1ENR_GPIOBEN
#define HAM_GPIOC_CLK()	\
	RCC->AHB1ENR|= RCC_AHB1ENR_GPIOCEN
#define HAM_GPIOD_CLK()	\
	RCC->AHB1ENR|= RCC_AHB1ENR_GPIODEN
#define HAM_GPIOE_CLK()	\
	RCC->AHB1ENR|= RCC_AHB1ENR_GPIOEEN
#define HAM_GPIOF_CLK()	\
	RCC->AHB1ENR|= RCC_AHB1ENR_GPIOFEN
#define HAM_GPIOG_CLK()	\
	RCC->AHB1ENR|= RCC_AHB1ENR_GPIOGEN
#define HAM_GPIOH_CLK()	\
	RCC->AHB1ENR|= RCC_AHB1ENR_GPIOHEN
#define HAM_GPIOI_CLK()	\
	RCC->AHB1ENR|= RCC_AHB1ENR_GPIOIEN
#define HAM_GPIOJ_CLK()	\
	RCC->AHB1ENR|= RCC_AHB1ENR_GPIOJEN
#define HAM_GPIOK_CLK()	\
	RCC->AHB1ENR|= RCC_AHB1ENR_GPIOKEN

#endif /* HAM_GPIO_H_ */
