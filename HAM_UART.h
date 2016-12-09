/*
 * HAM_UART.h
 *
 *  Created on: Dec 1, 2016
 *      Author: Daniel
 */

#ifndef HAM_UART_H_
#define HAM_UART_H_

#include "stm32f7xx.h"

#define HAM_UART_getBaud(clk, baudrate)	\
	((uint16_t) (clk/baudrate))

#define HAM_UART_init(UARTx, baud)	\
	UARTx->CR1 = USART_CR1_TE | USART_CR1_RE;\
	UARTx->BRR = baud

#define HAM_UART_enable(UARTx)	\
	UARTx->CR1|= USART_CR1_UE

#define HAM_UART_disable(UARTx)	\
	UARTx->CR1&= ~USART_CR1_UE;

#define HAM_UART_enableIT(UARTx) \
	UARTx->CR1|= USART_CR1_RXNEIE | USART_CR1_TXEIE

#define HAM_UART_disableIT(UARTx) \
	UARTx->CR1&= ~USART_CR1_RXNEIE & ~USART_CR1_TXEIE

#define HAM_UART_enableITRx(UARTx) \
	UARTx->CR1|= USART_CR1_RXNEIE

#define HAM_UART_disableITRx(UARTx) \
	UARTx->CR1&= ~USART_CR1_RXNEIE

#define HAM_UART_enableITTx(UARTx) \
	UARTx->CR1|= USART_CR1_TXEIE

#define HAM_UART_disableITTx(UARTx) \
	UARTx->CR1&= ~USART_CR1_TXEIE

#define HAM_UART_logicInvert(UARTx) \
	UARTx->CR2 = USART_CR2_TXINV | USART_CR2_RXINV

#define HAM_UART_RS232(UARTx)	\
	UARTx->CR3 = USART_CR3_CTSE | USART_CR3_RTSE


#define HAM_UART1_CLK()	\
	RCC->APB2ENR|= RCC_APB2ENR_USART1EN

#define HAM_UART2_CLK()	\
	RCC->APB1ENR|= RCC_APB1ENR_USART2EN

#define HAM_UART3_CLK()	\
	RCC->APB1ENR|= RCC_APB1ENR_USART3EN

#define HAM_UART4_CLK()	\
	RCC->APB1ENR|= RCC_APB1ENR_UART4EN

#define HAM_UART5_CLK()	\
	RCC->APB1ENR|= RCC_APB1ENR_UART5EN

#define HAM_UART6_CLK()	\
	RCC->APB2ENR|= RCC_APB2ENR_USART6EN

#define HAM_UART7_CLK()	\
	RCC->APB1ENR|= RCC_APB1ENR_UART7EN

#define HAM_UART8_CLK()	\
	RCC->APB1ENR|= RCC_APB1ENR_UART8EN



#endif /* HAM_UART_H_ */
