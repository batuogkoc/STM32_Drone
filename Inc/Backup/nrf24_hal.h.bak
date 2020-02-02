#ifndef __NRF24_HAL_H
#define __NRF24_HAL_H


// Hardware abstraction layer for NRF24L01+ transceiver (hardware depended functions)
// GPIO pins definition
// GPIO pins initialization and control functions
// SPI transmit functions


// Peripheral libraries
#include "stm32f1xx_hal.h"
extern SPI_HandleTypeDef hspi2;
// SPI port peripheral
#define nRF24_SPI_PORT             &hspi2

// nRF24 GPIO peripherals
#define nRF24_GPIO_PERIPHERALS     (RCC_APB2ENR_IOPBEN)

// CE (chip enable) pin (PB11)
#define nRF24_CE_PORT              GPIOB
#define nRF24_CE_PIN               GPIO_PIN_11
#define nRF24_CE_L()               HAL_GPIO_WritePin(nRF24_CE_PORT,nRF24_CE_PIN,0)
#define nRF24_CE_H()               HAL_GPIO_WritePin(nRF24_CE_PORT,nRF24_CE_PIN,1)

// CSN (chip select negative) pin (PB12)
#define nRF24_CSN_PORT             GPIOB
#define nRF24_CSN_PIN              GPIO_PIN_12
#define nRF24_CSN_L()              HAL_GPIO_WritePin(nRF24_CSN_PORT,nRF24_CSN_PIN,0)
#define nRF24_CSN_H()              HAL_GPIO_WritePin(nRF24_CSN_PORT,nRF24_CSN_PIN,1)

// IRQ pin (PB10)
#define nRF24_IRQ_PORT             GPIOB
#define nRF24_IRQ_PIN              GPIO_PIN_10


// Function prototypes
void nRF24_GPIO_Init(void);
uint8_t nRF24_LL_RW(uint8_t data);

#endif // __NRF24_HAL_H
