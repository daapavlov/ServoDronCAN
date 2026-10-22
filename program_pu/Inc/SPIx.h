#include "stm32f1xx_hal.h"


#ifndef SPIX_H_
#define SPIX_H_

void SPI1_Init(void);
uint8_t SPI1_Transmit(uint8_t *Buffer, uint8_t Amount);
void SPI_Delay_Cycles(uint32_t cycles);
#endif /* SPIX_H_ */
