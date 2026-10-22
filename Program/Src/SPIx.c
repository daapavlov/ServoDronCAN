#include <SPIx.h>

void SPI1_Init()
{
  RCC->APB2ENR |= RCC_APB2ENR_SPI1EN_Msk; //ENABLING ALTERNATIVE FUNCTIONS CLOCKING AND SPI1

  SPI1->CR1 |= 0b101<<SPI_CR1_BR_Pos;//Baud rate control f/256
  SPI1->CR1 |= SPI_CR1_BIDIMODE_Msk;//1-line bidirectional data mode selected
  SPI1->CR1 |= SPI_CR1_BIDIOE_Msk;//Output enabled
  SPI1->CR1 |= SPI_CR1_MSTR_Msk;//Master configuration
  SPI1->CR1 |= SPI_CR1_SSM_Msk;//Software slave management
  SPI1->CR1 |= SPI_CR1_SSI;
  SPI1->CR1 &= ~SPI_CR1_LSBFIRST_Msk; //MSB transmitted first
  SPI1->CR1 &= ~SPI_CR1_RXONLY_Msk;//Full duplex (Transmit and receive)
  SPI1->CR1 &= ~SPI_CR1_DFF_Msk;//8-bit data frame format is selected for transmission/reception
  SPI1->CR1 &= ~SPI_CR1_CPHA_Msk;//CPHA=1
  SPI1->CR1 &= ~SPI_CR1_CPOL_Msk;//CPOL=1

  SPI1->CR1 |= SPI_CR1_SPE_Msk;//SPI enable
}

uint8_t SPI1_Transmit(uint8_t *Buffer, uint8_t Amount)
{
  uint8_t FlagTimeOut=0;
  GPIOA->BSRR |= GPIO_BSRR_BS4;
  SPI_Delay_Cycles(72);
  GPIOA->BSRR |= GPIO_BSRR_BR4;
  SPI_Delay_Cycles(8);
  for(uint8_t i=0; i<Amount; i++)
  {
    // Передача в регистр данных элемента массива
    SPI1->DR = (uint8_t)Buffer[i];
    uint32_t	start_time = HAL_GetTick();
    // Ожидание освобождения буфера
    while((SPI1->SR & SPI_SR_TXE) == 0 && FlagTimeOut==0)
    {
      if((HAL_GetTick()-start_time)>10)
      {
          FlagTimeOut=1;
      }
    }
    FlagTimeOut=0;
  }
  //Ожидание окончания передачи
  FlagTimeOut=0;
  uint32_t start_time = HAL_GetTick();
  while((SPI1->SR & SPI_SR_BSY) > 0 && FlagTimeOut==0)
  {
      if((HAL_GetTick()-start_time)>10)
      {
          FlagTimeOut=1;
      }
  }
  
  SPI_Delay_Cycles(8);
  GPIOA->BSRR |= GPIO_BSRR_BS4;  
  GPIOB->BSRR |= GPIO_BSRR_BR2;
  SPI_Delay_Cycles(8);
  GPIOB->BSRR |= GPIO_BSRR_BS2;
  GPIOA->BSRR |= GPIO_BSRR_BR4;
  //HAL_Delay(1);

  
  return 1;
}
void SPI_Delay_Cycles(uint32_t cycles) {
    volatile uint32_t cnt = cycles;
    while (cnt--);
}