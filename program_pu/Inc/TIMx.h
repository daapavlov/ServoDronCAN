#include "stm32f1xx_hal.h"


#ifndef TIMX_H_
#define TIMX_H_

#define	TIM_Start(TIMx)	TIMx->CR1 |= TIM_CR1_CEN; //ВКЛЮЧЕНИЕ ТАЙМЕРА
#define	TIM_Stop(TIMx)	TIMx->CR1 &= ~TIM_CR1_CEN; //ВЫКЛЮЧЕНИЕ ТАЙМЕРА


typedef struct TIM_Flag
{
  uint8_t TIM_IT;
  uint8_t TIM_Counter_IT;
  uint16_t TIM_mSecond_Counter_IT;
  uint8_t TIM_Counter_ADC;
}TIM_Flag;

void TIM1_Init(void);
void TIM2_Init(void);
void TIM3_Init(void);
void TIM4_Init();

#endif
