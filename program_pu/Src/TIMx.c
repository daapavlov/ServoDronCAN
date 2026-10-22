#include <TIMx.h>


void TIM1_Init()
{

  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;//Timer 1 clock enable

  TIM1->CR1 = 0;
  TIM1->PSC = 720-1; //The counter clock frequency CK_CNT is equal to fCK_PSC / (PSC[15:0] + 1). 1MHz
  TIM1->ARR = 1000-1; //0.05s period counter
  TIM1->DIER |= TIM_DIER_UIE;//Update interrupt enable
  NVIC_EnableIRQ(TIM1_UP_IRQn);//ВКЛЮЧЕНИЕ ПРЕРЫВАНИЙ ОТ TIM1;
//  NVIC_SetPriority(TIM1_UP_IRQn);
}
void TIM2_Init()
{

  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;//Timer 2 clock enable

  TIM2->CR1 = 0;
  TIM2->PSC = 720-1; //The counter clock frequency CK_CNT is equal to fCK_PSC / (PSC[15:0] + 1). 1MHz
  TIM2->ARR = 410; //5ms period counter
  TIM2->DIER |= TIM_DIER_UIE;//Update interrupt enable
  NVIC_EnableIRQ(TIM2_IRQn);//ВКЛЮЧЕНИЕ ПРЕРЫВАНИЙ ОТ TIM2;
//  NVIC_SetPriority(TIM2_IRQn);
}
void TIM3_Init()
{
  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;//Timer 3 clock enable

  TIM3->CR1 &= ~TIM_CR1_UDIS;//Генерировать событие Update
  TIM3->CR1 &= ~TIM_CR1_URS;//Генерировать прерывание
  TIM3->CR1 &= ~TIM_CR1_OPM;//One pulse mode off(Счетчик не останавливается при обновлении)
  TIM3->CR1 &= ~TIM_CR1_DIR;//Считаем вверх
  TIM3->CR1 |= 0b00<<TIM_CR1_CMS_Pos;//Выравнивание по краю
  TIM3->CR1 |= TIM_CR1_ARPE;//Auto-reload preload enable
  TIM3->CR1 |= 0b00<<TIM_CR1_CKD_Pos;//Предделение выключено

  TIM3->DIER |= TIM_DIER_UIE; //Update interrupt enable

  TIM3->PSC = 72-1;
  TIM3->ARR = 20000-1;

  NVIC_EnableIRQ(TIM3_IRQn); //Разрешить прерывания по таймеру 3
//  SET_BIT(TIM3->CR1, TIM_CR1_CEN);  //Запуск таймера


//  TIM3->CCER |= TIM_CCER_CC4E; //On - OC4 signal is output on the corresponding output pin.
//  TIM3->CCER |= TIM_CCER_CC4P; //OC4 active high.
  MODIFY_REG(TIM3->CCMR2, TIM_CCMR2_CC4S_Msk, 0b00 << TIM_CCMR2_CC4S_Pos); //CC4 channel is configured as output
  CLEAR_BIT(TIM3->CCMR2, TIM_CCMR2_OC4FE); //Fast mode disable
  SET_BIT(TIM3->CCMR2, TIM_CCMR2_OC4PE); //Preload enable
  MODIFY_REG(TIM3->CCMR2, TIM_CCMR2_OC4M_Msk, 0b110 << TIM_CCMR2_OC4M_Pos); //PWM MODE 1
  CLEAR_BIT(TIM3->CCMR2, TIM_CCMR2_OC4CE); //OC4Ref is not affected by the ETRF input

  SET_BIT(TIM3->CCER, TIM_CCER_CC4E); //On - OC1 signal is output on the corresponding output pin.
  SET_BIT(TIM3->CCER, TIM_CCER_CC4P); //OC1 active high.
}
void TIM4_Init()
{

  RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;//Timer 2 clock enable

  TIM4->CR1 = 0;
  TIM4->PSC = 7200-1; //The counter clock frequency CK_CNT is equal to fCK_PSC / (PSC[15:0] + 1). 1MHz
  TIM4->ARR = 10000-1; //5ms period counter
  TIM4->DIER |= TIM_DIER_UIE;//Update interrupt enable
  NVIC_EnableIRQ(TIM4_IRQn);//ВКЛЮЧЕНИЕ ПРЕРЫВАНИЙ ОТ TIM2;
//  NVIC_SetPriority(TIM2_IRQn);
}