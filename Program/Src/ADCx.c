#include <ADCx.h>

void ADC_Init()
{
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;//ТАКТИРОВАНИЕ ADC1
  RCC->CFGR |= 0b10<<RCC_CFGR_ADCPRE_Pos;// предделитель 6

  ADC1->CR2 &= ~ADC_CR2_ADON; // запретить АЦП

  ADC1->CR1 = 0;      // запрещаем все в управляющих регистрах
  ADC1->CR2 = 0;

  ADC1->CR2 &= ~ADC_CR2_CONT; // запрет непрерывного режима
  ADC1->CR1 &= ~ADC_CR1_SCAN; // запрет режима сканирования

  ADC1->SMPR2 |= 0b111<<ADC_SMPR2_SMP0_Pos; //Задаем длительность выборки для 0 канала АЦП1 0.5 циклов

  ADC1->SQR1 |= 0b0000<<ADC_SQR1_L_Pos; //устанавливаем, что будет опрашиваться один регулярный канал
  ADC1->SQR3 |= 0b0000<<ADC_SQR3_SQ1_Pos; //опрашивается 0 канал

  ADC1->CR2 |= 0b111<<ADC_CR2_EXTSEL_Pos | ADC_CR2_EXTTRIG; //разрешаем запуск от SWSTART и программный запуск
  ADC1->CR2 |= ADC_CR2_ADON; // разрешить АЦП
  ADC1->CR2 |= ADC_CR2_CAL;//Запуск калибровки АЦП
  while ((ADC1->CR2 & ADC_CR2_CAL));//Ожидаем окончания калибровки
}
/*float ADC_Read()
{
  float res=0;
  ADC1->CR2 |= ADC_CR2_SWSTART; //однократный запуск преобразования
  while(!(ADC1->SR & ADC_SR_EOC)) ; // ожидание завершения преобразования
  res= (float)ADC1->DR * 3.3f / 4096. ; // пересчет в напряжение разрядность АЦП 12 бит
  return res;
}*/
uint16_t ADC_Read()
{
  ADC1->CR2 |= ADC_CR2_SWSTART; //однократный запуск преобразования
  while(!(ADC1->SR & ADC_SR_EOC)) ; // ожидание завершения преобразования
  return ADC1->DR;//Пересылка значения АЦП
}
