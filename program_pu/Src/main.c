/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#include <TIMx.h>
#include <SPIx.h>
#include <ADCx.h>
#include <TypeConversion.h>

#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>


/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;
CAN_FilterTypeDef sFilterConfig;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM4_Init(void);

/* USER CODE BEGIN PFP */
void GPIO_Init(void);
uint8_t DefinitionSymbol(char Symbol);
void HAL_TIM1_Callback();
static uint16_t ADC_PROCESSING();
static void Flash_Read(uint32_t StartPagesAddress, uint16_t* Data, uint16_t number_of_words);
static void Flash_Write(uint32_t StartPagesAddress,uint16_t* Data, uint16_t quntity);
static uint16_t moving_average(uint16_t* data, uint16_t amountData);
static uint8_t PressKey_PB5();
static void SegmentSwitching(uint8_t counter);
static uint8_t ParametrDisplay(uint8_t NumberParametrDisplay);
uint8_t TransmitOneDisplay(char* string, uint8_t *IndicatorDigit, uint8_t FlagFloat);
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

#define AddressFlash    0x08007C00
/*------------------------------------------------------------------*/



/*------------------------------------------------------------------*/
typedef struct LED_INDICATOR
{
  uint8_t TransmitData[4];
  char StringSymbol[6];
  uint16_t DataBuffer;
}LED_INDICATOR;
/*------------------------------------------------------------------*/
#define CouADC	70U //Количество считываний АЦП для усреднения
typedef struct SERVO
{
  uint16_t ValueAngle;
  uint16_t MovAverageArray[CouADC];
}SERVO;
/*------------------------------------------------------------------*/

/*------------------------------------------------------------------*/
typedef struct 
{
    uint8_t ID;
    uint32_t ExtId;
    float position; //позиция в градусах от -60 до 60
    float force; //ток в амперах
    float speed; // скорость перемещения
    uint16_t CanRx_Data_12[16];
}Actuator;
/*------------------------------------------------------------------*/
/*создание экземпляров*/
LED_INDICATOR LED1;
SERVO ServoHand;
Actuator ServoPosition;
TIM_Flag TIM2_Flag;
TIM_Flag TIM1_Flag;
TIM_Flag TIM4_Flag;
/*------------------------------------------------------------------*/
const uint8_t quntityArrayParametrDisplayMAX = 2;
static uint8_t quntityArrayParametrDisplay=1;
static uint8_t quntityArrayParametrDisplay_mem=1;

uint8_t segment_counter=0;
volatile uint8_t numberParametrOutput=1;

uint16_t counterDelay=0;
uint8_t FlagEnableTransmit=0;
/*CAN*/
CAN_TxHeaderTypeDef TxHeader;
CAN_TxHeaderTypeDef TxHeader_inv;
CAN_RxHeaderTypeDef RxHeader;
uint8_t TxData[8] = {0x00,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0xFF};
uint8_t RxData[8] = {0,};
uint32_t TxMailbox = 0;
uint16_t counter_5ms=0;
uint16_t interputPB5_counter=0;

uint16_t invert = 0;
char Save[6] = {'S','A','V','E'};
uint8_t save_usRegistr=0;

uint16_t counterDelayDisplay = 0;
/* USER CODE END 0 */

int main(void)
{

  HAL_Init();
  TIM2_Flag.TIM_IT = 0;
  TIM2_Flag.TIM_Counter_IT = 1;
  SystemClock_Config();
  
  Flash_Read(AddressFlash, &invert, 1);
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_CAN_Init();
  RCC->APB2ENR |= RCC_APB2ENR_AFIOEN_Msk;
  GPIO_Init();
  TIM1_Init();
  TIM2_Init();
  TIM4_Init();
  HAL_Delay(2000);
  SPI1_Init();
  ADC_Init();
  
  TxHeader.StdId = 0;
  TxHeader.ExtId = 0x1803F27E;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.IDE = CAN_ID_EXT;
  TxHeader.DLC = 4;


  TIM_Start(TIM1);
  TIM_Start(TIM2);
  TIM_Start(TIM3);
  TIM_Start(TIM4);
  HAL_CAN_Start(&hcan);
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO1_MSG_PENDING | CAN_IT_ERROR| CAN_IT_LAST_ERROR_CODE | CAN_IT_BUSOFF );
  HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);
  while (1)
  {
    
      if(TIM2_Flag.TIM_IT == 1){  
          
          ServoHand.MovAverageArray[TIM2_Flag.TIM_mSecond_Counter_IT] = ADC_PROCESSING(); //заполняем массив для скользящего среднего
          ServoHand.ValueAngle = moving_average(ServoHand.MovAverageArray, CouADC);//находим скользящее среднее
          if(invert==0){
              ServoPosition.position = LinearTransformation_f(ServoHand.ValueAngle, 0, 120, -60.f, 60.f); //преобразуем в -60 - 60
          }
          else{
              ServoPosition.position = LinearTransformation_f(ServoHand.ValueAngle, 0, 120, 60.f, -60.f); //преобразуем в -60 - 60
          }
          
          TIM2_Flag.TIM_mSecond_Counter_IT++;//инкрементируем переменную
          if(TIM2_Flag.TIM_mSecond_Counter_IT>=CouADC)//сбрасываем при переполнении
          {
              TIM2_Flag.TIM_mSecond_Counter_IT=0;
          }
          
          PressKey_PB5(); //обрабатываем нажатие кнопки 
          ParametrDisplay(quntityArrayParametrDisplay); //вывод данных на дисплей
          
          if(quntityArrayParametrDisplay==4)
          {
              
              if(counterDelayDisplay<400){
                  counterDelayDisplay+=1;
              }
              else{
                  counterDelayDisplay=0;
                  quntityArrayParametrDisplay=1;
                  quntityArrayParametrDisplay_mem=1;
              }
              
          }
          
          
          TIM2_Flag.TIM_IT=0;//Сброс флага прерывания
        }
      if(TIM1_Flag.TIM_IT==1){
          
          HAL_StatusTypeDef statusCAN=0;
          float ServoPosition_f;
          if(invert==0){
              
              ServoPosition_f = LinearTransformation_f(ServoHand.ValueAngle, 0, 120, -1.f, 1.f);
          }
          else{
              ServoPosition_f = LinearTransformation_f(ServoHand.ValueAngle, 0, 120, 1.f, -1.f);
          }
          uint16_t ServoPosition_ = float_to_uint16_t(ServoPosition_f);
          TxData[0]=127;
          TxData[1]=invert;
          TxData[2]=ServoPosition_;
          TxData[3]=ServoPosition_>>8;
          TxData[4]=0xff;
          statusCAN=HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);//Передаем сообщение в CAN
          TIM1_Flag.TIM_IT=0;
      }
      
      if(TIM4_Flag.TIM_IT == 1)
      {
          TIM4_Flag.TIM_IT = 0;
          quntityArrayParametrDisplay=3;
          
      } 
  }

}
uint8_t TransmitOneDisplay(char* string, uint8_t *IndicatorDigit, uint8_t FlagFloat)
{
  //функция передачи по spi буквы в конкретный сегмент и инкрементирование сегмента. 
    uint8_t TransmitBuffer[1]={0}; //Буфер для передачи по SPI
  uint8_t StringBuffer[4];
  uint8_t maxCounterDigit=4;
  //Управление счетчиком тиков таймера 2 для отображения секйий на дисплее


  //Заполняем уже символами массив, для передачи по spi
    for(uint8_t nSym=0; nSym<maxCounterDigit; nSym++)
    {
        StringBuffer[nSym] = DefinitionSymbol(string[nSym]);

    }
    //Заполняем буфер для передачи по SPI
    switch(*IndicatorDigit)
    {
      case 1:
      {
        TransmitBuffer[0] = StringBuffer[4-*IndicatorDigit];
        GPIOB->BSRR |= GPIO_BSRR_BR6;
        GPIOB->BSRR |= GPIO_BSRR_BR7;
        GPIOB->BSRR |= GPIO_BSRR_BR8;
        GPIOB->BSRR |= GPIO_BSRR_BS9;

      };break;
      case 2:
      {
        TransmitBuffer[0] = StringBuffer[4-*IndicatorDigit];
        GPIOB->BSRR |= GPIO_BSRR_BR6;
        GPIOB->BSRR |= GPIO_BSRR_BR7;
        GPIOB->BSRR |= GPIO_BSRR_BS8;
        GPIOB->BSRR |= GPIO_BSRR_BR9;
                if(FlagFloat==1)
        {
            TransmitBuffer[0] |= 1<<7;//если число float, то добавляем точку перед 4 сегментом
        }
      };break;
      case 3:
      {
        TransmitBuffer[0] = StringBuffer[4-*IndicatorDigit];
        GPIOB->BSRR |= GPIO_BSRR_BR6;
        GPIOB->BSRR |= GPIO_BSRR_BS7;
        GPIOB->BSRR |= GPIO_BSRR_BR8;
        GPIOB->BSRR |= GPIO_BSRR_BR9;

      };break;
      case 4:
      {
        TransmitBuffer[0] = StringBuffer[4-*IndicatorDigit];
        GPIOB->BSRR |= GPIO_BSRR_BS6;
        GPIOB->BSRR |= GPIO_BSRR_BR7;
        GPIOB->BSRR |= GPIO_BSRR_BR8;
        GPIOB->BSRR |= GPIO_BSRR_BR9;
      };break;
      default:
      {
        
      }
    }
    
    if(*IndicatorDigit<maxCounterDigit){
        *IndicatorDigit+=1;
    }
    else{
        *IndicatorDigit=0;
    }
    SPI1_Transmit(TransmitBuffer, 1);
    HAL_Delay(1);
}
uint8_t PressKey_PB5()
{
    //Функция обработки нажатия на кнопку и гашение ее звона, вызывается по таймеру каждые 5мс
    //Возвращает число, которое инкриментируется при каждом нажатии
        uint8_t InterputPB5 = (GPIOB->IDR & GPIO_IDR_IDR5) >> 5;
    if(InterputPB5)
    {
        interputPB5_counter++;
        if(interputPB5_counter>1000)
        {
            //очень долго нажатие 5сек
            if(quntityArrayParametrDisplay!=4){
                quntityArrayParametrDisplay=4;
                quntityArrayParametrDisplay_mem = quntityArrayParametrDisplay;
                if(invert == 0){
                    invert = 1;
                }
                else 
                {
                    invert = 0;
                }
                
                Flash_Write(AddressFlash, &invert, 1);
                
                TxHeader_inv.StdId = 0;
                TxHeader_inv.ExtId = 0xAA7e;
                TxHeader_inv.RTR = CAN_RTR_DATA;
                TxHeader_inv.IDE = CAN_ID_EXT;
                TxHeader_inv.DLC = 4;
                TxData[0] = 127;
                TxData[1] = invert;
                HAL_CAN_AddTxMessage(&hcan, &TxHeader_inv, TxData, &TxMailbox);//Передаем сообщение в CAN
                
            }
        }
    }
    else
    {
        if(interputPB5_counter>=20 && interputPB5_counter<200)
        {
            if(quntityArrayParametrDisplay<quntityArrayParametrDisplayMAX){
              quntityArrayParametrDisplay+=1;
              
            }
            else{
              quntityArrayParametrDisplay=1;
            }
            quntityArrayParametrDisplay_mem = quntityArrayParametrDisplay;
        }
        else if(interputPB5_counter>=200 && interputPB5_counter<=1000 && ServoHand.ValueAngle==0){
            //долгое нажатие от 1 до 5 сек
            
        }
        interputPB5_counter=0;
        
    }
    return 0;    
    
}
uint8_t ParametrDisplay(uint8_t NumberParametrDisplay)
{
    //Функция отображения конкретных данных на дисплее, выбирается кнопкой
    switch (NumberParametrDisplay){
        case 1: {
            uint16_t ValueAngle;
            ValueAngle = (uint16_t) (LinearTransformation_f(ServoHand.ValueAngle, 0, 120, 0, 100)); //преобразуем в -60 - 60

            snprintf(LED1.StringSymbol, sizeof(LED1.StringSymbol), "%4d", (uint16_t)ValueAngle);
            TransmitOneDisplay(LED1.StringSymbol, &segment_counter, 0);
            break;
        }
        case 2: {
            int16_t ServoPosition_ = (int16_t) ServoPosition.position;
            snprintf(LED1.StringSymbol, sizeof(LED1.StringSymbol), "%4d", (int16_t)ServoPosition_);
            TransmitOneDisplay(LED1.StringSymbol, &segment_counter, 0);
            break;
        }
        case 3: {
            LED1.StringSymbol[0] = '-';
            LED1.StringSymbol[1] = '-';
            LED1.StringSymbol[2] = '-';
            LED1.StringSymbol[3] = '-';
            TransmitOneDisplay(LED1.StringSymbol, &segment_counter, 0);
            break;
        }
        case 4: {
            LED1.StringSymbol[0] = 'I';
            LED1.StringSymbol[1] = 'N';
            LED1.StringSymbol[2] = 'V';
            LED1.StringSymbol[3] = 'T';
            TransmitOneDisplay(LED1.StringSymbol, &segment_counter, 0);
            break;
        }
    }
    return 0;    
}
void SegmentSwitching(uint8_t counter)
{
    //Выбор разряда идикатора путем подачи высокого сигнала на пины (открываем транзисторы)
    switch (counter)
    {

    }
}
uint16_t moving_average(uint16_t* data, uint16_t amountData)
{
    //Функция нахождения скользящего среднего
    uint32_t summ_counter_angle=0;

    for(uint16_t counter_cycle=0; counter_cycle<amountData; counter_cycle++)
    {
      summ_counter_angle+=data[counter_cycle];
    }
    return (uint16_t)(summ_counter_angle/CouADC); //нахождение скользящего среднего          
}
void Flash_Write(uint32_t StartPagesAddress,uint16_t* Data, uint16_t quntity)
{
    //функция записи в память 
    HAL_FLASH_Unlock(); //Разблокируем доступ для работы с памятью.
    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t SectorError;
    EraseInitStruct.TypeErase       = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.PageAddress     =  StartPagesAddress;
    EraseInitStruct.NbPages         = 1;

    if (HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK)
    {
        // Обработка ошибки стирания
        HAL_FLASH_Lock();
        return;
    }
    for (int i = 0; i < quntity; i++)
    {
        if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, StartPagesAddress, Data[i])!=HAL_OK){
            break;
        }
        StartPagesAddress += 2; // Переходим к следующему адресу (4 байта для слова)
    }
    HAL_FLASH_Lock();
}

void Flash_Read(uint32_t StartPagesAddress, uint16_t* Data, uint16_t number_of_words)
{
    //функция чтения из памяти
    volatile uint16_t DataBuffer=0;
    for (uint16_t i = 0; i<number_of_words; i++)
    {
        Data[i]= *(volatile uint16_t *)StartPagesAddress;
        StartPagesAddress += 2;
    }
}
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)//прерывания от CAN
{
    if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &RxHeader, RxData) == HAL_OK)//Читаем в массив RxData
    {

    	if(RxHeader.ExtId == 0x1403F27F && RxData[0]==1)
    	{
            TIM4->CNT=0;
            quntityArrayParametrDisplay=quntityArrayParametrDisplay_mem;
        }
    }
}
uint16_t ADC_PROCESSING()
{
    
    uint16_t minRud, maxRud;
    minRud=0, maxRud=120;
    uint16_t rudValue;
    uint16_t minAdc=1000, maxAdc=4000;
    uint16_t adcValue = ADC_Read();//Чтение значеиня АЦП
    if(adcValue<minAdc){
        adcValue=minAdc;
    }
    else if(adcValue>maxAdc){
        adcValue=maxAdc;
    }
 /*   if(invert==0){
        rudValue = minRud + (maxRud-minRud) * (maxAdc-adcValue)/(maxAdc-minAdc);       
    }
    else if(invert==1){        
        rudValue = minRud + (maxRud-minRud) * (adcValue-minAdc)/(maxAdc-minAdc);
    }*/
    rudValue = minRud + (maxRud-minRud) * (maxAdc-adcValue)/(maxAdc-minAdc);
    if(rudValue>maxRud){rudValue=maxRud;}
    return rudValue;
}
/*------------------------------------------------------------------*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}
static void MX_CAN_Init(void)
{
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 2;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_15TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = ENABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = ENABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = ENABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  
  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = (uint16_t)(0x00>>13);
  sFilterConfig.FilterIdLow = (uint16_t)(0x00<<3) | 0x04;
  sFilterConfig.FilterMaskIdHigh = (uint16_t)(0x00>>13);
  sFilterConfig.FilterMaskIdLow = (uint16_t)(0x00<<3) | 0x04;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO1;
  sFilterConfig.FilterActivation = ENABLE;

  if(HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  NVIC_SetPriority(USART2_IRQn,1);
}
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

}
void GPIO_Init()
{
  __HAL_RCC_GPIOB_CLK_ENABLE();

  //PA0 - analog input
  GPIOA->CRL &= ~(GPIO_CRL_MODE0_Msk | GPIO_CRL_CNF0_Msk);
  //PA5 - SPI SCLK Alternate function push-pull
  GPIOA->CRL &= ~(GPIO_CRL_MODE5_Msk | GPIO_CRL_CNF5_Msk);
  GPIOA->CRL |= (0b11<<GPIO_CRL_MODE5_Pos) | (0b10<<GPIO_CRL_CNF5_Pos);
  //PA6 - output push-pull
  GPIOA->CRL &= ~(GPIO_CRL_MODE6_Msk | GPIO_CRL_CNF6_Msk);
  GPIOA->CRL |= (0b11<<GPIO_CRL_MODE6_Pos);
  GPIOA->CRL |= (0b00<<GPIO_CRL_CNF6_Pos);
  //PA7 - SPI MOSI Alternate function push-pull
  GPIOA->CRL &= ~(GPIO_CRL_MODE7_Msk | GPIO_CRL_CNF7_Msk);
  GPIOA->CRL |= (0b11<<GPIO_CRL_MODE7_Pos) | (0b10<<GPIO_CRL_CNF7_Pos);
  //PB0 - output push-pull
  GPIOB->CRL &= ~(GPIO_CRL_MODE0_Msk | GPIO_CRL_CNF0_Msk);
  GPIOB->CRL |= (0b11<<GPIO_CRL_MODE0_Pos);
  GPIOB->CRL |= (0b00<<GPIO_CRL_CNF0_Pos);
  //PB1 - output CH
  GPIOB->CRL &= ~(GPIO_CRL_MODE1_Msk | GPIO_CRL_CNF1_Msk);
  GPIOB->CRL |= (0b10<<GPIO_CRL_CNF1_Pos | 0b11<<GPIO_CRL_MODE1_Pos);
  //PB12-PB15 - output push-pull
  GPIOB->CRH &= ~(GPIO_CRH_MODE12_Msk | GPIO_CRH_CNF12_Msk | GPIO_CRH_MODE13_Msk | GPIO_CRH_CNF13_Msk | GPIO_CRH_MODE14_Msk |
      GPIO_CRH_CNF14_Msk | GPIO_CRH_MODE15_Msk | GPIO_CRH_CNF15_Msk);//Clear
  GPIOB->CRH |= (0b11<<GPIO_CRH_MODE12_Pos | 0b00<<GPIO_CRH_CNF12_Pos | 0b11<<GPIO_CRH_MODE13_Pos | 0b00<<GPIO_CRH_CNF13_Pos |
      0b11<<GPIO_CRH_MODE14_Pos | 0b00<<GPIO_CRH_CNF14_Pos | 0b11<<GPIO_CRH_MODE15_Pos | 0b00<<GPIO_CRH_CNF15_Pos);
  GPIOB->CRH |= (0b00<<GPIO_CRL_CNF0_Pos); //PB0 Output push-pull
  //PB4-PB7 - reserved
  //PB6 & PB7 - output push-pull
  GPIOB->CRL &= ~(GPIO_CRL_MODE6_Msk | GPIO_CRL_CNF6_Msk);
  GPIOB->CRL |= (0b11<<GPIO_CRL_MODE6_Pos);
  GPIOB->CRL |= (0b00<<GPIO_CRL_CNF6_Pos);
  GPIOB->BSRR |= GPIO_BSRR_BR6;

  GPIOB->CRL &= ~(GPIO_CRL_MODE7_Msk | GPIO_CRL_CNF7_Msk);
  GPIOB->CRL |= (0b11<<GPIO_CRL_MODE7_Pos);
  GPIOB->CRL |= (0b00<<GPIO_CRL_CNF7_Pos);
  GPIOB->BSRR |= GPIO_BSRR_BR7;
  
  //PB8 & PB9 - output push-pull
  GPIOB->CRH &= ~(GPIO_CRH_MODE8_Msk | GPIO_CRH_CNF8_Msk);
  GPIOB->CRH |= (0b11<<GPIO_CRH_MODE8_Pos);
  GPIOB->CRH |= (0b00<<GPIO_CRH_CNF8_Pos);
  GPIOB->BSRR |= GPIO_BSRR_BR8;
  
  GPIOB->CRH &= ~(GPIO_CRH_MODE9_Msk | GPIO_CRH_CNF9_Msk);
  GPIOB->CRH |= (0b11<<GPIO_CRH_MODE9_Pos);
  GPIOB->CRH |= (0b00<<GPIO_CRH_CNF9_Pos);
  GPIOB->BSRR |= GPIO_BSRR_BR9;
  //PB5 - input 
  GPIOB->CRL &= ~(GPIO_CRL_MODE5_Msk | GPIO_CRL_CNF5_Msk);
  GPIOB->CRL |= (0b10<<GPIO_CRL_CNF5_Pos | 0b00<<GPIO_CRL_MODE5_Pos);
  /*AFIO->EXTICR[1] |= AFIO_EXTICR2_EXTI6_PB;
  EXTI->IMR |= EXTI_IMR_MR6;
  EXTI->RTSR = 0b001000000;// Срабатывание по переднему фронту
  //EXTI->FTSR = 0b001000000;// Срабатывание по заднему фронту
  NVIC_EnableIRQ(EXTI9_5_IRQn);*/
}
/*------------------------------------------------------------------*/
/*------------------------------------------------------------------*/
uint8_t DefinitionSymbol(char Symbol)
{
  /*
   * Функция декодирует символ в набор битов для дальнейшей демонстрации на 7-сегментном дисплее
   * результатом выполнения явлеется 8-битное число
   *
   * */
  uint8_t Symb;
  switch(Symbol)
  {
    case'0': Symb = 0b00111111; break;
    case'1': Symb = 0b00000110; break;
    case'2': Symb = 0b01011011; break;
    case'3': Symb = 0b01001111; break;
    case'4': Symb = 0b01100110; break;
    case'5': Symb = 0b01101101; break;
    case'6': Symb = 0b01111101; break;
    case'7': Symb = 0b00000111; break;
    case'8': Symb = 0b01111111; break;
    case'9': Symb = 0b01101111; break;
    case' ': Symb = 0b00000000; break;

    case'A': Symb = 0b01110111; break;
    case'b': Symb = 0b01111100; break;
    case'C': Symb = 0b00111001; break;
    case'D': Symb = 0b01011110; break;
    case'E': Symb = 0b01111001; break;
    case'F': Symb = 0b01110001; break;
    case'G': Symb = 0b00111101; break;
    case'H': Symb = 0b01110110; break;
    case'I': Symb = 0b00110000; break;
    case'J': Symb = 0b00011110; break;
    case'L': Symb = 0b00111000; break;
    case'M': Symb = 0b01001001; break;
    case'O': Symb = 0b00111111; break;
    case'P': Symb = 0b01110011; break;
    case'S': Symb = 0b01101101; break;
    case'T': Symb = 0b01111000; break;
    case't': Symb = 0b11111000; break;
    case'r': Symb = 0b01010000; break;
    case'N': Symb = 0b01010100; break;
    case'X': Symb = 0b01110110; break;
    case'U': Symb = 0b00111110; break;
    case'V': Symb = 0b00011100; break;
    case'Y': Symb = 0b01101110; break;
    case'-': Symb = 0b01000000; break;
    // Если символ окончания строки, выводится число 125
    case'\0': Symb = 0b11111111; break;
    // В случае неизвестного символа генерируется непонятный символ
    default: Symb = 0b01000011;break;
  }//end switch
  return Symb;
}
/*------------------------------------------------------------------*/





void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

void assert_failed(uint8_t* file, uint32_t line)
{
}

#endif
