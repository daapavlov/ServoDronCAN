/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#include <TIMx.h>
#include <SPIx.h>
#include <ADCx.h>

#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <stdint.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;
CAN_FilterTypeDef sFilterConfig;


UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define AddressFlash    0x08007C00
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN_Init(void);


/* USER CODE BEGIN PFP */
void GPIO_Init(void);
static void Flash_Read(uint32_t StartPagesAddress, uint16_t* Data, uint16_t number_of_words);
static void Flash_Write(uint32_t StartPagesAddress,uint16_t* Data, uint16_t quntity);
void output_DAC(float x);
float uint16_to_float(uint16_t x);
void output_CH(float x);
void output_SIGNAL(float x);
uint16_t float_to_uint16_t(float x);
float LinearTransformation_f(uint16_t value, uint16_t in_min, uint16_t in_max,
                   float out_min, float out_max);
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
#define IDE_EXT_actuator_Status 0x1803F300 //uavcan.equipment.actuator.Status
uint8_t CAN_RX_Data[8];
typedef struct 
{
    uint8_t ID;
    uint32_t ExtId;
    float position; //позиция в градусах от -60 до 60
    float force; //ток в амперах
    float speed; // скорость перемещения
    uint16_t CanRx_Data_12[16];
}Actuator;

uint8_t CanTx_Data_5[5];
uint32_t TxMailbox = 0;
/*------------------------------------------------------------------*/


CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
Actuator ServoPrivod;
Actuator BPU;
TIM_Flag TIM2_Flag;
TIM_Flag TIM1_Flag;

float minPosition = 0.f;
float minPositionBuffer = 0.f;
uint16_t position_first=1;
/* USER CODE END 0 */

int main(void)
{

  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  
  Flash_Read(AddressFlash,  &position_first, 1);
  if(position_first==0xFFFF)
  {
      minPosition = LinearTransformation_f(60, 0, 120, -1, 1);
  }
  else{
      minPosition = LinearTransformation_f(position_first, 0, 120, -1, 1);
  }
  
  GPIO_Init();
  MX_CAN_Init();
  SPI1_Init();
  TIM4_Init();
  TIM_Start(TIM4);
  TIM4->CCR2 = 5000;
  TIM3_Init();
  TIM_Start(TIM3);
  TIM2_Init();
  TxHeader.StdId = 0;
  TxHeader.ExtId = 0x1403F27F;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.IDE = CAN_ID_EXT;
  TxHeader.DLC = 5;

  ServoPrivod.ID = 3;
  BPU.ID = 126;
  HAL_CAN_Start(&hcan);
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO1_MSG_PENDING | CAN_IT_ERROR | CAN_IT_BUSOFF | CAN_IT_LAST_ERROR_CODE);
  TIM_Start(TIM2);
  while (1)
  {
         if(TIM2_Flag.TIM_IT==1)
         {
             
             //В случае обрыва сообщения (1 сек ожидания и отправляем машинку в 0);
             TIM2_Flag.TIM_IT=0;
             uint16_t TIM_CCR = (uint16_t)(700*minPosition+1500);               

             uint16_t pos_u16 =  float_to_uint16_t(minPosition);
             CanTx_Data_5[0]=1;
             CanTx_Data_5[1]=0x00;
             CanTx_Data_5[2]=pos_u16;
             CanTx_Data_5[3]=pos_u16>>8;
             CanTx_Data_5[4]=0xFF;
             HAL_CAN_AddTxMessage(&hcan, &TxHeader, CanTx_Data_5, &TxMailbox);//Передаем сообщение в CAN
             TIM3->CCR4 = TIM_CCR;
         }
  }

}
float LinearTransformation_f(uint16_t value, uint16_t in_min, uint16_t in_max,
                   float out_min, float out_max)
{
    // Защита от деления на ноль
    if (in_min == in_max) {
        return out_min;
    }

    float in_range = (float)in_max - in_min;
    float out_range = (float)out_max - out_min;

    // Формула: out = out_min + (value - in_min) * out_range / in_range
    float result = (float)out_min +
                     (((float)value - in_min) * out_range) / in_range;

    return result;
}
void output_SIGNAL(float x)
{
    float alpha = 0;
    if(x<-60){
        alpha=-60.f;
    }
    else if(x>60.f){
        alpha=60.f;
    }
    else{
        alpha=x;
    }
    output_CH(alpha);//частотный выход -60_60 100_1000 Гц
    output_DAC((alpha+60)/24.f);//аналоговый выход -60_60 0_5В
}
void output_CH(float x)
{
    //input -60 -- 60 градусов
    uint16_t Period = (uint16_t) (-7.5*x+550);
    TIM4->ARR = Period;//преобразование в частоту
    TIM4->CCR2 =(uint16_t)Period/2;
}
void output_DAC(float x)
{
    float k=1.083f;
    uint16_t dac_value=0;
    uint8_t dataBuffer[2];
    dac_value = (uint16_t)((4096*x)/(5*k));
    dac_value |= 1<<13;//G=0
    dac_value |= 1<<14;//buff on
    dac_value |= 1<<12;//active on
    
    dataBuffer[1] = dac_value;
    dataBuffer[0] = dac_value>>8;
     
    SPI1_Transmit(dataBuffer, 2);
}
float uint16_to_float(uint16_t x)
{
    uint32_t sign = (x & 0x8000) << 16;
    uint32_t exp  = (x & 0x7C00) >> 10;
    uint32_t mant = (x & 0x03FF);
    uint32_t f_bits;

    if (exp == 0) {
        if (mant != 0) {
            // Денормализованное число: приводим к нормализованному виду для float32
            exp = 127 - 15;
            while ((mant & 0x0400) == 0) {
                mant <<= 1;
                exp--;
            }
            f_bits = sign | ((exp + 1) << 23) | ((mant & 0x03FF) << 13);
        } else {
            f_bits = sign; // Точный ноль (+0.0 или -0.0)
        }
    } else if (exp == 31) {
        // Infinity или NaN
        f_bits = sign | 0x7F800000 | (mant << 13);
    } else {
        // Нормализованное число
        f_bits = sign | ((exp - 15 + 127) << 23) | (mant << 13);
    }

    float result;
    memcpy(&result, &f_bits, sizeof(result));
    return result;
}
uint16_t float_to_uint16_t(float x)
{
    uint32_t f_bits;
    memcpy(&f_bits, &x, sizeof(f_bits));

    uint32_t sign = (f_bits >> 16) & 0x8000;      // бит 31 → бит 15
    uint32_t exp  = (f_bits >> 23) & 0xFF;        // 8 бит экспоненты float32
    uint32_t mant = f_bits & 0x007FFFFF;           // 23 бита мантиссы float32

    // Особые случаи: NaN, Inf
    if (exp == 0xFF) {
        return (uint16_t)(sign | 0x7C00 | ((mant >> 13) & 0x3FF));
    }

    // Нули и денормализованные float32 (exp=0)
    if (exp == 0) {
        if (mant == 0) {
            return (uint16_t)sign; // +0 или -0
        }
        // Нормализуем вручную, чтобы корректно округлить
        uint32_t shift = 0;
        uint32_t m = mant;
        while ((m & 0x00800000) == 0 && m != 0) {
            m <<= 1;
            shift++;
        }
        int32_t effective_exp = 1 - 127 - (int32_t)shift;
        exp = (uint32_t)effective_exp;
        mant = m;
    }

    int32_t new_exp = (int32_t)exp - 127 + 15; // конвертация bias: 127 → 15

    // Округление мантиссы до 10 бит (round to nearest, ties to even)
    uint32_t round_bit = (mant >> 12) & 1;
    uint32_t sticky_bit = (mant & 0x00000FFF) != 0;
    mant = (mant >> 13) & 0x03FF;

    if (round_bit) {
        if (sticky_bit || (mant & 1)) {
            mant++;
            if (mant == 0x400) {
                mant = 0;
                new_exp++;
            }
        } else if (mant & 1) { // ровно половина, младший бит 1 → округляем вверх
            mant++;
            if (mant == 0x400) {
                mant = 0;
                new_exp++;
            }
        }
    }

    // Обработка переполнения/поднормализованных
    if (new_exp >= 31) {
        return (uint16_t)(sign | 0x7C00); // Infinity
    }
    if (new_exp <= 0) {
        if (new_exp <= -15) {
            return (uint16_t)sign; // слишком мало → ноль
        }
        int32_t shift = 1 - new_exp;
        mant >>= shift;
        new_exp = 0;
    }

    return (uint16_t)(sign | ((uint32_t)new_exp << 10) | mant);
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
    if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &RxHeader, CAN_RX_Data) == HAL_OK)//Читаем в массив RxData
    {
        ServoPrivod.ExtId = IDE_EXT_actuator_Status + ServoPrivod.ID;
        BPU.ExtId = 0x1803F200  + BPU.ID;
    	if(RxHeader.ExtId == ServoPrivod.ExtId)
    	{
            if(RxHeader.DLC == 8){
                for(uint8_t i=0; i<8; i++){
                  ServoPrivod.CanRx_Data_12[i]=CAN_RX_Data[i];
                }
                uint16_t dd=ServoPrivod.CanRx_Data_12[4]<<8 | ServoPrivod.CanRx_Data_12[3];
                ServoPrivod.position = uint16_to_float(dd);
                output_SIGNAL(ServoPrivod.position);
            }
            else if(RxHeader.DLC == 4){
                for(uint8_t i=0; i<4; i++){
                  ServoPrivod.CanRx_Data_12[i+8]=CAN_RX_Data[i];
                }
            }
        }
        else if(RxHeader.ExtId==BPU.ExtId){
            
            if(CAN_RX_Data[0]==127){
                TIM2->CNT = 0;
                uint16_t Pos=CAN_RX_Data[1];
                if(Pos!=position_first){
                    position_first = Pos;
                    Flash_Write(AddressFlash, &Pos, 1);
                }
                minPosition = LinearTransformation_f(Pos, 0, 120, -1, 1);

                uint16_t dd=CAN_RX_Data[3]<<8 | CAN_RX_Data[2];
                BPU.position = uint16_to_float(dd);
                CanTx_Data_5[0]=1;
                CanTx_Data_5[1]=0x00;
                CanTx_Data_5[2]=CAN_RX_Data[2];
                CanTx_Data_5[3]=CAN_RX_Data[3];
                CanTx_Data_5[4]=0xFF;
                HAL_CAN_AddTxMessage(hcan, &TxHeader, CanTx_Data_5, &TxMailbox);//Передаем сообщение в CAN
                uint16_t TIM_CCR = (uint16_t)(700*BPU.position+1500);
                if(TIM_CCR>=800 && TIM_CCR<=2200){
                    
                    TIM3->CCR4 = TIM_CCR;
                }
                else{
                    TIM3->CCR4 = 800;
                }
                
            }

        }
        else if(RxHeader.ExtId==0xAA7e)
        {
            if(CAN_RX_Data[0]==127)
            {
                uint16_t Pos=CAN_RX_Data[1];
                
                
                //Flash_Write(AddressFlash, &Pos, 1);
            }
        }
    }
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
  //PA4 - output push-pull
  GPIOA->CRL &= ~(GPIO_CRL_MODE4_Msk | GPIO_CRL_CNF4_Msk);
  GPIOA->CRL |= (0b11<<GPIO_CRL_MODE4_Pos);
  GPIOA->CRL |= (0b00<<GPIO_CRL_CNF4_Pos);
  GPIOA->BSRR |= GPIO_BSRR_BS4;
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
  //PB2 - output push-pull
  GPIOB->CRL &= ~(GPIO_CRL_MODE2_Msk | GPIO_CRL_CNF2_Msk);
  GPIOB->CRL |= (0b11<<GPIO_CRL_MODE2_Pos);
  GPIOB->CRL |= (0b00<<GPIO_CRL_CNF2_Pos);
  GPIOB->BSRR |= GPIO_BSRR_BS2;
  //PB12-PB15 - output push-pull
  GPIOB->CRH &= ~(GPIO_CRH_MODE12_Msk | GPIO_CRH_CNF12_Msk | GPIO_CRH_MODE13_Msk | GPIO_CRH_CNF13_Msk | GPIO_CRH_MODE14_Msk |
      GPIO_CRH_CNF14_Msk | GPIO_CRH_MODE15_Msk | GPIO_CRH_CNF15_Msk);//Clear
  GPIOB->CRH |= (0b11<<GPIO_CRH_MODE12_Pos | 0b00<<GPIO_CRH_CNF12_Pos | 0b11<<GPIO_CRH_MODE13_Pos | 0b00<<GPIO_CRH_CNF13_Pos |
      0b11<<GPIO_CRH_MODE14_Pos | 0b00<<GPIO_CRH_CNF14_Pos | 0b11<<GPIO_CRH_MODE15_Pos | 0b00<<GPIO_CRH_CNF15_Pos);
  GPIOB->CRH |= (0b00<<GPIO_CRL_CNF0_Pos); //PB0 Output push-pull
  //PB4-PB7 - reserved
  //PB7 - output CH
  GPIOB->CRL &= ~(GPIO_CRL_MODE7_Msk | GPIO_CRL_CNF7_Msk);
  GPIOB->CRL |= (0b10<<GPIO_CRL_CNF7_Pos | 0b11<<GPIO_CRL_MODE7_Pos);

  //PB4 - Input with pull-down
  GPIOB->CRL &= ~(GPIO_CRL_MODE4_Msk | GPIO_CRL_CNF4_Msk);
  GPIOB->CRL |= (0b00<<GPIO_CRL_MODE4_Pos);
  GPIOB->CRL |= (0b10<<GPIO_CRL_CNF4_Pos);
  GPIOB->ODR &= ~GPIO_ODR_ODR4;
  //PB6 - input EXTI
  GPIOB->CRL &= ~(GPIO_CRL_MODE6_Msk | GPIO_CRL_CNF6_Msk);
  GPIOB->CRL |= (0b10<<GPIO_CRL_CNF6_Pos | 0b00<<GPIO_CRL_MODE6_Pos);
  /*AFIO->EXTICR[1] |= AFIO_EXTICR2_EXTI6_PB;
  EXTI->IMR |= EXTI_IMR_MR6;
  EXTI->RTSR = 0b001000000;// Срабатывание по переднему фронту
  //EXTI->FTSR = 0b001000000;// Срабатывание по заднему фронту
  NVIC_EnableIRQ(EXTI9_5_IRQn);*/
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
