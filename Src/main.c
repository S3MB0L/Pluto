#include "stm32f0xx_hal.h"
#include "cmsis_os.h"
#include "stdio.h"
#include "string.h"

#ifdef __GNUC__
	#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
	#define GETCHAR_PROTOTYPE int __io_getchar(int chr)
#else 
	#define PUTCHAR_PROTOTYPE int fputc(int ch,FILE *f)
	#define GETCHAR_PROTOTYPE int fgetc(FILE *f)
#endif
	
#define TXBUFFERSIZE                      (COUNTOF(aTxBuffer_1) - 1)
#define RXBUFFERSIZE                      TXBUFFERSIZE
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))
#define UartHandle1 huart1
#define UartHandle2 huart2
#define EEPROM_ADDRESS_READ            0xA1
#define EEPROM_ADDRESS_WRITE					 0xA0
#define EEPROM_MAXPKT             8              //(page size)
#define EEPROM_WRITE              5               //time to wait in ms
#define EEPROM_TIMEOUT             2*EEPROM_WRITE  //timeout while writing
#define EEPROM_MEMBASEADDRESS   (uint16_t)0x100 //must be multiple of (EEPROM_MAXPKT)
#define BUF_SIZE 8

	
__IO uint32_t UserButtonStatus=0;
__IO uint32_t isDelayFinished_G=0;
__IO ITStatus Uart1_TXReady = RESET;
__IO ITStatus Uart1_RXReady = RESET;
__IO ITStatus Uart2_TXReady = RESET;
__IO ITStatus Uart2_RXReady = RESET;
uint8_t dizi1[BUF_SIZE]="12345678";
uint8_t dizi2[BUF_SIZE];
uint8_t aTxBuffer_1[] = " **UART_Board_1**               ";
uint8_t aTxBuffer_2[] = " **UART_Board_2** 							 ";
uint8_t aRxBuffer_1[RXBUFFERSIZE];
uint8_t aRxBuffer_2[RXBUFFERSIZE];
uint32_t eeprom_read=0;
uint32_t uwPrescalervalue=0;
char __chr=0;
	
CRC_HandleTypeDef hcrc;
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart2_rx;

osThreadId Uart_TaskHandle;
osThreadId EasyDriveTaskHandle;

void SystemClock_Config(void);
static void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CRC_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static uint16_t Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength);
void StartUartTask(void const * argument);
void StartEasyDriveTask(void const * argument);
void delay_70usec(void);
HAL_StatusTypeDef SysHW_readEEPROM(uint8_t* MemTarget, uint16_t Size);
HAL_StatusTypeDef SysHW_writeEEPROM(uint8_t* MemSource, uint16_t Size);

GETCHAR_PROTOTYPE
{
	HAL_UART_Receive(&huart1,(uint8_t *)&__chr,1,1000);
	return __chr;
}

PUTCHAR_PROTOTYPE
 {
	 HAL_UART_Transmit(&huart1,(uint8_t *)&ch,1,1000);
	 return ch;
 }

int main(void)
{

  HAL_Init();

  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CRC_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();

	uwPrescalervalue = (uint32_t)(SystemCoreClock/10000)-1;

  osThreadDef(Uart_Task, StartUartTask, osPriorityNormal, 0, 128);
  Uart_TaskHandle = osThreadCreate(osThread(Uart_Task), NULL);

  osThreadDef(EasyDriveTask, StartEasyDriveTask, osPriorityIdle, 0, 128);
  EasyDriveTaskHandle = osThreadCreate(osThread(EasyDriveTask), NULL);

  osKernelStart();
	
	// Port B, pin 1-2 control MS1 and MS2
  //                       MS1     MS2
  // quarter step	          L       H
	// half step              H       L
	// full step (2 phase)    L       L
	// Eigth Step             H       H
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);  // MS1
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);  // MS2
	  if(HAL_UART_Receive_DMA(&UartHandle1, (uint8_t *)aRxBuffer_1, RXBUFFERSIZE) != HAL_OK)
  {
    Error_Handler();
  }
	  if(HAL_UART_Receive_DMA(&UartHandle2, (uint8_t *)aRxBuffer_2, RXBUFFERSIZE) != HAL_OK)
  {
    Error_Handler();
  }
  while (1)
  {


  }

}

void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  HAL_NVIC_SetPriority(SysTick_IRQn, 3, 0);
}

void MX_CRC_Init(void)
{

  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLED;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  HAL_CRC_Init(&hcrc);

}

void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x0000020B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLED;
  HAL_I2C_Init(&hi2c1);

  HAL_I2CEx_AnalogFilter_Config(&hi2c1, I2C_ANALOGFILTER_ENABLED);

}

void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 24;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 620-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim3);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);

}

void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONEBIT_SAMPLING_DISABLED ;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  HAL_UART_Init(&huart1);

}

void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONEBIT_SAMPLING_DISABLED ;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  HAL_UART_Init(&huart2);

}

void MX_DMA_Init(void) 
{
  __DMA1_CLK_ENABLE();

  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
  HAL_NVIC_SetPriority(DMA1_Channel4_5_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_5_IRQn);

}

void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  __GPIOA_CLK_ENABLE();
  __GPIOC_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();

  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = GPIO_PIN_10 |GPIO_PIN_11 | GPIO_PIN_1 | GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	/* Enable and set button EXTI interrupt to the lowest priority */
	HAL_NVIC_SetPriority((IRQn_Type) EXTI0_1_IRQn, 0x03, 0x00);
	HAL_NVIC_EnableIRQ((IRQn_Type) EXTI0_1_IRQn);
}

void StartUartTask(void const * argument)
{

  for(;;)
  {
		      if(Uart1_RXReady == SET && Uart2_TXReady == SET)
      {
          Uart1_RXReady = RESET;
          Uart2_TXReady = RESET;
          memcpy(aTxBuffer_2, aRxBuffer_1, 20);
          if(HAL_UART_Transmit_DMA(&UartHandle2, (uint8_t *)aTxBuffer_2, TXBUFFERSIZE) != HAL_OK)
          {
              Error_Handler();
          }
      }

      HAL_Delay(5);

	  if(Uart2_RXReady == SET && Uart1_TXReady == SET )
	  {
			Uart2_RXReady = RESET;
			Uart1_TXReady = RESET;
			memcpy(aTxBuffer_1, aRxBuffer_2, 20);
			if(HAL_UART_Transmit_DMA(&UartHandle1, (uint8_t*)aTxBuffer_1, TXBUFFERSIZE)!= HAL_OK)
          {
						Error_Handler();
					}
	  }
		

      HAL_Delay(5);
		if(Uart2_RXReady==SET && Uart2_TXReady==SET){
			Uart2_RXReady=RESET;
			Uart2_TXReady=RESET;
			memcpy(aRxBuffer_2,aTxBuffer_1,20);
			if(HAL_UART_Transmit_DMA(&UartHandle1,(uint8_t*) aTxBuffer_1,TXBUFFERSIZE)!=HAL_OK){
				Error_Handler();
			}
		}
		
			HAL_Delay(10);
		
		}
	
		printf("Giris Bekleniyor\n\r");
		osDelay(1000);
		if(eeprom_read==1){

		printf("\n\r\n\r");
		osDelay(3000);
		SysHW_writeEEPROM((uint8_t *)dizi1,BUF_SIZE);
		SysHW_readEEPROM((uint8_t *)dizi2,BUF_SIZE);
		printf("Okunan veri=\n\r");
		for(int i=0;i<BUF_SIZE;i++)
			printf("%c",dizi2[i]);
		eeprom_read=0;
    }
    osDelay(1);
  }

void StartEasyDriveTask(void const * argument)
{

  for(;;)
  {
		if(UserButtonStatus == 1)
		{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET); // Enable the motors, active low
			HAL_Delay(500);
			for(int i=0; i < 17500; i++){				
				
				//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
				delay_70usec();
				//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
				delay_70usec();
				//for(int j=0; j < 1; j++){
				//	delay_70usec();
				//}
			}
			UserButtonStatus = 0;
			HAL_Delay(500);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET); // Disable the motor, set it to high
		}
		
		HAL_Delay(500);
		
		
  }
    osDelay(1);
}
void delay_70usec(void)
{
	
	htim3.Instance->CNT = 0;
	isDelayFinished_G = 0;
	while(isDelayFinished_G == 0){
		__NOP;
	}
	//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);
}
HAL_StatusTypeDef SysHW_readEEPROM(uint8_t* MemTarget, uint16_t Size)
{
		printf("Okuma Basliyor\n\r");
    uint16_t Counter = 0;
    HAL_StatusTypeDef Result = HAL_OK;
		printf("Okuma Dongusu Basliyor\r\n");
    while (Counter < Size && Result == HAL_OK)
    {
        uint16_t Diff = Size - Counter;

        if (Diff >= EEPROM_MAXPKT)
        {
            //Multi-Byte
            //Result = HAL_I2C_Mem_Read(&hi2c1, EEPROM_ADDRESS_READ, EEPROM_MEMBASEADDRESS + Counter, I2C_MEMADD_SIZE_8BIT, &MemTarget[Counter], EEPROM_MAXPKT, EEPROM_TIMEOUT);
            Result=HAL_I2C_Mem_Read_DMA(&hi2c1,EEPROM_ADDRESS_WRITE,EEPROM_MEMBASEADDRESS+Counter,I2C_MEMADD_SIZE_8BIT,&MemTarget[Counter],Size);
						Counter += EEPROM_MAXPKT;
        }
        else
        {
            //and the remaining ones...low packet size
            //Result = HAL_I2C_Mem_Read(&hi2c1, EEPROM_ADDRESS_READ, EEPROM_MEMBASEADDRESS + Counter, I2C_MEMADD_SIZE_8BIT, &MemTarget[Counter], Diff, EEPROM_TIMEOUT);
            Result=HAL_I2C_Mem_Read_DMA(&hi2c1,EEPROM_ADDRESS_WRITE,EEPROM_MEMBASEADDRESS+Counter,I2C_MEMADD_SIZE_8BIT,&MemTarget[Counter],Size);            
						Counter += Diff;
        }
        //HAL_Delay(EEPROM_WRITE);
    }
		printf("Okuma Dongusu Bitti\r\n");
		if(Result==0) printf("Okuma basarili\n\r"); else if(Result==1) printf("Okuma basarisiz\n\r"); 
		else if(Result==2) printf("Aygit Mesgul\r\n"); else if(Result==3) printf("Zaman asimi\n\r");

    return Result;
}

HAL_StatusTypeDef SysHW_writeEEPROM(uint8_t* MemSource, uint16_t Size)
{
		printf("Yazim basliyor\n\r");
    uint16_t Counter = 0;
    HAL_StatusTypeDef Result = HAL_OK;
		printf("Yazim Dongusu basliyor\r\n");
    while (Counter < Size && Result == HAL_OK)
    {
        uint16_t Diff = Size - Counter;

        if (Diff >= EEPROM_MAXPKT)
        {
            //Multi-Byte
            //Result = HAL_I2C_Mem_Write(&hi2c1, EEPROM_ADDRESS_WRITE, EEPROM_MEMBASEADDRESS + Counter, I2C_MEMADD_SIZE_8BIT, &MemSource[Counter], EEPROM_MAXPKT, EEPROM_TIMEOUT);
            Result=HAL_I2C_Mem_Write_DMA(&hi2c1,EEPROM_ADDRESS_WRITE,EEPROM_MEMBASEADDRESS+Counter,I2C_MEMADD_SIZE_8BIT,&MemSource[Counter],Size);
						Counter += EEPROM_MAXPKT;
        }
        else
        {
            //and the remaining ones...low packet size
            //Result = HAL_I2C_Mem_Write(&hi2c1, EEPROM_ADDRESS_WRITE, EEPROM_MEMBASEADDRESS + Counter, I2C_MEMADD_SIZE_8BIT, &MemSource[Counter], Diff, EEPROM_TIMEOUT);
            Result=HAL_I2C_Mem_Write_DMA(&hi2c1,EEPROM_ADDRESS_WRITE,EEPROM_MEMBASEADDRESS+Counter,I2C_MEMADD_SIZE_8BIT,&MemSource[Counter],Size);
						Counter += Diff;
        }
        //HAL_Delay(EEPROM_WRITE);
    }
		
		printf("Yazim dongusu bitti\r\n");
		
		if(Result==0) printf("Yazim basarili\n\r"); 
		else if(Result==1) printf("Yazim basarisiz\n\r"); 
		else if(Result==2) printf("Aygit Mesgul\r\n"); 
		else if(Result==3) printf("Zaman asimi\n\r");
		
    return Result;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9);  // GPIO port C, pin 9 => LED3
	//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_10);
	isDelayFinished_G = 1;
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == GPIO_PIN_0)
  {  
    UserButtonStatus = 1;
  }
}
	void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
	{

		
        if(UartHandle->Instance == USART1)
        {
            /* Set transmission flag: transfer complete */
		    Uart1_TXReady = SET;
		    /* Turn LED3 on: Transfer in transmission process is correct */

				printf("Uart_Tx_1 tamamlandi\n\r");
        }

        if(UartHandle->Instance == USART2)
        {
          /* Set transmission flag: transfer complete */
          Uart2_TXReady = SET;
				printf("Uart_Tx_2 tamamlandi\n\r");
        }
		
	}
		void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
	{

						
        if(UartHandle->Instance == USART1)
        {
	    	/* Set transmission flag: trasfer complete*/
		    Uart1_RXReady = SET;
		    if(HAL_UART_Receive_DMA(UartHandle, (uint8_t *)aRxBuffer_1, RXBUFFERSIZE) != HAL_OK)
            {
               Error_Handler();
            }

		    /* Turn LED3 on: Transfer in reception process is correct */
		    printf("Uart_1_RX tamamlandi\n\r");
        }

        if(UartHandle->Instance == USART2)
        {
            Uart2_RXReady = SET;
            if(HAL_UART_Receive_DMA(UartHandle, (uint8_t *)aRxBuffer_2, RXBUFFERSIZE) != HAL_OK)
            {
                Error_Handler();
            }
				printf("Uart_2_RX tamamlandi\n\r");
						
        }			

	}
static uint16_t Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength)
{
  while (BufferLength--)
  {
    if ((*pBuffer1) != *pBuffer2)
    {
      return BufferLength;
    }
    pBuffer1++;
    pBuffer2++;
  }

  return 0;
}
static void Error_Handler(void)
{
  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_SET);
  while(1)
  {
    HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_9); 
    HAL_Delay(1000); 
  }  
}

#ifdef USE_FULL_ASSERT

void assert_failed(uint8_t* file, uint32_t line)
{

}

#endif
