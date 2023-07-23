/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  * @Auther:		Abdalla ragab created Base file From MXCube and implemented the algorthim
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FLASH_ADDRESS 0x08005000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define FLASH_NUM_PAGES			64
#define MY_RECORD_SIZE			22
#define MY_BL_SIZE_PAGES		20
/* -----------CAN MSG IDS--------------------*/
//incoming
#define START_SIGNAL_STDID_CAN	0x19
#define START_SIGNAL_DATA_CAN	0x0
//ST1 -BMS BMS Sensors
#define FRUP_START_STDID_CAN	0x16
#define FRUP_RECORD_STDID_CAN	0x23
#define FRUP_NLINE_STDID_CAN	0x26


#define APP_START_STDID_CAN		0x22
//outgoing
#define FRUP_READY					0x27
#define FRUP_NLINE_RDY_STDID_CAN	0x29


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */
CAN_TxHeaderTypeDef MTxHeader;
CAN_RxHeaderTypeDef RxHeader;

uint8_t TxData[8]={1};
uint8_t RxData[8];
uint32_t Mailbox=CAN_TX_MAILBOX0;
uint8_t resend=0;
typedef void (*Function_t)(void);
//Function_t addr_to_call = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
void gotoAPP(void);
void EraseFlash(uint32_t pageAddress,uint32_t pagesNum);
void programFlash(uint8_t* frame);
void HandleMessage(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* FLASH_PAGE_SIZE should be able to get the size of the Page according to the controller */
//static uint32_t GetPage(uint32_t Address)
//{
//  for (int indx=0; indx<128; indx++)
//  {
//	  if((Address < (0x08000000 + (FLASH_PAGE_SIZE *(indx+1))) ) && (Address >= (0x08000000 + FLASH_PAGE_SIZE*indx)))
//	  {
//		  return (0x08000000 + FLASH_PAGE_SIZE*indx);
//	  }
//  }
//
//  return 0;
//}
void HandleMessage(void){

	//code array will contain 4 records
	// MyFrameShape
	// RecordSize--HighAddress--LowAddress--recordType--____DATA___---
	// __1 Byte__--__2_Bytes__--__2_Bytes_--__1 Byte__--__16 Byte__---
	static uint8_t codearray[MY_RECORD_SIZE*4];
	static uint8_t linesCounter;
	static uint8_t recordsize;
	static uint8_t lastrecordlocation;
	static uint8_t highAddress;
	static uint8_t isnewliine;
	//ciopy the received data to local var

	switch (RxHeader.StdId) {
		case APP_START_STDID_CAN:
			gotoAPP();
			HAL_TIM_Base_Stop_IT(&htim1);
			break;
		case FRUP_START_STDID_CAN:
			//erase flash then send ready
			HAL_TIM_Base_Stop_IT(&htim1);
			EraseFlash(MY_BL_SIZE_PAGES, FLASH_NUM_PAGES);
			MTxHeader.DLC=0;
			MTxHeader.StdId=FRUP_READY;
			//setnew line flag
			isnewliine=1;
			MTxHeader.StdId=FRUP_READY;
			HAL_CAN_AddTxMessage(&hcan, &MTxHeader, TxData, &Mailbox);
			break;
		case FRUP_RECORD_STDID_CAN :
			//if its a new line and have 04 that incates
			///the a new address at the high part
			if (isnewliine) {
				if (RxData[3]==0x04) {
					highAddress=(RxData[4]<<8)|(RxData[5]);
				}
				isnewliine=0;
				recordsize=TxData[0];
				//copy first Byte and add address in the second Byte
				codearray[(linesCounter*MY_RECORD_SIZE)+0]=TxData[0];
				codearray[(linesCounter*MY_RECORD_SIZE)+1]=highAddress;

					for (uint8_t var = 0; ((var < RxHeader.DLC)&&(var < recordsize)); ++var) {
						codearray[(linesCounter*MY_RECORD_SIZE)+2+var]=RxData[var+1];
					}
					lastrecordlocation=8;
				}
			else{
				for (uint8_t var = 0; ((var < RxHeader.DLC)&&(var < recordsize)); ++var) {
					//add 1Byte offset for the HIgh address part i added
					codearray[(linesCounter*MY_RECORD_SIZE)+1+lastrecordlocation+var]=RxData[var];
				}
			}

			break;
		case FRUP_NLINE_STDID_CAN:
			isnewliine=1;
			linesCounter++;
			break;
		default:
			break;
	}
	if(linesCounter==4){
		//writeDataToFlash(recordsize, tempdata, 4);
		programFlash(codearray);
		lastrecordlocation=0;
		linesCounter=0;
		MTxHeader.StdId=FRUP_NLINE_RDY_STDID_CAN;
		HAL_CAN_AddTxMessage(&hcan, &MTxHeader, TxData, &Mailbox);

	}
		//send here the ok value



}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{

	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);
//	MTxHeader.DLC = RxHeader.DLC;
//	MTxHeader.ExtId =RxHeader.ExtId;
//	MTxHeader.IDE = RxHeader.IDE;
//	MTxHeader.RTR =RxHeader.RTR;
//	MTxHeader.TransmitGlobalTime =DISABLE;
//	MTxHeader.StdId =RxHeader.StdId;

//	 HAL_CAN_AddTxMessage(hcan, &MTxHeader, RxData, &Mailbox);


}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	static uint8_t callCounter;
	if (htim->Instance == TIM1)
	    {
		 if (callCounter) {

		 	 HAL_TIM_Base_Stop_IT(&htim1);
	        // Call the function
	        gotoAPP();

	        // Stop the Timer

	    }}
	callCounter++;

}

void gotoAPP(void) {

	HAL_TIM_Base_Stop_IT(&htim1);
	//SCB->VTOR = FLASH_ADDRESS;
	Function_t jumpFunc;
	uint32_t jumpAddress;

	if (((*(uint32_t*) FLASH_ADDRESS) & 0x2FFE0000) == 0x20000000) {
		//addr_to_call = *(Function_t*) (FLASH_ADDRESS+4);
		//the first byte is reserved for stackpointer
		jumpAddress=*(uint32_t*)(FLASH_ADDRESS+4);
		jumpFunc = (Function_t) jumpAddress;

		//addr_to_call();
		SCB->VTOR = FLASH_ADDRESS;
		__set_MSP(FLASH_ADDRESS);
		jumpFunc();
	}
	else{

		//there is no apllication installed
	}
}


void EraseFlash(uint32_t pageAddress,uint32_t pagesNum){
    HAL_FLASH_Unlock();
    static FLASH_EraseInitTypeDef flsherastemp;
    flsherastemp.TypeErase=FLASH_TYPEERASE_PAGES;
    //flsherastemp.Banks=FLASH_BANK_1;
    flsherastemp.NbPages=pagesNum;
	uint32_t PAGEError;
    HAL_FLASH_Unlock();
    HAL_FLASHEx_Erase(&flsherastemp, &PAGEError);
	FLASH_WaitForLastOperation(5000);
	HAL_FLASH_Lock();

}


void programFlash(uint8_t* frame) {
    // Extract the record size, high address, low address, and record type from the frame
    for(uint8_t var=0;var<4;var++){
	uint8_t recordSize = frame[0+(var*MY_RECORD_SIZE)];
    uint16_t highAddress = (frame[1+(var*MY_RECORD_SIZE)] << 8) | frame[2+(var*MY_RECORD_SIZE)];
    uint16_t lowAddress = (frame[3+(var*MY_RECORD_SIZE)] << 8) | frame[4+(var*MY_RECORD_SIZE)];
    uint8_t recordType = frame[5+(var*MY_RECORD_SIZE)];

    // Pointer to the data in the frame
    uint8_t* data = &frame[6+(var*MY_RECORD_SIZE)];

    // Calculate the total number of bytes to be written to flash
    uint32_t totalBytes = recordSize - 5; // Record size minus overhead bytes (5 bytes)

    // Check if the record type is valid for programming flash
    if (recordType == 0x00) {
        // Disable interrupts while programming flash
        __disable_irq();

        // Unlock the flash memory
        HAL_FLASH_Unlock();

        // Erase the flash sector where the data will be written
        uint32_t flashAddress = (highAddress << 16) | lowAddress;

        // Program the data into flash
        for (uint32_t i = 0; i < totalBytes; i += 4) {
            uint32_t word = *(uint32_t*)(data + i);
            HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, flashAddress + i, word);
        }
    }

        // Lock the flash memory again
        HAL_FLASH_Lock();

        // Enable interrupts
        __enable_irq();
    }
}

/*
// Function to write data to flash memory
void writeDataToFlash(uint32_t address, uint8_t *data, uint32_t length)
{
    // Enable the Flash interface and wait for it to be ready
    HAL_FLASH_Unlock();
    while (HAL_FLASH_GetError()!= HAL_FLASH_ERROR_NONE)
    {
        // Wait for flash to be ready
    }
    // Write the data to flash memory
    for (uint32_t i = 0; i < length; i += 4)
    {
        uint32_t value = (uint32_t)(data[i]) << 24 |
                         (uint32_t)(data[i + 1]) << 16 |
                         (uint32_t)(data[i + 2]) << 8 |
                         (uint32_t)(data[i + 3]);

        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address + i, value);

    }

    // Lock the Flash interface
    HAL_FLASH_Lock();
}
*/
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */
  MTxHeader.DLC = 1;
  MTxHeader.ExtId = 0;
  MTxHeader.IDE = CAN_ID_STD;
  MTxHeader.RTR = CAN_RTR_DATA;
  MTxHeader.TransmitGlobalTime = DISABLE;
  MTxHeader.StdId = 0x1;

  HAL_CAN_ActivateNotification(&hcan,CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_CAN_Start(&hcan);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //HAL_TIMEx_OCN_Start(htim, Channel)
  MX_TIM1_Init();
  HAL_TIM_Base_Start_IT(&htim1);

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 65536;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
  CAN_FilterTypeDef CanFilterConfig;
    CanFilterConfig.FilterActivation = CAN_FILTER_ENABLE;
    CanFilterConfig.FilterBank = 0;
    CanFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    CanFilterConfig.FilterIdHigh = 0;
    CanFilterConfig.FilterIdLow = 0;
    CanFilterConfig.FilterMaskIdHigh = 0;
    CanFilterConfig.FilterMaskIdLow = 0;
    CanFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    CanFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    CanFilterConfig.SlaveStartFilterBank = 2;

    HAL_CAN_ConfigFilter(&hcan, &CanFilterConfig);
  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */
  //uwPrescalerValue = (uint32_t)(SystemCoreClock / 10000) - 1;
  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0xffff;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 50000 - 1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
