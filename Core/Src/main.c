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

#define ST1_BOOTLOADER			1
#define ST2_BOOTLOADER			2

/*change this macro according to the Desired ST*/
#define ST_BOOTLOADER			ST1_BOOTLOADER

#define FLASH_NUM_PAGES			64
#define MY_RECORD_SIZE			22
#define MY_BL_SIZE_PAGES		20
/* ---------------------------CAN MSG IDS--------------------*/
#if (ST_BOOTLOADER==ST1_BOOTLOADER)
/***********************Incoming Messages************************/
#define APP_START_STDID_CAN			0x22      /*Skip BootLoader and start */
#define FRUP_START_STDID_CAN		0x16      /*BootLoader enter firmware Upgrade Mode */
#define FRUP_RECORD_STDID_CAN		0x23      /*Data in hex transfer */
#define FRUP_NLINE_STDID_CAN		0x26      /*New Line in hex transfer */
/***********************Outgoing Messages************************/
#define FRUP_READY					0x27      /*ready to receive firmware*/
#define FRUP_NLINE_RDY_STDID_CAN	0x29      /*ready to receive next Data*/
#define ECU_START_SIGNAL			0x19      /*the microController started signal*/
#else
/***********************Incoming Messages************************/
#define APP_START_STDID_CAN			0x21		/*Skip BootLoader and start */
#define FRUP_START_STDID_CAN		0x17	/*BootLoader enter firmware Upgrade Mode */
#define FRUP_RECORD_STDID_CAN		0x24	/*Data in hex transfer */
#define FRUP_NLINE_STDID_CAN		0x25	/*New Line in hex transfer */


/***********************Outgoing Messages************************/
#define FRUP_READY					0x27	/*ready to receive firmware*/
#define FRUP_NLINE_RDY_STDID_CAN	0x29	/*ready to receive next Data*/
#define ECU_START_SIGNAL			0x20	/*the microController started signal*/

#endif
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */
CAN_TxHeaderTypeDef MTxHeader;
CAN_RxHeaderTypeDef RxHeader;
uint8_t jump=0;
uint8_t TxData[8]={1};
uint8_t RxData[8];
uint32_t Mailbox=CAN_TX_MAILBOX0;
uint8_t resend=0;
typedef void (*PFunc_t)(void);
//Function_t addr_to_call = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
static void gotoAPP(void);
void EraseFlash(uint32_t pageNumber,uint32_t pagesNum);
void programFlash(uint8_t* frame,uint8_t linesNum);
void HandleMessage(void);
uint32_t getAddressFromPage(uint32_t pageNum);
void resetSysTick(void);
void DeinitEverything();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{

	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);
	HandleMessage();

}




void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	static uint8_t callCounter;
	if (htim->Instance == TIM1)
	    {
		 if (callCounter==2) {

		 	 HAL_TIM_Base_Stop_IT(&htim1);
		 	 jump=1;

	    }}
	callCounter++;

}


void DeinitEverything()
{
	//-- reset peripherals to guarantee flawless start of user application
	HAL_TIM_Base_DeInit(&htim1);
	 HAL_CAN_DeactivateNotification(&hcan,CAN_IT_RX_FIFO0_MSG_PENDING);
	 HAL_CAN_Stop(&hcan);
		HAL_GPIO_DeInit(GPIOA, GPIO_PIN_12);
		HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11);
	 HAL_CAN_DeInit(&hcan);
	  __HAL_RCC_GPIOC_CLK_DISABLE();
	  __HAL_RCC_GPIOD_CLK_DISABLE();
	  __HAL_RCC_GPIOB_CLK_DISABLE();
	  __HAL_RCC_GPIOA_CLK_DISABLE();
	HAL_RCC_DeInit();
	HAL_DeInit();
	SysTick->CTRL = 0;
	SysTick->LOAD = 0;
	SysTick->VAL = 0;
}

static void gotoAPP(void) {

	HAL_TIM_Base_Stop_IT(&htim1);
	 // HAL_CAN_DeactivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
	//SCB->VTOR = FLASH_ADDRESS;
	PFunc_t jumpFunc;
	uint32_t jumpAddress;

	if (((*(uint32_t*) FLASH_ADDRESS) & 0x2FFE0000) == 0x20000000) {
		//the first byte is reserved for stackpointer
		jumpAddress=*(uint32_t*)(FLASH_ADDRESS+4);
		jumpFunc = (PFunc_t) jumpAddress;
		__set_MSP(*(__IO uint32_t* ) FLASH_ADDRESS);
		uint32_t newVectorTableAddress = 0x08005000 & 0xFFFFFFF8;
		SCB->VTOR = newVectorTableAddress;
		jumpFunc();

		/*
		uint32_t newVectorTableAddress = 0x08005000 & 0xFFFFFFF8;
		SCB->VTOR = newVectorTableAddress;
		const JumpStruct* vector_p = (JumpStruct*)FLASH_ADDRESS;
		DeinitEverything();
		 asm("msr msp, %0; bx %1;" : : "r"(vector_p->stack_addr), "r"(vector_p->func_p));
*/
	}
	else{

		//there is no apllication installed
	}
}


// Function to write data to flash memory

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
  MX_TIM1_Init();
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
  MTxHeader.StdId=ECU_START_SIGNAL;
  HAL_CAN_AddTxMessage(&hcan, &MTxHeader, TxData, &Mailbox);
  MX_TIM1_Init();
  HAL_TIM_Base_Start_IT(&htim1);
  while (1)
  {
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
			  if(jump==1){
				  gotoAPP();
			  }
			  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
			  HAL_Delay(100);
	  }
  }
  /* USER CODE END 3 */


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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  hcan.Init.Prescaler = 18;
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
  htim1.Init.Prescaler = 0x0fff;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
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


void EraseFlash(uint32_t pageNumber,uint32_t pagesNum){
    HAL_FLASH_Unlock();
    static FLASH_EraseInitTypeDef flsherastemp;
    flsherastemp.TypeErase=FLASH_TYPEERASE_PAGES;
    flsherastemp.NbPages=pagesNum;
    flsherastemp.PageAddress=getAddressFromPage(pageNumber);
    //flsherastemp.Banks=FLASH_BANK_1;
	uint32_t PAGEError;
    HAL_FLASHEx_Erase(&flsherastemp, &PAGEError);
	FLASH_WaitForLastOperation(5000);
	HAL_FLASH_Lock();

}

/**
 * @brief Handles incoming CAN messages and processes them accordingly.
 *
 * This function is responsible for processing different CAN messages received
 * and taking appropriate actions based on the message content and type.
 *
 * @note This function expects specific CAN messages with predefined message IDs,
 *       and it performs actions based on the received data and message ID.
 *
 * @note This function utilizes various static variables to keep track of the
 *       received data and the state of the processing.
 *
 * @retval None.
 */
void HandleMessage(void){

	//code array will contain 4 records
	// MyFrameShape
	// RecordSize--HighAddress--LowAddress--recordType--____DATA___---
	// __1 Byte__--__2_Bytes__--__2_Bytes_--__1 Byte__--__16 Byte__---
	static uint8_t codearray[MY_RECORD_SIZE*4];
	static uint8_t linesCounter;
	static uint8_t recordsize;
	static uint8_t lastrecordlocation;
	static uint16_t highAddress;
	static uint8_t isnewliine;
	//ciopy the received data to local var

	switch (RxHeader.StdId) {
		//signal that indicates direct start command
		case APP_START_STDID_CAN:
			HAL_TIM_Base_Stop_IT(&htim1);
			jump=1;
			MTxHeader.StdId=FRUP_NLINE_RDY_STDID_CAN;
			HAL_CAN_AddTxMessage(&hcan, &MTxHeader, TxData, &Mailbox);
			break;
		//signal that indicates there is a file will be sent
		case FRUP_START_STDID_CAN:
			//erase flash then send ready
			HAL_TIM_Base_Stop_IT(&htim1);
			EraseFlash(MY_BL_SIZE_PAGES, FLASH_NUM_PAGES-MY_BL_SIZE_PAGES);
			MTxHeader.DLC=0;
			MTxHeader.StdId=FRUP_READY;
			//setnew line flag
			isnewliine=1;
			MTxHeader.StdId=FRUP_READY;
			HAL_CAN_AddTxMessage(&hcan, &MTxHeader, TxData, &Mailbox);
			MTxHeader.StdId=FRUP_NLINE_RDY_STDID_CAN;
			HAL_CAN_AddTxMessage(&hcan, &MTxHeader, TxData, &Mailbox);
			break;
		case FRUP_RECORD_STDID_CAN :
			//if its a new line and have 04 that incates
			///the a new address at the high part
			if (isnewliine) {
				if (RxData[3]==0x04) {
					highAddress=(RxData[4]<<8)|(RxData[5]);
				}
				//Indicates end of file received
				else if(RxData[3]==0x01){
					//if the current record is end of file,write the current date in buffer to flash
					programFlash(codearray,linesCounter);
					jump=1;
					break;

				}
				isnewliine=0;
				recordsize=RxData[0];
				//copy first Byte and add address in the second AND THIRD Byte
				codearray[(linesCounter*MY_RECORD_SIZE)+0]=RxData[0];
				codearray[(linesCounter*MY_RECORD_SIZE)+1]=(uint8_t)(highAddress>>8);
				codearray[(linesCounter*MY_RECORD_SIZE)+2]=(uint8_t)highAddress;
					//record size +3 to add header
					for (uint8_t var = 0; ((var < RxHeader.DLC)&&(var < recordsize+3)); ++var) {
						codearray[(linesCounter*MY_RECORD_SIZE)+3+var]=RxData[var+1];
					}
					lastrecordlocation=8;
				}
			else{
				//this variable indicates the current increment inside for loop
				uint8_t currentRecordLoc=0;
				for (uint8_t var = 0; ((var < RxHeader.DLC)&&(var < recordsize)&&((lastrecordlocation+var)<recordsize+4)); ++var) {
					//add 2Byte offset for the HIgh address part i added
					codearray[(linesCounter*MY_RECORD_SIZE)+2+lastrecordlocation+var]=RxData[var];
					currentRecordLoc++;

				}
				lastrecordlocation+=currentRecordLoc;
			}
			MTxHeader.StdId=FRUP_NLINE_RDY_STDID_CAN;
			HAL_CAN_AddTxMessage(&hcan, &MTxHeader, TxData, &Mailbox);

			break;
			//a message that indicates there is a new line in hex file
		case FRUP_NLINE_STDID_CAN:
			isnewliine=1;
			linesCounter++;
			if(linesCounter==4){
					//writeDataToFlash(data, number of records);
					programFlash(codearray,linesCounter);
					lastrecordlocation=0;
					linesCounter=0;

				}
			MTxHeader.StdId=FRUP_NLINE_RDY_STDID_CAN;
			HAL_CAN_AddTxMessage(&hcan, &MTxHeader, TxData, &Mailbox);
			break;
		default:
			break;
	}
	//send here the ok value





}
/**
 * @brief Programs the provided data frames into the Flash memory.
 *
 * This function takes an array of data frames along with the number of frames to be programmed
 * and writes the data contained in those frames into the Flash memory.
 *
 * @param frame Pointer to an array of data frames.
 * @param linesNum Number of data frames to be programmed.
 *
 * @note The provided data frames must adhere to a specific format, including the record size,
 *       high and low addresses, record type, and data. Only records with a valid record type (0x00)
 *       will be programmed into the Flash memory.
 *       @ref MyFrameShape
 *
 * @warning This function disables interrupts during the Flash programming process. It is the caller's
 *          responsibility to ensure that interrupt-sensitive operations are handled appropriately.
 *
 * @note This function unlocks the Flash memory, erases the appropriate sector, and programs
 *       the data into the Flash memory.
 *
 * @note It is important to check the Flash programming specifications of the target microcontroller
 *       and ensure proper Flash sector alignment and erase before using this function.
 *
 * @retval None.
 */
void programFlash(uint8_t* frame,uint8_t linesNum) {
    // Extract the record size, high address, low address, and record type from the frame
    for(uint8_t var=0;var<linesNum;var++){
	uint8_t recordSize = frame[0+(var*MY_RECORD_SIZE)];
    uint16_t highAddress = (frame[1+(var*MY_RECORD_SIZE)] << 8) | frame[2+(var*MY_RECORD_SIZE)];
    uint16_t lowAddress = (frame[3+(var*MY_RECORD_SIZE)] << 8) | frame[4+(var*MY_RECORD_SIZE)];
    uint8_t recordType = frame[5+(var*MY_RECORD_SIZE)];

    // Pointer to the data in the frame
    uint8_t* data = &frame[6+(var*MY_RECORD_SIZE)];

    // Calculate the total number of bytes to be written to flash
    uint32_t totalBytes = recordSize - 0; // Record size minus overhead bytes (5 bytes)

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

// Function to calculate the address from a given page
uint32_t getAddressFromPage(uint32_t pageNum) {
    // Flash start address
    uint32_t flashStartAddress = 0x08000000;

    // Page size in bytes
    uint32_t pageSize = 1024;

    // Calculate the address for the given page
    uint32_t address = flashStartAddress + (pageNum * pageSize);

    return address;
}

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
