/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BL_DEBUG_UART_EN 
#define BL_RX_Length 200
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
GPIO_PinState state;
HAL_StatusTypeDef ret1;
uint8_t BL_RX_Buffer[BL_RX_Length];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CRC_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
static void printf1(char * format_string,...);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_CRC_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
	
	
	printf1("BL DEBUG MESSAGE: STM32 BOOTLOADER PROJECT \r\n");


	//================== wait for timeout to go to BL mode ==============================//

	 state= HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
	 if(state==GPIO_PIN_RESET)
	 {
		 printf1("BL DEBUG MESSAGE: Going into BL MODE BUTTON PRESSED.\r\n");
		 bootloader_read_uart();
	 }
	 else 
	 {
		 printf1("BL DEBUG MESSAGE: Going into user application MODE BUTTON not PRESSED.\r\n");
		 bootloader_jump_to_user_app();
	 }


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
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
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//=====================================================JUMP TO USER APPLICATION==========================================================================================
void  bootloader_jump_to_user_app(void){
	
	//1. store reset handler and msp value to set mso value and jump to reset handler
	//MSP value to be stoed 	
	uint32_t mspvalue = *(FLASH_SECTOR_2_BASE_ADDR);
	__set_MSP(mspvalue);
	
	//2. POINTER TO RESET HANDLER 
	//FUNCTION POINTER 
	void (*RESET_HANDLER)(void);
	RESET_HANDLER =((void (*)(void))*(FLASH_SECTOR_2_BASE_ADDR+1));
	
	//3. control jumps from bootloader to the reset handler of user application
	RESET_HANDLER();
	
}

//=====================================================RECEIVE COMMANDS FROM HOST=====================================================================
void  bootloader_read_uart(void){
	printf1("BL DEBUG MESSAGE: bootloader_read_uart executing...\r\n");
	
	uint8_t rx_length;
	
	while(1)
	{
//memset global varibale RXBuffer 
	memset(BL_RX_Buffer,0,200);//===============BL_RX_BUFFER contains the individual bytes corresponding to length, command code, crc etc
		
	//RECEIVING THE FIRST BYTE TO KNOW THE REMAINING NUMBER OF COMMAND BYTES TO BE RECEIVED 
	HAL_UART_Receive(&huart2,BL_RX_Buffer,1,HAL_MAX_DELAY);	
	rx_length=BL_RX_Buffer[0];
	//RECEIVE REMAINING NUMBER F BYTES USING HAL UART AND THE LENGTH OF RX BYTES VALUE:
	HAL_UART_Receive(&huart2,&BL_RX_Buffer[1],rx_length,HAL_MAX_DELAY);
		
	//==============DECODING AND HANDLING OF THE COMMAND CODE===========================//
		switch(BL_RX_Buffer[1])
		{
			
			case BL_GET_VER:
				bootloader_handle_getver_cmd(BL_RX_Buffer);
			break;
			case BL_GET_HELP:
				bootloader_handle_gethelp_cmd(BL_RX_Buffer);
			break;
			case BL_GET_CID:
				bootloader_handle_getcid_cmd(BL_RX_Buffer);
			break;
			case BL_GET_RDP_STATUS:
				bootloader_handle_getrdp_cmd(BL_RX_Buffer);
			break;
			case BL_GO_TO_ADDR:
				bootloader_handle_gotoaddr_cmd(BL_RX_Buffer);
			break;
			case BL_FLASH_ERASE:
				bootloader_handle_flasherase_cmd(BL_RX_Buffer);
			break;
			case BL_MEM_WRITE:
				bootloader_handle_memwrite_cmd(BL_RX_Buffer);
			break;
			case BL_EN_R_W_PROTECT:
				bootloader_handle_enrwprotect_cmd(BL_RX_Buffer);
			break;
			case BL_MEM_READ:
				bootloader_handle_memread_cmd(BL_RX_Buffer);
			break;
			case BL_READ_SECTOR_STATUS:
				bootloader_handle_readsectorstatus_cmd(BL_RX_Buffer);
			break;
			case BL_OTP_READ:
				bootloader_handle_otpread_cmd(BL_RX_Buffer);
			break;
			case BL_DIS_R_W_PROTECT:
				bootloader_handle_disrwprotect_cmd(BL_RX_Buffer);
			break;
		}
	}	
}


//======================BOOTLOADER COMMAND HANDLER FUNCTION PROTOTYPES===============
	void 	bootloader_handle_getver_cmd(uint8_t *BL_RX_Buffer)
	{
		printf1("BL DEBUG MESSAGE: bootloader_handle_getver_cmd executing...\r\n");
		
		//verify CRC
		uint8_t command_pack_length=BL_RX_Buffer[0]+1; //LENGTH OF FOLLOW BYTES +LENGTH OF 1ST BYTE
		uint32_t host_crc= *((uint32_t*)(BL_RX_Buffer+(command_pack_length-4)));
		
		if(!(bootloader_verify_crc(BL_RX_Buffer,command_pack_length -4,host_crc)))
		{
		printf1("BL DEBUG MESSAGE: CRC Verification success!\r\n");
		bootloader_send_ack(BL_RX_Buffer[1],1);//send the ack 
		//========================retrive the version adn send the version========================
			uint8_t bl_version = bootloader_get_version();
			printf1("BL DEBUG MESSAGE: Bootloader version retrieved: %d",bl_version);
			bootloader_uart_write_data(&bl_version,1);
		}
		
		else 
		{
		printf1("BL DEBUG MESSAGE: CRC Verification failed!\r\n");
		bootloader_send_nack();
		}
		
	}
	void				bootloader_handle_gethelp_cmd(uint8_t *BL_RX_Buffer)
	{
		printf1("BL DEBUG MESSAGE: bootloader_handle_getver_cmd executing...\r\n");
		
		//verify CRC
		uint8_t command_pack_length=BL_RX_Buffer[0]+1; //LENGTH OF FOLLOW BYTES +LENGTH OF 1ST BYTE
		uint32_t host_crc= *((uint32_t*)(BL_RX_Buffer+(command_pack_length-4)));
		
		if(!(bootloader_verify_crc(BL_RX_Buffer,command_pack_length -4,host_crc)))
		{
		printf1("BL DEBUG MESSAGE: CRC Verification success!\r\n");
		uint8_t bl_commands[] = {0x51,0x52,0x53,0x54,0x55,0x56,0x57,0x58,0x59,0x5A,0x5B,0X5C};	
		bootloader_send_ack(BL_RX_Buffer[1],sizeof(bl_commands));//send the ack 
		//========================retrive the version adn send the version========================
		bootloader_uart_write_data(bl_commands,sizeof(bl_commands));
		}
		
		else 
		{
		printf1("BL DEBUG MESSAGE: CRC Verification failed!\r\n");
		bootloader_send_nack();
		}
		
	}
	void				bootloader_handle_getcid_cmd(uint8_t *BL_RX_Buffer)
		{
			printf1("BL DEBUG MESSAGE: bootloader_handle_getcid_cmd executing...\r\n"); //================CHIP IDENTIFICATION NUMBER====================
		
		//verify CRC
		uint8_t command_pack_length=BL_RX_Buffer[0]+1; //LENGTH OF FOLLOW BYTES +LENGTH OF 1ST BYTE
		uint32_t host_crc= *((uint32_t*)(BL_RX_Buffer+(command_pack_length-4)));
		
		if(!(bootloader_verify_crc(BL_RX_Buffer,command_pack_length -4,host_crc)))
		{
		printf1("BL DEBUG MESSAGE: CRC Verification success!\r\n");
		bootloader_send_ack(BL_RX_Buffer[1],2);//send the ack 
		//========================retrive AND SEND chip id number========================
		uint16_t cid=bootloader_get_mcu_cid();	
		bootloader_uart_write_data((uint8_t *)&cid,2);
		}
		
		else 
		{
		printf1("BL DEBUG MESSAGE: CRC Verification failed!\r\n");
		bootloader_send_nack();
		}
			
	}
	void				bootloader_handle_getrdp_cmd(uint8_t*BL_RX_Buffer)
		{
			printf1("BL DEBUG MESSAGE: bootloader_handle_getrdp_cmd executing...\r\n"); //================CHIP IDENTIFICATION NUMBER====================
		
		//verify CRC
		uint8_t command_pack_length=BL_RX_Buffer[0]+1; //LENGTH OF FOLLOW BYTES +LENGTH OF 1ST BYTE
		uint32_t host_crc= *((uint32_t*)(BL_RX_Buffer+(command_pack_length-4)));
		
		if(!(bootloader_verify_crc(BL_RX_Buffer,command_pack_length -4,host_crc)))
		{
		printf1("BL DEBUG MESSAGE: CRC Verification success!\r\n");
		bootloader_send_ack(BL_RX_Buffer[1],1);//send the ack 
		//========================retrive RDP LEVEL===========================
		uint8_t rdp_level=bootloader_retrieve_rdp_level();
		bootloader_uart_write_data(&rdp_level,2);
		}
		
		else 
		{
		printf1("BL DEBUG MESSAGE: CRC Verification failed!\r\n");
		bootloader_send_nack();
		}
			
	}
	void				bootloader_handle_gotoaddr_cmd(uint8_t*pBuffer)
		{
	}
	void				bootloader_handle_flasherase_cmd(uint8_t*pBuffer)
		{
	}
	void				bootloader_handle_memwrite_cmd(uint8_t*pBuffer)
		{
	}
	void				bootloader_handle_enrwprotect_cmd(uint8_t*pBuffer)
		{
	}
	void				bootloader_handle_memread_cmd(uint8_t*pBuffer)
		{
	}
	void				bootloader_handle_readsectorstatus_cmd(uint8_t*pBuffer)
		{
	}
	void				bootloader_handle_otpread_cmd(uint8_t*pBuffer)
		{
	}
	void				bootloader_handle_disrwprotect_cmd(uint8_t*pBuffer)
		{
	}
//=============================bootloader_send_nack() and bootloader_send_ack()=====

	
	void bootloader_send_nack(void)
	{
		uint8_t nack= BL_NACK;
		HAL_UART_Transmit(&huart2,&nack,1,HAL_MAX_DELAY);
	}
		void bootloader_send_ack(uint8_t command_code,uint8_t length_of_follow_data) //========ALWAYS 2 BYTES IS SENT=============
	{
		
		uint8_t ack[2];
		ack[0]=BL_ACK;
		ack[1]=length_of_follow_data;
		HAL_UART_Transmit(&huart2,ack,2,HAL_MAX_DELAY);
		
	}
	//======================Bootloader verify CRC ============================================//
	
	uint8_t bootloader_verify_crc(uint8_t *pbuffer, uint8_t len_for_calc, uint32_t host_crc)
	{
		uint32_t crc_value=0xFFFFFFFF;
		for (int i=0;i<len_for_calc;i++)
		{
			uint32_t data_i= pbuffer[i];
			crc_value=HAL_CRC_Accumulate(&hcrc,&data_i,1);
		}
		if(crc_value==host_crc)
		{
			return CRC_VERIFY_SUCCESS;
		}
		else 
			return CRC_VERIFY_FAILURE;
		
	}
	//===================================GET BOOTLOADER VERSION ================================//
	uint8_t bootloader_get_version(void)
	{
		return BL_VERSION;
	}
	//==============================BOOTLOADER WRITE DATA =====================================//
	void bootloader_uart_write_data(uint8_t *pdata,uint16_t size_of_data)
	{
				HAL_UART_Transmit(&huart2,pdata,size_of_data,HAL_MAX_DELAY);
	}
	//============================================bootloader_get_mcu_cid=====================================================//
	uint16_t bootloader_get_mcu_cid(void)
	{
		uint16_t cid=(DBGMCU->IDCODE& 0x00000FFF);
		return cid;
	}
	//============================================bootloader_get_mcu_cid=====================================================//
	uint8_t bootloader_retrieve_rdp_level(void)
	{
		uint8_t * prdp =FLASH_OPB_RDP_LEVEL;
		uint8_t rdp_level= *(FLASH_OPB_RDP_LEVEL+1); 
		return rdp_level;
		
		
	}
	//=====================================================implementation of printf1() function using va_args=====================================================================
void printf1(char * format_string,...)
{
	#ifdef BL_DEBUG_UART_EN
	char buffer[100];
	va_list args;
	va_start(args,format_string);
	
	vsprintf(buffer,format_string,args);
	
	HAL_UART_Transmit(&huart3,(uint8_t *)buffer,strlen(buffer),HAL_MAX_DELAY);
	
	va_end(args);
	#endif
	
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
