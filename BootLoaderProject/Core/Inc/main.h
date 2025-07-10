/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
	void  bootloader_read_uart(void);
	void  bootloader_jump_to_user_app(void);


/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */
#define FLASH_SECTOR_2_BASE_ADDR   ((uint32_t *)0x08008000U)

//============================COMMAND CODES BL_RX_Buffer[1]=====================//
#define BL_GET_VER     				0x51
#define BL_GET_HELP     			0x52
#define BL_GET_CID     				0x53
#define BL_GET_RDP_STATUS   	0x54
#define BL_GO_TO_ADDR     		0x55
#define BL_FLASH_ERASE      	0x56
#define BL_MEM_WRITE     			0x57
#define BL_EN_R_W_PROTECT   	0x58
#define BL_MEM_READ     			0x59
#define BL_READ_SECTOR_STATUS 0x5A
#define BL_OTP_READ     			0x5B
#define BL_DIS_R_W_PROTECT  	0x5C
//================================================================================//
//======================BOOTLOADER COMMAND HANDLER FUNCTION PROTOTYPES===============
	void 				bootloader_handle_getver_cmd(uint8_t *pBuffer);
	void				bootloader_handle_gethelp_cmd(uint8_t *pBuffer);
	void				bootloader_handle_getcid_cmd(uint8_t *pBuffer);
	void				bootloader_handle_getrdp_cmd(uint8_t*pBuffer);
	void				bootloader_handle_gotoaddr_cmd(uint8_t*pBuffer);
	void				bootloader_handle_flasherase_cmd(uint8_t*pBuffer);
	void				bootloader_handle_memwrite_cmd(uint8_t*pBuffer);
	void				bootloader_handle_enrwprotect_cmd(uint8_t*pBuffer);
	void				bootloader_handle_memread_cmd(uint8_t*pBuffer);
	void				bootloader_handle_readsectorstatus_cmd(uint8_t*pBuffer);
	void				bootloader_handle_otpread_cmd(uint8_t*pBuffer);
	void				bootloader_handle_disrwprotect_cmd(uint8_t*pBuffer);

//=============================NACK AND ACK============================
#define BL_NACK 		0x7F
#define BL_ACK 			0xA5

#define CRC_VERIFY_SUCCESS  0
#define CRC_VERIFY_FAILURE  1

#define BL_VERSION 1

#define FLASH_OPB_RDP_LEVEL   ((uint8_t *)0x1FFFC000)

#define VALID 		0X00
#define INVALID 	0X01


#define FLASH_SIZE			512*1024
#define SRAM1_SIZE			112*1024
#define SRAM2_SIZE			16*1024
#define BKPSRAM_SIZE			4*1024

#define FLASH_END1      (FLASH_BASE+FLASH_SIZE)
#define SRAM1_END				(SRAM1_BASE+SRAM1_SIZE)
#define SRAM2_END				(SRAM2_BASE+SRAM2_SIZE)
#define BKPSRAM_END			(BKPSRAM_BASE+BKPSRAM_SIZE)


void bootloader_send_ack(uint8_t command_code,uint8_t length_of_follow_data);
void bootloader_send_nack(void);
uint8_t bootloader_verify_crc(uint8_t *pbuffer, uint8_t len_for_calc, uint32_t host_crc);
uint8_t bootloader_get_version(void);
void bootloader_uart_write_data(uint8_t *pdata,uint16_t size_of_data);
uint16_t bootloader_get_mcu_cid(void);
uint8_t bootloader_retrieve_rdp_level(void);
uint8_t verify_address(uint32_t goaddr);




/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
