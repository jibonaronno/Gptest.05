
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "lwip.h"
#include "lwip/apps/httpd.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "fsdata_custom.h"
#include "ds18b20_mflib.h"
#include "tm_onewire.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);

uint16_t *encrypt = (uint16_t *)0x1FFFF7E8;

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

ADC_HandleTypeDef hadc1;

IWDG_HandleTypeDef   IwdgHandle;

static void MX_ADC1_Init(void);

volatile uint32_t time_us;
uint32_t temp = 0;

uint8_t Mrelays[8];
uint8_t Midx = 0;
uint8_t MidxCounter = 0;
uint8_t SwitchStates = 0;

char strCmd01[40];

uint8_t flash[2048];

extern char idx_html[];
extern struct fsdata_file file__index_html[];

const char * LEDS_CGI_Handler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[]);

GPIO_PinState flag_systick01 = GPIO_PIN_RESET;
uint32_t systick_counter01 = 0;

GPIO_PinState flag_systick02 = GPIO_PIN_RESET;
uint32_t systick_counter02 = 0;

GPIO_PinState flag_systick03 = GPIO_PIN_RESET;
uint32_t systick_counter03 = 0;

GPIO_PinState PB6_State = GPIO_PIN_RESET;
uint32_t PB6_high_count = 0;

tCGI CGI_Handlers[] = {
	{"/", LEDS_CGI_Handler}, /* LEDS_CGI_Handler will be called when user connects to "/ledaction.cgi" URL */
};

void SetCGIHandlers(const tCGI *cgis, uint16_t number_of_handlers)
{
	http_set_cgi_handlers((tCGI *)cgis, number_of_handlers);
}

//--------------------------------------------------
/*
__STATIC_INLINE void DelayMicro(__IO uint32_t micros)
{
micros *= (SystemCoreClock / 1000000) / 9;
while (micros--) ;
} */
//--------------------------------------------------

extern uint8_t IP_ADDRESS[4];
extern uint8_t GATEWAY_ADDRESS[4];

uint16_t volt = 0;


uint8_t ip_addr1 = 10;
uint8_t ip_addr2 = 5;
uint8_t ip_addr3 = 40;
uint8_t ip_addr4 = 83;


/*
uint8_t ip_addr1 = 192;
uint8_t ip_addr2 = 168;
uint8_t ip_addr3 = 88;
uint8_t ip_addr4 = 22;
*/
/*
uint8_t ip_addr1 = 10;
uint8_t ip_addr2 = 0;
uint8_t ip_addr3 = 0;
uint8_t ip_addr4 = 22;
*/

char *ptr;

char uid[50];

char str_ip[22];

char str_Mrelays[22];

uint32_t gidx01 = 0;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	
	uint16_t volt_adc = 0;
	uint16_t adc4_acc_counter = 0;
	uint32_t adc4_acc = 0;
	uint32_t adc4_avg = 0;
	
	uid[0] = 0;

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
	
	MX_ADC1_Init();
	
	HAL_ADC_Init(&hadc1);
	
	HAL_ADC_Start(&hadc1);
	
	/*##-1- Check if the system has resumed from WWDG reset ####################*/
  if(__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) != RESET)
  { 
    /* Clear reset flags */
    __HAL_RCC_CLEAR_RESET_FLAGS();
  }
  else
  {
    /* WWDGRST flag is not set: Turn LED1 off */
  }
	
	/*##-2- Configure the WWDG peripheral ######################################*/
  /* WWDG clock counter = (PCLK1 (42MHz)/4096)/8) = 1281 Hz (~780 us) 
     WWDG Window value = 80 means that the WWDG counter should be refreshed only 
     when the counter is below 80 (and greater than 64) otherwise a reset will 
     be generated. 
     WWDG Counter value = 127, WWDG timeout = ~780 us * 64 = 49.9 ms */
  IwdgHandle.Instance = IWDG;

  IwdgHandle.Init.Prescaler = IWDG_PRESCALER_128;
  IwdgHandle.Init.Reload = 0x0AAA;
  //IwdgHandle.Init.Counter   = 127;
	
	IP_ADDRESS[0] = ip_addr1;
  IP_ADDRESS[1] = ip_addr2;
  IP_ADDRESS[2] = ip_addr3;
  IP_ADDRESS[3] = ip_addr4;
	
	GATEWAY_ADDRESS[0] = ip_addr1;
	GATEWAY_ADDRESS[1] = ip_addr2;
	GATEWAY_ADDRESS[2] = ip_addr3;
	GATEWAY_ADDRESS[3] = (ip_addr4 - 2);
	
	readSector(0x8012000, (void *)flash, 30);
	flash[19] = 0;
	
	strcat(str_ip, (char *)&flash[4]);
	
	flash[7] = 0;
	ip_addr1 = (uint8_t)strtol((char *)&flash[4], &ptr, 10);
	//ip_addr1 = (uint8_t)atoi((char *)&flash[5]);
	
	flash[11] = 0;
	ip_addr2 = (uint8_t)strtol((char *)&flash[8], &ptr, 10);
	//ip_addr2 = (uint8_t)atoi((char *)&flash[8]);
	
	flash[15] = 0;
	ip_addr3 = (uint8_t)strtol((char *)&flash[12], &ptr, 10);
	//ip_addr3 = (uint8_t)atoi((char *)&flash[12]);
	
	flash[19] = 0;
	ip_addr4 = (uint8_t)strtol((char *)&flash[16], &ptr, 10);
	//ip_addr4 = (uint8_t)atoi((char *)&flash[17]);
	
	if((ip_addr1 > 0) && (ip_addr2 > 0) && (ip_addr3 > 0) && (ip_addr4 > 0))
	{
		IP_ADDRESS[0] = ip_addr1;
		IP_ADDRESS[1] = ip_addr2;
		IP_ADDRESS[2] = ip_addr3;
		IP_ADDRESS[3] = ip_addr4;
		
		GATEWAY_ADDRESS[0] = ip_addr1;
		GATEWAY_ADDRESS[1] = ip_addr2;
		GATEWAY_ADDRESS[2] = ip_addr3;
		GATEWAY_ADDRESS[3] = (ip_addr4 - 2);
		
		for(gidx01=0;gidx01<8;gidx01++)
		{
			//flash[20+gidx01] = 0; //Mrelays[gidx01];
			Mrelays[gidx01] = flash[20+gidx01];
		}
	}
	else
	{
		sprintf(str_ip, "%d,%d,%d,%d", IP_ADDRESS[0], IP_ADDRESS[1], IP_ADDRESS[2], IP_ADDRESS[3]);
		
		for(gidx01=0;gidx01<8;gidx01++)
		{
			flash[20+gidx01] = 0; //Mrelays[gidx01];
		}
		
		for(gidx01=0;gidx01<8;gidx01++)
		{
			Mrelays[gidx01] = flash[20+gidx01];
		}
		
		strncpy((char *)flash, "dip:010.005.040.083", 19);
		
		eraseSector(0x8012000);
		HAL_Delay(100);
		writeSector(0x8012000, flash, 30); //++
		HAL_Delay(200);

	}
	
	sprintf(str_Mrelays, "%d-%d-%d-%d-%d-%d-%d-%d", Mrelays[0], Mrelays[1], Mrelays[2], Mrelays[3], Mrelays[4], Mrelays[5], Mrelays[6], Mrelays[7]);
	
	SwitchStates = ((Mrelays[0] * 1) + (Mrelays[1] * 2) + (Mrelays[2] * 4) + (Mrelays[3] * 8) + (Mrelays[4] * 16) + (Mrelays[5] * 32));
	
	HAL_Delay(200);
	HAL_Delay(200);
	HAL_Delay(200);
	HAL_Delay(200);
	HAL_Delay(200);
	
  MX_LWIP_Init();
  /* USER CODE BEGIN 2 */
	
	HAL_Delay(200);
	HAL_Delay(200);
	HAL_Delay(200);
	HAL_Delay(200);
	HAL_Delay(200);
	
	httpd_init();
	
	SetCGIHandlers(CGI_Handlers, 1);
	
	ds18b20_init_seq();
	ds18b20_send_rom_cmd(SKIP_ROM_CMD_BYTE);
	ds18b20_send_function_cmd(CONVERT_T_CMD);
	
	if(HAL_IWDG_Init(&IwdgHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
	
	//(encrypt[1] != 0x05A)
	//FF37-5D7-524D-3938-4528-4324
	//FF37-5D6-524D-3938-5628-4324
	//FF37-5D6-524D-3938-4628-4324
	//FF37-5D7-524D-3938-4232-4324
	//FF37-5DA-524D-3938-3057-4324
	//FF37-5D5-524D-3938-4828-4324
	//FF37-5DB-524D-3938-5428-4324
	//FF37-5D5-524D-3938-5028-4324
	//FF37-5D4-524D-3938-4928-4324
	//FF37-5D8-524D-3938-5528-4324
	
	//FF37-5D5-524D-3938-3630-4323
	//FF37-5D5-524D-3938-3332-4324
	//FF37-5D7-524D-3938-5529-4324
	//FF37-5DE-524D-3938-5129-4324
	//FF37-5D2-524D-3938-3132-4324
	//FF37-5D3-524D-3938-5232-4324
	//FF37-5D5-524D-3938-2932-4324
	//FF37-5D9-524D-3938-3532-4324
	//FF37-5DD-524D-3938-6156-4323
	//FF37-5D3-524D-3938-2832-4324
	//FF37-5D3-524D-3938-5433-4324
	//FF37-5D4-524D-3938-5128-4324
	//FF37-5D5-524D-3938-5429-4324
	//FF37-5D7-524D-3938-5029-4324
	//FF37-5D3-524D-3938-3431-4324
	//FF37-5D6-524D-3938-3032-4324
	//FF37-5D9-524D-3938-5328-4324
	//FF37-5D7-524D-3938-4428-4324
	//FF37-5D8-524D-3938-5633-4324
	//FF37-5D9-524D-3938-2831-4324
	//FF37-5D3-524D-3938-2732-4324
	//FF37-5D5-524D-3938-3031-4324
	//FF37-5D8-524D-3938-3331-4324
	//FF37-5D8-524D-3938-4829-4324
	//FF37-5D8-524D-3938-4929-4324
	//FF37-5D4-524D-3938-3432-4324
	//FF37-5D6-524D-3938-5228-4324
	//FF37-5D4-524D-3938-3231-4324
	
	if((encrypt[0] != 0xFF37) || (encrypt[1] != 0x05D4) || (encrypt[2] != 0x524D) || (encrypt[3] != 0x3938) || (encrypt[4] != 0x3231) || (encrypt[5] != 0x4324))
	{
		while(1)
		{
			HAL_Delay(200);
		}
	}
	
	
	
	
	sprintf(uid, "%X %X %X %X", encrypt[0], encrypt[1], encrypt[2], encrypt[3]);
	

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
		//HAL_Delay(200);
		//HAL_Delay(200);
		//HAL_Delay(200);
		//HAL_Delay(200);
		MX_LWIP_Process();
		
		if(flag_systick03)
		{
			flag_systick03 = GPIO_PIN_RESET;
			volt_adc = HAL_ADC_GetValue(&hadc1);
			HAL_ADC_Start(&hadc1);
			
			if(adc4_acc_counter < 100)
			{
				adc4_acc += (uint32_t)volt_adc;
				adc4_acc_counter++;
			}
			else
			{
				adc4_acc_counter = 0;
				adc4_avg = (adc4_acc / 100UL);
				volt = (uint16_t)((adc4_avg * 100) / 429);
				adc4_acc = 0;
			}
			
			PB6_State = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6);
			
			if(PB6_State == GPIO_PIN_RESET)
			{
				if(PB6_high_count < 2000)
				{
					PB6_high_count++;
				}
				else
				{
					eraseSector(0x8012000);
					HAL_Delay(100);
					while(1)
					{
						HAL_Delay(200);
					}
				}
			}
			else
			{
				PB6_high_count = 0;
			}
			
		}
		
		if(flag_systick01)
		{
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
			flag_systick01 = GPIO_PIN_RESET;
			//sprintf((char *)idx_html, "HTTP/1.0 200 OK\r\nContent-type: text/html\r\n\r\n10.222.202.59:0,11,%2d,%02d,%03d", SwitchStates, temp, volt); //, uid); //, strCmd01);
			//sprintf((char *)idx_html, "HTTP/1.0 200 OK\r\nContent-type: text/html\r\n\r\n%s:0,11,%2d,%02d,%03d", str_ip, SwitchStates, temp, volt); //, uid); //, strCmd01);
			sprintf((char *)idx_html, "%s:0,11,%d,%d,%d", str_ip, SwitchStates, temp, volt);
			//sprintf((char *)idx_html, " %s ", uid); //, strCmd01);
			file__index_html[0].len = strlen(idx_html);
			
			HAL_IWDG_Refresh(&IwdgHandle);
		}
		
		if(flag_systick02)
		{
			flag_systick02 = GPIO_PIN_RESET;
			
			ds18b20_init_seq();
			ds18b20_send_rom_cmd(SKIP_ROM_CMD_BYTE);
			ds18b20_send_function_cmd(CONVERT_T_CMD);
			
			HAL_Delay(1);

			ds18b20_init_seq();
			ds18b20_send_rom_cmd(SKIP_ROM_CMD_BYTE);
			ds18b20_send_function_cmd(READ_SCRATCHPAD_CMD);
			temp = ds18b20_read_temp();	// returns float value
			
			if(Mrelays[0] == 1)
			{
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
			}
			else
			{
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
			}
			
			if(Mrelays[1] == 1)
			{
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
			}
			else
			{
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
			}
			
			if(Mrelays[2] == 1)
			{
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
			}
			else
			{
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
			}
			
			if(Mrelays[3] == 1)
			{
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
			}
			else
			{
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
			}
			
			if(Mrelays[4] == 1)
			{
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
			}
			else
			{
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
			}
			
			if(Mrelays[5] == 1)
			{
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
			}
			else
			{
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
			}
			
		}

  }
  /* USER CODE END 3 */
}

void HAL_SYSTICK_Callback(void)
{
	if(systick_counter01 < 200)
	{
		systick_counter01++;
	}
	else
	{
		flag_systick01 = GPIO_PIN_SET;
		systick_counter01 = 0;
	}
	
	if(systick_counter02 < 800)
	{
		systick_counter02++;
	}
	else
	{
		flag_systick02 = GPIO_PIN_SET;
		systick_counter02 = 0;
	}
	
	if(systick_counter03 < 2)
	{
		systick_counter03++;
	}
	else
	{
		systick_counter03 = 0;
		flag_systick03 = GPIO_PIN_SET;
	}
}

const char* LEDS_CGI_Handler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[]) 
{
	
	uint8_t i = 0;
//	uint32_t iLen = 0;
	
	if (iIndex == 0) 
	{
		if(strstr(pcParam[i], "command"))
			{
				if(strstr(pcValue[i], "a0"))
				{
					Mrelays[0] = 0;
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
				}
				else if(strstr(pcValue[i], "a1"))
				{
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
					Mrelays[0] = 1;
				}
				else if(strstr(pcValue[i], "b0"))
				{
					Mrelays[1] = 0;
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
				}
				else if(strstr(pcValue[i], "b1"))
				{
					Mrelays[1] = 1;
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
				}
				else if(strstr(pcValue[i], "c0"))
				{
					Mrelays[2] = 0;
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
				}
				else if(strstr(pcValue[i], "c1"))
				{
					Mrelays[2] = 1;
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
				}
				else if(strstr(pcValue[i], "d0"))
				{
					Mrelays[3] = 0;
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
				}
				else if(strstr(pcValue[i], "d1"))
				{
					Mrelays[3] = 1;
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
				}
				else if(strstr(pcValue[i], "e0"))
				{
					Mrelays[4] = 0;
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
				}
				else if(strstr(pcValue[i], "e1"))
				{
					Mrelays[4] = 1;
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
				}
				else if(strstr(pcValue[i], "f0"))
				{
					Mrelays[5] = 0;
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
				}
				else if(strstr(pcValue[i], "f1"))
				{
					Mrelays[5] = 1;
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
				}				
				else if(strstr(pcValue[i], "dip"))
				{
					//Mrelays[4] = 1;
					//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
					//memcpy(flash, pcValue[i], 20); //++
					//memcpy(&flash[20], Mrelays, 8); //++
					
					for(gidx01 = 0; gidx01 < 20; gidx01++)
					{
						flash[gidx01] = pcValue[i][gidx01];
					}
					
					for(gidx01 = 0; gidx01 < 8; gidx01++)
					{
						flash[20 + gidx01] = Mrelays[gidx01];
					}
					
					eraseSector(0x8012000);
					HAL_Delay(200);
					HAL_Delay(200);
					//writeSector(0x8012000, pcValue[i], 21); //--
					writeSector(0x8012000, flash, 30); //++
					HAL_Delay(200);
					HAL_Delay(200);
					while(1){;}
				}
				else if(strstr(pcValue[i], "uid"))
				{
					sprintf(uid, "%X %X %X %X", encrypt[0], encrypt[1], encrypt[2], encrypt[3]);
				}
				
				sprintf(strCmd01, "%s %s", pcParam[i], pcValue[i]);
			}
	}
	
	SwitchStates = ((Mrelays[0] * 1) + (Mrelays[1] * 2) + (Mrelays[2] * 4) + (Mrelays[3] * 8) + (Mrelays[4] * 16) + (Mrelays[5] * 32));
	
	/****************************************************************************************************************/
	//memcpy(flash, pcValue[i], 20); //++
	//memcpy(&flash[20], Mrelays, 8); //++
	
	for(gidx01 = 0; gidx01 < 8; gidx01++)
	{
		flash[20+gidx01] = Mrelays[gidx01];
	}
	
	eraseSector(0x8012000);
	HAL_Delay(100);
	//writeSector(0x8012000, pcValue[i], 21); //--
	writeSector(0x8012000, flash, 30); //++
	HAL_Delay(200);
	
	/****************************************************************************************************************/
	
	//sprintf((char *)idx_html, "HTTP/1.0 200 OK\r\nContent-type: text/html\r\n\r\n%s:0,11,%2d,%02d,%03d", str_ip, SwitchStates, temp, volt); //, uid); //, strCmd01);
	sprintf((char *)idx_html, "%s:0,11,%d,%d,%d", str_ip, SwitchStates, temp, volt);
	//sprintf((char *)idx_html, " %s ", uid);
	
	flag_systick01 = GPIO_PIN_SET;
	
	return "/index.html";
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
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
  RCC_OscInitStruct.Prediv1Source = RCC_PREDIV1_SOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL2.PLL2State = RCC_PLL_NONE;
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

    /**Configure the Systick interrupt time 
    */
  __HAL_RCC_PLLI2S_ENABLE();

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
