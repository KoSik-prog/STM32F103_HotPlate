/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "oled_ssd1306.h"
#include "buzzer.h"
#include "max6675.h"
#include "usbd_cdc_if.h"
#include "pid.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_spi2_rx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
#define MENU_POS_COUNT 3 //number of menu items

extern uint8_t oledRefreshActiveFlag;

uint16_t MessageLength = 0;
uint8_t TxBuffer[40];
uint8_t RxBuffer[40];

uint8_t buf[32];

double bedTemperature = 0;
double expectedBedTemperature = 0;
double pcbTemperature = 0;

int16_t menuPos = 1;
int16_t menuYPos = 0;
uint8_t menuActiveFlag = 1;
uint8_t buttonFlag = 0;

//
double bedPower = 0;

// PID
PID_TypeDef bedTempPID;
double bedTempPIDSetpoint = 0.0;

uint8_t readTemperatureFlag = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
void click_callback(uint8_t direction);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM2){
		oledRefreshActiveFlag = 1;
		oledRefreshAll(&hspi1);
	}
}


void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi){
	oledRefreshAll(&hspi1);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == BUTTON_1_Pin){
		if(buttonFlag == 0){ //debouncing
			buttonFlag = 1;
			if(menuActiveFlag == 1){
					click_callback(2);
				}
			beep(500, 30);
		}
	}
	if(GPIO_Pin == ENC_1_Pin){
		beep(1500, 10);
		if(HAL_GPIO_ReadPin(ENC_2_GPIO_Port, ENC_2_Pin) == GPIO_PIN_SET){
			click_callback(0);
		} else {
			click_callback(1);
		}
	}

}

uint8_t station_init(void){
	uint8_t errorFlag = 0;
	char errorDescription[20];
	uint16_t bufTemp;

	uint8_t status = max6675_read_temperature(&hspi2, MAX6675_1_CS_GPIO_Port, MAX6675_1_CS_Pin, &bufTemp);
	if(status != 0){
		errorFlag = 1;
		sprintf(errorDescription, "temp sensor 1");
	}
    status = max6675_read_temperature(&hspi2, MAX6675_2_CS_GPIO_Port, MAX6675_2_CS_Pin, &bufTemp);
    if(status != 0){
		errorFlag = 1;
		sprintf(errorDescription, "temp sensor 2");
	}
    return errorFlag;
}

void read_bed_temperature(double *temperature){
	uint16_t bufTemp;

	max6675_read_temperature(&hspi2, MAX6675_2_CS_GPIO_Port, MAX6675_2_CS_Pin, &bufTemp);
	*temperature = (double)bufTemp;
}

void read_pcb_temperature(double *temperature){
	uint16_t bufTemp;

	max6675_read_temperature(&hspi2, MAX6675_1_CS_GPIO_Port, MAX6675_1_CS_Pin, &bufTemp);
	*temperature = (double)bufTemp;
}

void display_menu_line(int8_t x, int8_t y, const uint8_t *text){
	#define FONT_HEIGHT 18

	oledDispTxt(x, y, text, Font_11x18, 1);
	oledDrawHorizontalLine(y -1, 0);
	oledDrawHorizontalLine(y + FONT_HEIGHT + 1, 0);
}

void display_main_menu(uint8_t position){
	#define CENTER_POS (64/2) - (18/2) //displayHeight/2 - fontSize/2
	#define SPACING 27
	uint8_t xPos = 25;

	position--;

	oledDrawHorizontalBox((-position * SPACING) + CENTER_POS - SPACING, 18, 0); //clear box
	for(uint8_t i=28; i<33; i++){ //selected point
		for(uint8_t q=0; q<3; q++){
			oledDrawPoint(q, i, 1);
		}
	}
	sprintf((char*)buf,"Solder  ");
	if(position == 0){xPos = 7;} else {xPos = 25;}
	display_menu_line(xPos, (-position * SPACING) + CENTER_POS, buf);
	sprintf((char*)buf,"Preheat ");
	if(position == 1){xPos = 7;} else {xPos = 25;}
	display_menu_line(xPos, (-position * SPACING) + SPACING + CENTER_POS, buf);
	sprintf((char*)buf,"Reflow  ");
	if(position == 2){xPos = 7;} else {xPos = 25;}
	display_menu_line(xPos, (-position * SPACING) + SPACING*2 + CENTER_POS, buf);
	sprintf((char*)buf,"Settings");
	if(position == 3){xPos = 7;} else {xPos = 25;}
	display_menu_line(xPos, (-position * SPACING) + SPACING*3 + CENTER_POS, buf);
	oledDrawHorizontalBox((-position * SPACING) + SPACING*4 + CENTER_POS, 18, 0); //clear box
}

void display_submenu_1(uint16_t mainPos, uint16_t position){
	#define CENTER_POS (64/2) - (18/2) //displayHeight/2 - fontSize/2
	#define SPACING 27
	uint8_t xPos = 25;

	position--;

	oledDrawHorizontalBox((-position * SPACING) + CENTER_POS - SPACING, 18, 0); //clear box
		for(uint8_t i=28; i<33; i++){ //selected point
			for(uint8_t q=0; q<3; q++){
				oledDrawPoint(q, i, 1);
			}
		}
	sprintf((char*)buf,"Set temp  ");
	if(position == 0){xPos = 7;} else {xPos = 25;}
	display_menu_line(xPos, (-position * SPACING) + CENTER_POS, buf);
	sprintf((char*)buf,"Start     ");
	if(position == 1){xPos = 7;} else {xPos = 25;}
	display_menu_line(xPos, (-position * SPACING) + SPACING + CENTER_POS, buf);
	sprintf((char*)buf,"Back      ");
	if(position == 2){xPos = 7;} else {xPos = 25;}
	display_menu_line(xPos, (-position * SPACING) + SPACING*2 + CENTER_POS, buf);
	oledDrawHorizontalBox((-position * SPACING) + SPACING*3 + CENTER_POS, 18, 0); //clear box
}

void display_preheat_set(void){
	if(readTemperatureFlag == 0){
		readTemperatureFlag = 1;
	}
	sprintf((char*)buf, "Set temp: ");
	oledDispTxt(0, 2, buf, Font_11x18, 1);
	if(bedTemperature < 100){
		sprintf((char*)buf," %.0f*C  ", expectedBedTemperature);
	} else {
		sprintf((char*)buf,"%.0f*C  ", expectedBedTemperature);
	}
	oledDispTxt(33, 35, buf, Font_11x18, 1);
}

void display_preheat_start(void){
	if(readTemperatureFlag == 0){
		readTemperatureFlag = 1;
	}
	sprintf((char*)buf, "Bed temp: ");
	oledDispTxt(0, 2, buf, Font_11x18, 1);
	if(bedTemperature < 100){
		sprintf((char*)buf," %.0f*C  ", bedTemperature);
	} else {
		sprintf((char*)buf,"%.0f*C  ", bedTemperature);
	}
	oledDispTxt(33, 35, buf, Font_11x18, 1);
}

void display_menu(void){
	if(menuPos >=100 && menuPos < 1000){
		if(menuPos == 122){
			display_preheat_start();
		}else if(menuPos == 112){
			display_preheat_set();
		}
	}else if(menuPos >=10 && menuPos < 100){
		uint16_t subMenuPos = menuPos / 10;
		uint16_t mainMenuPos = menuPos % 10;
		if(mainMenuPos == 2){
			display_submenu_1(mainMenuPos, subMenuPos);
		}
	} else if(menuPos < 10){
		display_main_menu(menuPos);
	}
}


// direction 0-left(-)  1-right(+)  2-OK
void click_callback(uint8_t direction){
	if(menuPos >=100 && menuPos < 1000){
		if(direction == 0){
			if(menuPos > 200){
				menuPos -= 100;
			}
		} else if(direction == 1){
				menuPos += 100;
		} else { //OK
			if(menuPos == 122){ //if back
				menuPos = 12;
				oledDisplayCls(0);
				readTemperatureFlag = 0;

			}
		}
	}else if(menuPos >=10 && menuPos < 100){
		if(direction == 0){
			if(menuPos > 20){
				menuPos -= 10;
			}
		} else if(direction == 1){
				menuPos += 10;
		} else { //OK
			if(menuPos == 32){ //if back
				menuPos = menuPos - ((menuPos / 10)*10);
			} else {
				menuPos += 100;
				oledDisplayCls(0);
			}
		}
	} else if(menuPos < 10){
		if(direction == 0){
			if(menuPos > 1){
				menuPos--;
			}
		} else if(direction == 1){
				menuPos++;
		} else { //OK
			menuPos += 10;
			oledDisplayCls(0);
		}
	}
}
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
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  	HAL_GPIO_WritePin(MAX6675_1_CS_GPIO_Port, MAX6675_1_CS_Pin, 1);
  	HAL_GPIO_WritePin(MAX6675_2_CS_GPIO_Port, MAX6675_2_CS_Pin, 1);

  	HAL_TIM_Base_Start_IT(&htim2);

  	oledInit(&hspi1, 1);
  	sprintf((char*)buf,"HOT");
  	oledDispTxt(40, 0, buf, Font_16x26, 1);
  	sprintf((char*)buf,"PLATE");
  	oledDispTxt(20, 25, buf, Font_16x26, 1);
  	sprintf((char*)buf,"ver.1.0");
  	oledDispTxt(74, 54, buf, Font_7x10, 1);
  	oledRefreshActiveFlag = 1;
  	oledRefreshAll(&hspi1);

  	beep(1400, 20);
  	beep_callback(&htim3, TIM_CHANNEL_1);

  	station_init();
	HAL_Delay(200);
	oledDisplayCls(0);

	//PID
	bedTempPIDSetpoint = 0.0;
	//PID(&bedTempPID, &bedTemperature, &bedPower, &bedTempPIDSetpoint, 0.8, 0.7, 0.8, _PID_P_ON_E, _PID_CD_REVERSE);
	//PID_SetMode(&bedTempPID, _PID_MODE_AUTOMATIC);
	//PID_SetSampleTime(&bedTempPID, 1); //1ms refresh time
	//PID_SetOutputLimits(&bedTempPID, 11, 300);
	//---
	//PID_Compute(&PowerPID); //calculate PID - in timer
	//---
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  display_menu();

	  HAL_Delay(200);
	  beep_callback(&htim3, TIM_CHANNEL_1);
	  buttonFlag = 0; //debouncing


	  if(readTemperatureFlag == 1){
		  read_bed_temperature(&bedTemperature);
		  read_pcb_temperature(&pcbTemperature);
	  }
	  MessageLength = sprintf((char*) TxBuffer, "%i -> bed:%.0f / pcb:%.0f\n\r", menuPos, bedTemperature, pcbTemperature);
	  CDC_Transmit_FS(TxBuffer, MessageLength);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 9999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 219;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 700;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 17;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, OLED_CS_Pin|OLED_RES_Pin|OLED_DC_Pin|MAX6675_1_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, MAX6675_2_CS_Pin|FAN1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : AC_ZERO_Pin */
  GPIO_InitStruct.Pin = AC_ZERO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(AC_ZERO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ENC_1_Pin BUTTON_1_Pin */
  GPIO_InitStruct.Pin = ENC_1_Pin|BUTTON_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ENC_2_Pin */
  GPIO_InitStruct.Pin = ENC_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ENC_2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OLED_CS_Pin OLED_RES_Pin OLED_DC_Pin MAX6675_1_CS_Pin */
  GPIO_InitStruct.Pin = OLED_CS_Pin|OLED_RES_Pin|OLED_DC_Pin|MAX6675_1_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : MAX6675_2_CS_Pin FAN1_Pin */
  GPIO_InitStruct.Pin = MAX6675_2_CS_Pin|FAN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
