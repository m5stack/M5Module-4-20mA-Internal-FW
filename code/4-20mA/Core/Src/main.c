/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "i2c_ex.h"
#include "flash.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CAL_TIME 10
#define I2C_ADDRESS 0x55
#define FLASH_DATA_SIZE 24
#define FIRMWARE_VERSION 1
#define APPLICATION_ADDRESS     ((uint32_t)0x08001800) 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */
union
{
  uint8_t i_array[4];
  uint32_t i_32;
  float    f;
} cal_store_ch1;

union
{
  uint8_t i_array[4];
  uint32_t i_32;
  float    f;
} cal_store_ch2;

union
{
  uint8_t i_array[4];
  uint32_t i_32;
  float    f;
} cal_store_ch3;

union
{
  uint8_t i_array[4];
  uint32_t i_32;
  float    f;
} cal_store_ch4;

volatile uint8_t fm_version = FIRMWARE_VERSION;

volatile uint8_t flag_jump_bootloader = 0;
volatile uint32_t jump_bootloader_timeout = 0;

__IO uint32_t uiAdcValueBuf[800];
__IO uint16_t usAdcValue16[4];
__IO uint8_t usAdcValue8[4];
__IO float usVoltageValue[4];
__IO float usCurrentValue[4];
__IO float usCalValue[4];
uint8_t flash_data[FLASH_DATA_SIZE] = {0};
uint8_t i2c_address[1] = {0};
__IO uint32_t ref_current;
__IO float ref_current_float[4];
__IO uint8_t cal_index[4];
__IO uint8_t cal_counter = 0;
__IO uint8_t cal_flag = 0;
uint32_t current_uint32[4] = {0};
uint32_t last_current_uint32[4] = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void IAP_Set()
{
	uint8_t i;
 
	uint32_t *pVecTab=(uint32_t *)(0x20000000);

	for(i = 0; i < 48; i++)
	{
		*(pVecTab++) = *(__IO uint32_t*)(APPLICATION_ADDRESS + (i<<2));
	}
  /* Enable the SYSCFG peripheral clock*/
#if 1 //STM32
  __HAL_RCC_SYSCFG_CLK_ENABLE();

  __HAL_SYSCFG_REMAPMEMORY_SRAM();
#else //AMP32
    RCM_EnableAPB2PeriphClock(RCM_APB2_PERIPH_SYSCFG);
    /* Remap SRAM at 0x00000000 */
    SYSCFG->CFG1_B.MMSEL = SYSCFG_MemoryRemap_SRAM;
#endif
}

long map(long x, long in_min, long in_max, long out_min, long out_max) {
  long result;

  result = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  if (result < out_min)
    result = out_min;
  else if (result > out_max)
    result = out_max;
  
  return result;
}

void i2c_port_set_to_input(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = (GPIO_PIN_8 | GPIO_PIN_9);
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void user_i2c_init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00602173;
  hi2c1.Init.OwnAddress1 = i2c_address[0]<<1;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

void init_cal_data(void)
{
  cal_store_ch1.f = 148.443;
  cal_store_ch2.f = 148.443;
  cal_store_ch3.f = 148.443;
  cal_store_ch4.f = 148.443;
}

void init_flash_data(void) 
{   
  if (!(readPackedMessageFromFlash(flash_data, FLASH_DATA_SIZE))) {
    flash_data[0] = I2C_ADDRESS;
    flash_data[1] = 0;
    flash_data[2] = 0;
    flash_data[3] = 0;
    flash_data[4] = 0;
    flash_data[5] = 0;
    flash_data[6] = 0;
    flash_data[7] = 0;
    flash_data[8] = cal_store_ch1.i_array[0];
    flash_data[9] = cal_store_ch1.i_array[1];
    flash_data[10] = cal_store_ch1.i_array[2];
    flash_data[11] = cal_store_ch1.i_array[3];
    flash_data[12] = cal_store_ch2.i_array[0];
    flash_data[13] = cal_store_ch2.i_array[1];
    flash_data[14] = cal_store_ch2.i_array[2];
    flash_data[15] = cal_store_ch2.i_array[3];
    flash_data[16] = cal_store_ch3.i_array[0];
    flash_data[17] = cal_store_ch3.i_array[1];
    flash_data[18] = cal_store_ch3.i_array[2];
    flash_data[19] = cal_store_ch3.i_array[3];
    flash_data[20] = cal_store_ch4.i_array[0];
    flash_data[21] = cal_store_ch4.i_array[1];
    flash_data[22] = cal_store_ch4.i_array[2];
    flash_data[23] = cal_store_ch4.i_array[3];
    writeMessageToFlash(flash_data , FLASH_DATA_SIZE);
  } else {
    i2c_address[0] = flash_data[0];
    cal_store_ch1.i_array[0] = flash_data[8];
    cal_store_ch1.i_array[1] = flash_data[9];
    cal_store_ch1.i_array[2] = flash_data[10];
    cal_store_ch1.i_array[3] = flash_data[11];
    cal_store_ch2.i_array[0] = flash_data[12];
    cal_store_ch2.i_array[1] = flash_data[13];
    cal_store_ch2.i_array[2] = flash_data[14];
    cal_store_ch2.i_array[3] = flash_data[15];
    cal_store_ch3.i_array[0] = flash_data[16];
    cal_store_ch3.i_array[1] = flash_data[17];
    cal_store_ch3.i_array[2] = flash_data[18];
    cal_store_ch3.i_array[3] = flash_data[19];
    cal_store_ch4.i_array[0] = flash_data[20];
    cal_store_ch4.i_array[1] = flash_data[21];
    cal_store_ch4.i_array[2] = flash_data[22];
    cal_store_ch4.i_array[3] = flash_data[23];

    usCalValue[0] = cal_store_ch1.f;
    usCalValue[1] = cal_store_ch2.f;
    usCalValue[2] = cal_store_ch3.f;
    usCalValue[3] = cal_store_ch4.f;
  }
}

void cal_data_write_back(void)
{
  if (readPackedMessageFromFlash(flash_data, FLASH_DATA_SIZE)) {
    flash_data[8] = cal_store_ch1.i_array[0];
    flash_data[9] = cal_store_ch1.i_array[1];
    flash_data[10] = cal_store_ch1.i_array[2];
    flash_data[11] = cal_store_ch1.i_array[3];
    flash_data[12] = cal_store_ch2.i_array[0];
    flash_data[13] = cal_store_ch2.i_array[1];
    flash_data[14] = cal_store_ch2.i_array[2];
    flash_data[15] = cal_store_ch2.i_array[3];
    flash_data[16] = cal_store_ch3.i_array[0];
    flash_data[17] = cal_store_ch3.i_array[1];
    flash_data[18] = cal_store_ch3.i_array[2];
    flash_data[19] = cal_store_ch3.i_array[3];
    flash_data[20] = cal_store_ch4.i_array[0];
    flash_data[21] = cal_store_ch4.i_array[1];
    flash_data[22] = cal_store_ch4.i_array[2];
    flash_data[23] = cal_store_ch4.i_array[3];
    writeMessageToFlash(flash_data , FLASH_DATA_SIZE);
  }     
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
  uint64_t adcTotal[4]={0};
  
	HAL_ADC_Stop_DMA(hadc);
	for (uint16_t i = 0; i < 800; i++) {
    adcTotal[i%4] += uiAdcValueBuf[i];
  }
	for (uint8_t i = 0; i < 4; i++) {
    usAdcValue16[i] = (adcTotal[i%4] / 200);
  }

	for (uint8_t i = 0; i < 4; i++) {
    usAdcValue8[i] = map(usAdcValue16[i],0,65535,0,255);
  }

  if (cal_index[0]) {
    cal_index[0] = 0;
    usVoltageValue[0] = ((float)usAdcValue16[0]) / 65535.0 * 3.3;
    usCalValue[0] = usVoltageValue[0] / ref_current_float[0];  
    cal_store_ch1.f = usCalValue[0];  
    cal_data_write_back();
  }
  if (cal_index[1]) {
    cal_index[1] = 0;
    usVoltageValue[1] = ((float)usAdcValue16[1]) / 65535.0 * 3.3;
    usCalValue[1] = usVoltageValue[1] / ref_current_float[1];    
    cal_store_ch2.f = usCalValue[1];
    cal_data_write_back();
  }
  if (cal_index[2]) {
    cal_index[2] = 0;
    usVoltageValue[2] = ((float)usAdcValue16[2]) / 65535.0 * 3.3;
    usCalValue[2] = usVoltageValue[2] / ref_current_float[2];  
    cal_store_ch3.f = usCalValue[2];  
    cal_data_write_back();
  }
  if (cal_index[3]) {
    cal_index[3] = 0;
    usVoltageValue[3] = ((float)usAdcValue16[3]) / 65535.0 * 3.3;
    usCalValue[3] = usVoltageValue[3] / ref_current_float[3];  
    cal_store_ch4.f = usCalValue[3];  
    cal_data_write_back();
  }

  if (!cal_index[0] && !cal_index[1] && !cal_index[2] && !cal_index[3]) {
    for (uint8_t i = 0; i < 4; i++) {
      usVoltageValue[i] = ((float)usAdcValue16[i]) / 65535.0 * 3.3;
      usCurrentValue[i] = usVoltageValue[i] / usCalValue[i];
      current_uint32[i] = (uint32_t)(usCurrentValue[i] * 100000);
    } 
  }

	HAL_ADC_Start_DMA(hadc, (uint32_t *)uiAdcValueBuf, 800);
}

void i2c1_receive_callback(uint8_t *rx_data, uint16_t len) 
{
  uint8_t buf[4];
  uint8_t rx_buf[16];
  uint8_t rx_mark[16] = {0};  

  if(len == 1 && ( rx_data[0] <= 0x07)) 
  {
    if(rx_data[0] == 0x00 || rx_data[0] == 0x01)
    {
      buf[0] = usAdcValue16[0] & 0xff;
      buf[1] = (usAdcValue16[0] >> 8) & 0xff;
      i2c1_set_send_data(buf, 2);
    }  
    else if(rx_data[0] == 0x02 || rx_data[0] == 0x03) 
    {
      buf[0] = usAdcValue16[1] & 0xff;
      buf[1] = (usAdcValue16[1] >> 8) & 0xff;
      i2c1_set_send_data(buf, 2);
    }      
    else if(rx_data[0] == 0x04 || rx_data[0] == 0x05)
    {
      buf[0] = usAdcValue16[2] & 0xff;
      buf[1] = (usAdcValue16[2] >> 8) & 0xff;
      i2c1_set_send_data(buf, 2);
    }      
    else if(rx_data[0] == 0x06 || rx_data[0] == 0x07)
    {
      buf[0] = usAdcValue16[3] & 0xff;
      buf[1] = (usAdcValue16[3] >> 8) & 0xff;
      i2c1_set_send_data(buf, 2);
    }      
	} 
  else if(len == 1 && ((rx_data[0] >= 0x10) && (rx_data[0] <= 0x13))) 
  {
    if(rx_data[0] == 0x00)
    {
      buf[0] = usAdcValue8[0] & 0xff;
      i2c1_set_send_data(buf, 1);
    }  
    else if(rx_data[0] == 0x01) 
    {
      buf[0] = usAdcValue8[1] & 0xff;
      i2c1_set_send_data(buf, 1);
    }      
    else if(rx_data[0] == 0x02)
    {
      buf[0] = usAdcValue8[2] & 0xff;
      i2c1_set_send_data(buf, 1);
    }      
    else if(rx_data[0] == 0x03)
    {
      buf[0] = usAdcValue8[3] & 0xff;
      i2c1_set_send_data(buf, 1);
    } 
	} 
  else if(len == 1 && ((rx_data[0] >= 0x20) && (rx_data[0] <= 0x27))) 
  {
    if(rx_data[0] == 0x20 || rx_data[0] == 0x21)
    {
      buf[0] = (current_uint32[0] & 0xff);
      buf[1] = ((current_uint32[0] >> 8) & 0xff);
      i2c1_set_send_data(buf, 2);
    }  
    else if(rx_data[0] == 0x22 || rx_data[0] == 0x23) 
    {
      buf[0] = (current_uint32[1] & 0xff);
      buf[1] = ((current_uint32[1] >> 8) & 0xff);
      i2c1_set_send_data(buf, 2);
    }      
    else if(rx_data[0] == 0x24 || rx_data[0] == 0x25)
    {
      buf[0] = (current_uint32[2] & 0xff);
      buf[1] = ((current_uint32[2] >> 8) & 0xff);
      i2c1_set_send_data(buf, 2);
    }      
    else if(rx_data[0] == 0x26 || rx_data[0] == 0x27)
    {
      buf[0] = (current_uint32[3] & 0xff);
      buf[1] = ((current_uint32[3] >> 8) & 0xff);
      i2c1_set_send_data(buf, 2);
    }
	} 
  else if(len > 1 && ((rx_data[0] >= 0x30) && (rx_data[0] <= 0x37))) 
  {
    for(int i = 0; i < len - 1; i++) {
      rx_buf[rx_data[0]-0x30+i] = rx_data[1+i];
      rx_mark[rx_data[0]-0x30+i] = 1;     
    }    
    if (rx_mark[0] && rx_mark[1]) {
      ref_current = (rx_buf[0] | (rx_buf[1] << 8));
      ref_current_float[0] = ((float)ref_current / 100000.0 );
      cal_counter = CAL_TIME;
      cal_index[0] = 1;
    }
    if (rx_mark[2] && rx_mark[3]) {
      ref_current = (rx_buf[2] | (rx_buf[3] << 8));
      ref_current_float[1] = ((float)ref_current / 100000.0 );
      cal_counter = CAL_TIME;
      cal_index[1] = 1;
    }
    if (rx_mark[4] && rx_mark[5]) {
      ref_current = (rx_buf[4] | (rx_buf[5] << 8));
      ref_current_float[2] = ((float)ref_current / 100000.0 );
      cal_counter = CAL_TIME;
      cal_index[2] = 1;
    }
    if (rx_mark[6] && rx_mark[7]) {
      ref_current = (rx_buf[6] | (rx_buf[7] << 8));
      ref_current_float[3] = ((float)ref_current / 100000.0 );
      cal_counter = CAL_TIME;
      cal_index[3] = 1;
    }
	}
  else if (len == 1 && (rx_data[0] == 0xFE))
  {
    i2c1_set_send_data((uint8_t *)&fm_version, 1);
  }
  else if (len > 1 && (rx_data[0] == 0xFF))
  {
    if (len == 2) {
      if (rx_data[1] < 128) {
        if (readPackedMessageFromFlash(flash_data, FLASH_DATA_SIZE)) {
          i2c_address[0] = rx_data[1];
          flash_data[0] = i2c_address[0];
          flash_data[1] = 0;
          writeMessageToFlash(flash_data , FLASH_DATA_SIZE);
          user_i2c_init();
        }        
      }
    }    
  }
  else if (len == 1 && (rx_data[0] == 0xFF))
  {
    i2c1_set_send_data(i2c_address, 1);    
  }  
  else if (len > 1 && (rx_data[0] == 0xFD))
  {
    if (rx_data[1] == 1) {
      flag_jump_bootloader = 1;
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
  IAP_Set();
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
  MX_ADC1_Init();
  // MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  init_cal_data();
  init_flash_data();
  user_i2c_init();
  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)uiAdcValueBuf, 800);
  HAL_I2C_EnableListen_IT(&hi2c1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (flag_jump_bootloader) {
      HAL_I2C_DeInit(&hi2c1);  
      i2c_port_set_to_input();
      while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) || HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9))
      {
        jump_bootloader_timeout++;
        if (jump_bootloader_timeout >= 60000) {
          flag_jump_bootloader = 0;
          break;
        }
      }
      if (jump_bootloader_timeout < 60000) {
        HAL_NVIC_SystemReset();
      } else {
        user_i2c_init();
        jump_bootloader_timeout = 0;
      }
    }   
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_39CYCLES_5;
  hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_39CYCLES_5;
  hadc1.Init.OversamplingMode = ENABLE;
  hadc1.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_256;
  hadc1.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_4;
  hadc1.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00602173;
  hi2c1.Init.OwnAddress1 = 0x55<<1;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

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
