/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dac.h"
#include "i2c.h"
#include "sdio.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "vcp_time_segmentation.h"
#include "itb.h"
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

/* USER CODE BEGIN PV */
type_VCP_UART vcp;
type_ITB_DEVICE itb;
uint16_t adc_num=0;
uint8_t tx_data[256], tx_data_len=0; //массив для формирования данных для отправки через VCP
uint8_t timer_flag = 0x00;
uint16_t i2c_register = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  MX_DAC_Init();
  MX_I2C2_Init();
  MX_SDIO_SD_Init();
  MX_USB_DEVICE_Init();
  MX_TIM5_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */
  itb_device_init(&itb);
  HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 0);
	HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
	HAL_DAC_Start(&hdac, DAC_CHANNEL_2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		// Обработка событий один раз в 10 мс
		if (timer_flag & (1 << 3)){
			timer_flag = 0;
			Current_Calc_Step_10ms(&itb);
			HAL_ADC_Start_IT(&hadc2);
		}
		// обработка команд
		if (vcp_uart_read(&vcp)){
			if (vcp.rx_buff[0] == DEV_ID){
				if (vcp.rx_buff[4] == 0x00){ //зеркало для ответа
					tx_data_len = vcp.rx_buff[5];
					memcpy(tx_data, &vcp.rx_buff[6], vcp.tx_size);
				}
				else if (vcp.rx_buff[4] == 0x01){ //чтение 16-ти каналов АЦП
					tx_data_len = 16*2;
					memcpy(tx_data, itb.adc_data, tx_data_len);
				}
				else if (vcp.rx_buff[4] == 0x02){ //запуск измерения в указаноом режиме
					tx_data_len = 0;
					itb.control.mode = vcp.rx_buff[6];
				}
				else if (vcp.rx_buff[4] == 0x03){ //чтение результатов измерения токов для 8-ми каналов
					get_current_measure_data(&itb, tx_data, &tx_data_len);
				}
				else if (vcp.rx_buff[4] == 0x04){ //установка напряжения каналов DAC
					uint16_t dac_ch1 = 0, dac_ch2 = 0;  // DAC_out[mV] = Vref*(DOR/4098), Vref=2048mV
					dac_ch1 = ((vcp.rx_buff[6] << 8) + (vcp.rx_buff[7] << 0)) << 1;
					dac_ch2 = ((vcp.rx_buff[8] << 8) + (vcp.rx_buff[9] << 0)) << 1;
					HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dac_ch1);
					HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, dac_ch2);
				}
				else if (vcp.rx_buff[4] == 0x05){ // установка параметров измерения: длительность и мертвое время
					set_measure_parameters(&itb, __REV(*(uint32_t*)&vcp.rx_buff[6]), __REV(*(uint32_t*)&vcp.rx_buff[10]));
					get_current_measure_parameters(&itb, tx_data, &tx_data_len);
				}
				else if (vcp.rx_buff[4] == 0x06){ // отправка параметров измерения: длительность и мертвое время
					get_current_measure_parameters(&itb, tx_data, &tx_data_len);
				}
				else if (vcp.rx_buff[4] == 0x07){ // debug_mode
					set_ch_gpio_settings(itb.channel, vcp.rx_buff[6], vcp.rx_buff[7], vcp.rx_buff[8]);
					itb.control.mode |= 0x80;
					tx_data_len = 0;
				}
				vcp.tx_size = com_ans_form(vcp.rx_buff[1], DEV_ID, &vcp.tx_seq_num, vcp.rx_buff[4], tx_data_len, tx_data, vcp.tx_buff);
				vcp_uart_write(&vcp, vcp.tx_buff, vcp.tx_size);
			}
		}
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enables the Clock Security System 
  */
  HAL_RCC_EnableCSS();
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim2) {
		itb.global_time_s += 1;
	}
	if (htim == &htim5) {
	}
	if (htim == &htim3) {
		timer_flag = 0x01 << 3;
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if (hadc == &hadc2){
		itb.adc_data[adc_num] = HAL_ADC_GetValue(hadc);
		adc_num++;
		if (adc_num >= 4){
			adc_num = 0;
		}
		else{
			HAL_ADC_Start_IT(&hadc2);
		}
	}
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef* hadc)
{
	if (hadc == &hadc2){
	}
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef* hi2c)
{
	if (hi2c == &hi2c2){
		
	}
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef* hi2c)
{
	if (hi2c == &hi2c2){
		
	}
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
