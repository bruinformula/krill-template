/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "adc.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_VREF            3.3f
#define ADC_MAX             4096.0f

#define OPAMP_GAIN          0.833f
#define SENSOR_OFFSET_V     0.5f
#define SENSOR_SENSITIVITY  0.100f // Change this (based on the datasheet)

#define NUM_SAMPLES         16
#define SAMPLE_DELAY_MS     10
#define CAN_TX_INTERVAL_MS  100
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
int time;
int voltage_mv;
float voltage_sum;
float avg_voltage;
float voltage;
uint32_t raw;
volatile uint16_t AD_RES[2];
volatile uint8_t halfComplete = 0;
volatile uint8_t fullComplete = 0;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
float current_amps = 0.0f;
// uint32_t last_can_tx_time = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Return raw ADC voltage (0-3.3)
//float ReadADC(void) {
//  raw = 0;
//
//  HAL_ADC_Start(&hadc1);
//  if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK) {
//    raw = HAL_ADC_GetValue(&hadc1);
//  }
//
//  HAL_ADC_Stop(&hadc1);
//  return (raw*ADC_VREF) / ADC_MAX;
//}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	fullComplete = 1;
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc) {
	halfComplete = 1;
}

void StartADC_DMA(void) {
	halfComplete = 0;
	fullComplete = 0;
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)AD_RES, 2);
}

float ReadADC(void) {
  uint16_t avg = (AD_RES[0] + AD_RES[1]) / 2;
  return (avg * ADC_VREF) / ADC_MAX;
}

// Undo gain and calculate current from sensitivity
float GetCurrent(float adc_voltage) {
  float sensor_voltage = adc_voltage / OPAMP_GAIN;
  float current = sensor_voltage / SENSOR_SENSITIVITY;
  return current;
}

// Send current over CAN
// void SendCAN(float current) {
//     FDCAN_TxHeaderTypeDef TxHeader = {0};
//     uint8_t TxData[8];

//     TxHeader.Identifier = 0x100;
//     TxHeader.IdType = FDCAN_STANDARD_ID;
//     TxHeader.TxFrameType = FDCAN_DATA_FRAME;
//     TxHeader.DataLength = FDCAN_DLC_BYTES_8;
//     TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
//     TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
//     TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
//     TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
//     TxHeader.MessageMarker = 0;

//     memcpy(&TxData[0], &current, 4);
//     uint32_t timestamp = HAL_GetTick();
//     memcpy(&TxData[4], &timestamp, 4);

//     HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData);
// }
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
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  StartADC_DMA();
  // Setup can filters
  // FDCAN_FilterTypeDef sFilterConfig = {0};
  // sFilterConfig.IdType = FDCAN_STANDARD_ID;
  // sFilterConfig.FilterIndex = 0;
  // sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
  // sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  // sFilterConfig.FilterID1 = 0x000;
  // sFilterConfig.FilterID2 = 0x7FF;

  // if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK) {
  //   Error_Handler();
  // }

  // if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
  //                                   FDCAN_ACCEPT_IN_RX_FIFO0,
  //                                   FDCAN_REJECT,
  //                                   DISABLE,
  //                                   DISABLE) != HAL_OK) {
  //     Error_Handler();
  // }

  // if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
  //     Error_Handler();
  // }

  // char msg[] = "Current Sensor Ready\r\n";
  // HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), 100);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    voltage_sum = 0.0f;
#if 0
    for (int i = 0; i < NUM_SAMPLES; i++) {
      voltage_sum += ReadADC();
      HAL_Delay(SAMPLE_DELAY_MS);
    }
    avg_voltage = voltage_sum / NUM_SAMPLES;

    current_amps = GetCurrent(avg_voltage);
#endif

    if (halfComplete) {
    	halfComplete = 0;
    	StartADC_DMA();
    }

    if (fullComplete) {
    	voltage = ReadADC();
    	fullComplete = 0;
    	StartADC_DMA();
    }

    char uart_buf[100];

    // After your averaging code:
      voltage_mv = (int)(avg_voltage * 1000);
//    sprintf(uart_buf, "Voltage: %d mV\r\n", voltage_mv);
//    // HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, strlen(uart_buf), 100);
//    printf("Voltage: %d mV\r\n", voltage_mv);

    time += 1;
//    HAL_Delay(1);

    // // Step 3: Debug print over UART
    // char debug[64];
    // snprintf(debug, sizeof(debug), "V=%.3f I=%.2fA\r\n", avg_voltage, current_amps);
    // HAL_UART_Transmit(&huart2, (uint8_t *)debug, strlen(debug), 100);

    // // Step 4: Send over CAN every 100ms
    // if ((HAL_GetTick() - last_can_tx_time) >= CAN_TX_INTERVAL_MS) {
    //     SendCAN(current_amps);
    //     last_can_tx_time = HAL_GetTick();
    // }

    

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
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
#ifdef USE_FULL_ASSERT
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
