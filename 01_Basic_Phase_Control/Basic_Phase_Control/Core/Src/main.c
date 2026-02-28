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
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void displayDigit(uint8_t digit) {
    // Önce tüm segmentleri söndür (Temizle)
    HAL_GPIO_WritePin(GPIOA, SEG_A_Pin|SEG_F_Pin|SEG_G_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, SEG_B_Pin|SEG_C_Pin|SEG_D_Pin|SEG_E_Pin, GPIO_PIN_RESET);

    // Rakamlara göre segmentleri yak (Ortak Katot: SET = YANAR)
    switch(digit) {
        case 0: HAL_GPIO_WritePin(GPIOA, SEG_A_Pin|SEG_F_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(GPIOB, SEG_B_Pin|SEG_C_Pin|SEG_D_Pin|SEG_E_Pin, GPIO_PIN_SET); break;
        case 1: HAL_GPIO_WritePin(GPIOB, SEG_B_Pin|SEG_C_Pin, GPIO_PIN_SET); break;
        case 2: HAL_GPIO_WritePin(GPIOA, SEG_A_Pin|SEG_G_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(GPIOB, SEG_B_Pin|SEG_D_Pin|SEG_E_Pin, GPIO_PIN_SET); break;
        case 3: HAL_GPIO_WritePin(GPIOA, SEG_A_Pin|SEG_G_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(GPIOB, SEG_B_Pin|SEG_C_Pin|SEG_D_Pin, GPIO_PIN_SET); break;
        case 4: HAL_GPIO_WritePin(GPIOA, SEG_F_Pin|SEG_G_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(GPIOB, SEG_B_Pin|SEG_C_Pin, GPIO_PIN_SET); break;
        case 5: HAL_GPIO_WritePin(GPIOA, SEG_A_Pin|SEG_F_Pin|SEG_G_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(GPIOB, SEG_C_Pin|SEG_D_Pin, GPIO_PIN_SET); break;
        case 6: HAL_GPIO_WritePin(GPIOA, SEG_A_Pin|SEG_F_Pin|SEG_G_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(GPIOB, SEG_C_Pin|SEG_D_Pin|SEG_E_Pin, GPIO_PIN_SET); break;
        case 7: HAL_GPIO_WritePin(GPIOA, SEG_A_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(GPIOB, SEG_B_Pin|SEG_C_Pin, GPIO_PIN_SET); break;
        case 8: HAL_GPIO_WritePin(GPIOA, SEG_A_Pin|SEG_F_Pin|SEG_G_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(GPIOB, SEG_B_Pin|SEG_C_Pin|SEG_D_Pin|SEG_E_Pin, GPIO_PIN_SET); break;
        case 9: HAL_GPIO_WritePin(GPIOA, SEG_A_Pin|SEG_F_Pin|SEG_G_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(GPIOB, SEG_B_Pin|SEG_C_Pin|SEG_D_Pin, GPIO_PIN_SET); break;
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
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  HAL_GPIO_WritePin(SEG_A_GPIO_Port, SEG_A_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SEG_B_GPIO_Port, SEG_B_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SEG_C_GPIO_Port, SEG_C_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SEG_D_GPIO_Port, SEG_D_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SEG_E_GPIO_Port, SEG_E_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SEG_F_GPIO_Port, SEG_F_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SEG_G_GPIO_Port, SEG_G_Pin, GPIO_PIN_SET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  // 1. ADC Değerini Oku (Potansiyometre)
	    HAL_ADC_Start(&hadc1);
	    HAL_ADC_PollForConversion(&hadc1, 10);
	    uint32_t pot_val = HAL_ADC_GetValue(&hadc1); // 0 - 4095 arası değer

	    // 2. Display Güncelle (0-9 arası kademe)
	    uint8_t digit = pot_val / 410; // 4095/10 yaklasık 410
	    displayDigit(digit);

	    // 3. Tetikleme Gecikmesini Hesapla (ms cinsinden)
	    // 50Hz periyot 20ms. Yarım dalga 10ms.
	    // Gecikme = (Pot / 4095) * 10 ms
	    uint32_t delay_time = (pot_val * 10) / 4095;

	    // 4. Sanal Şebeke ve Tetikleme (Sanal Zero-Crossing)
	    HAL_GPIO_WritePin(GRID_SIG_GPIO_Port, GRID_SIG_Pin, GPIO_PIN_SET); // Pozitif Alternans Başladı

	    HAL_Delay(delay_time); // Tetikleme açısı (alfa) kadar bekle

	    // Tristörü tetikle (Kısa bir darbe gönder)
	    HAL_GPIO_WritePin(PULSE_OUT_GPIO_Port, PULSE_OUT_Pin, GPIO_PIN_SET);
	    HAL_Delay(1); // 1ms tetikleme genişliği
	    HAL_GPIO_WritePin(PULSE_OUT_GPIO_Port, PULSE_OUT_Pin, GPIO_PIN_RESET);

	    HAL_Delay(10 - delay_time - 1); // Pozitif alternansın kalanını tamamla

	    HAL_GPIO_WritePin(GRID_SIG_GPIO_Port, GRID_SIG_Pin, GPIO_PIN_RESET); // Negatif Alternans
	    HAL_Delay(10); // Negatif kısımda tetikleme olmaz

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
