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

uint8_t system_armed = 0;       // 0: Kilitli (L), 1: Aktif (0-9)
uint32_t last_button_press = 0; // Buton arkını (debounce) önlemek için
uint32_t adc_val = 0;           // Potansiyometre değeri

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void displayDigit(uint8_t digit) {
    // Önce tüm segmentleri söndür (Temizlik)
    HAL_GPIO_WritePin(GPIOA, SEG_A_Pin|SEG_F_Pin|SEG_G_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, SEG_B_Pin|SEG_C_Pin|SEG_D_Pin|SEG_E_Pin, GPIO_PIN_RESET);

    if (digit == 10) { // "L" Harfi (Locked)
        HAL_GPIO_WritePin(GPIOA, SEG_F_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, SEG_E_Pin|SEG_D_Pin, GPIO_PIN_SET);
        return;
    }

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

  HAL_ADC_Start(&hadc1); // ADC sürekli okuma modunda başlasın
  displayDigit(10);      // Başlangıçta "L" göster

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  // --- 1. BUTON VE EMNİYET KONTROLÜ ---
	    // Mavi butona basıldığında (Pull-up olduğu için RESET durumunda basılmış sayılır)
	    if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET)
	    {
	        if(HAL_GetTick() - last_button_press > 250)
	        { // 250ms Debounce
	            system_armed = !system_armed; // Durumu tersine çevir
	            last_button_press = HAL_GetTick();
	            HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin); // Durum değiştiğini kart üzerindeki LED'den de gör
	        }
	    }

	    // --- 2. ÇALIŞMA MANTIĞI ---
	    if(!system_armed)
	    {
	        displayDigit(10); // Sistem kilitli: "L" göster
	        HAL_GPIO_WritePin(GRID_SIG_GPIO_Port, GRID_SIG_Pin, GPIO_PIN_RESET);
	        HAL_GPIO_WritePin(PULSE_OUT_GPIO_Port, PULSE_OUT_Pin, GPIO_PIN_RESET);
	    }
	    else
	    {
	        // Potansiyometre değerini al (10uF kondansatör sayesinde artık daha stabil!)
	        HAL_ADC_PollForConversion(&hadc1, 10);
	        adc_val = HAL_ADC_GetValue(&hadc1);

	        // Display'de 0-9 arası rakamı göster
	        displayDigit(adc_val / 410);

	        // Tetikleme Gecikmesini Hesapla (0 - 10ms arası)
	        uint32_t delay_ms = (adc_val * 10) / 4095;

	        // SANAL ŞEBEKE VE TETİKLEME (Lojik Analizörde İzle)
	        HAL_GPIO_WritePin(GRID_SIG_GPIO_Port, GRID_SIG_Pin, GPIO_PIN_SET); // Pozitif Alternans Başladı
	        HAL_Delay(delay_ms); // Faz açısı gecikmesi

	        // Tristörü ve Mavi LED'i Tetikle
	        HAL_GPIO_WritePin(PULSE_OUT_GPIO_Port, PULSE_OUT_Pin, GPIO_PIN_SET);
	        HAL_Delay(1); // 1ms darbe genişliği
	        HAL_GPIO_WritePin(PULSE_OUT_GPIO_Port, PULSE_OUT_Pin, GPIO_PIN_RESET);

	        HAL_Delay(10 - delay_ms - 1); // 10ms'lik süreyi tamamla
	        HAL_GPIO_WritePin(GRID_SIG_GPIO_Port, GRID_SIG_Pin, GPIO_PIN_RESET); // Negatif Alternans Başladı
	        HAL_Delay(10); // Negatif alternansta tetikleme yapılmaz
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
