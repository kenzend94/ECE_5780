/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32f072xb.h"

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();
  // Enable the GPIOC clock in the RCC
  RCC->AHBENR |= RCC_AHBENR_GPIOCEN;

  // Enable the GPIOA clock in the RCC to use the user button B1 (PA0)
  RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

  // green and orange LEDs are on pins PC8 & 9
  // red and blue LEDs are on pins PC6 & 7

  // // clear mode bits for PC8 & 9
  // GPIOC->MODER &= ~(GPIO_MODER_MODER8_Msk | GPIO_MODER_MODER9_Msk);
  // clear mode bits for PC6 & 7
  GPIOC->MODER &= ~(GPIO_MODER_MODER6_Msk | GPIO_MODER_MODER7_Msk);

  // config GPIOC PC8 & PC9 as general-purpose output mode

  // // set mode bits to 01 for PC8 & 9
  // GPIOC->MODER |= (GPIO_MODER_MODER8_0 | GPIO_MODER_MODER9_0);
  // set mode bits to 01 for PC6 & 7
  GPIOC->MODER |= (GPIO_MODER_MODER6_0 | GPIO_MODER_MODER7_0);

  // // config PC8 and PC9 as push-pull output type
  // GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_8 | GPIO_OTYPER_OT_9);
  // config PC6 and PC7 as push-pull output type
  GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_6 | GPIO_OTYPER_OT_7);

  // // config PC8 and PC9 to low speed
  // GPIOC->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEEDR8_Msk | GPIO_OSPEEDR_OSPEEDR9_Msk);
  // config PC6 and PC7 to low speed
  GPIOC->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEEDR6_Msk | GPIO_OSPEEDR_OSPEEDR7_Msk);

  // // config PC8 and PC9 with no pull-up/down resistors
  // GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR8_Msk | GPIO_PUPDR_PUPDR9_Msk);
  // config PC6 and PC7 with no pull-up/down resistors
  GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR6_Msk | GPIO_PUPDR_PUPDR7_Msk);

  // Initialize one pin logic high and the other to low.
  // GPIOC->BSRR = GPIO_BSRR_BS_8; // Set PC8 high
  // GPIOC->BSRR = GPIO_BSRR_BR_9; // Set PC9 low

  GPIOC->BSRR = GPIO_BSRR_BS_6; // Set PC6 high
  GPIOC->BSRR = GPIO_BSRR_BR_7; // Set PC7 low

  // Configure the button pin to input mode with the internal pull-down resistor enabled.
  // Set PA0 to input mode
  GPIOA->MODER &= ~(GPIO_MODER_MODER0_Msk);
  // Set PA0 to low speed
  GPIOA->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEEDR0_Msk);
  // Clear pull-up/pull-down bits for PA0
  GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR0_Msk;
  // Set PA0 to pull-down mode
  GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR0_Msk);

  // Monitor the button pin input state within the endless program loop.
  // Use the IDR register.

  // Initialize the debouncer and LED state
  uint32_t debouncer = 0;
  uint32_t ledState = 0;

  while (1)
  {
    // Shift the debouncer register and read the button state
    debouncer = (debouncer << 1);

    // check if button is set
    int buttonPressed = GPIOA->IDR & 1;

    // Check if the button is pressed
    if (buttonPressed)
    {
      // Set lowest bit of bit-vector
      debouncer |= 0x01;
    }

    if (debouncer == 0x7FFFFFFF)
    {
      if (ledState == 0)
      {
        GPIOC->BSRR = GPIO_BSRR_BR_6; // Set PC6 low
        GPIOC->BSRR = GPIO_BSRR_BS_7; // Set PC7 high
        ledState = 1;
      }
      else
      {
        GPIOC->BSRR = GPIO_BSRR_BS_6; // Set PC6 high
        GPIOC->BSRR = GPIO_BSRR_BR_7; // Set PC7 low
        ledState = 0;
      }
    }

  }
  HAL_Delay(1);

  // Here is the first part 1.5.1: Configuring a GPIO Pin to Output and Blink an LED

  // while (1)
  // {
  //   // Toggle PC6 and PC7 using ODR
  //   GPIOC->ODR ^= (GPIO_ODR_6 | GPIO_ODR_7); // Toggle PC7 and PC6

  //   // Delay for a while to see the toggling effect
  //   HAL_Delay(500); // Delay for 500 milliseconds
  // }
}

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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
