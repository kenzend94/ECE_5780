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
#include "stm32f0xx_hal_gpio_ex.h"


/* Private variables ----------------------------------------------------------*/
int flag;
char color, mode;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
	
/**
 * @brief Read a character to Transmit Data Register
 * @param c character it reads
 */
void transmit_char(char c)
{
  while (!(USART3->ISR & USART_ISR_TXE))
  {
  }
  USART3->TDR = c;
}

/**
 * @brief Read a string to Transmit Data Register
 * @param s a C string
 */
void transmit_string(char *s)
{
  for (int i = 0; s[i] != '\0'; i++)
    transmit_char(s[i]);
}

/**
 * @brief Toggle LEDS from given r,g,b,o characters
 *        4.1 section checkoff
 */
void receive_LED()
{
  if (USART3->ISR & USART_CR1_RXNEIE)
  {
    color = USART3->RDR;
    switch (color)
    {
    case 'r':
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
      break;
    case 'b':
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);
      break;
    case 'o':
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);
      break;
    case 'g':
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9);
      break;
    }
    if (color != 'r' && color != 'b' && color != 'g' && color != 'o')
      transmit_string("Error");
  }
}

/**
 * @brief Toggle LEDS from given r,g,b,o characters with extra modes
 *        4.2 section checkoff
 */
void USART3_4_IRQHandler()
{
  if (!flag)
  {
    color = USART3->RDR;
    flag = 1;
  }
  else
  {
    switch (color)
    {
    case 'r':
      mode = USART3->RDR;
      if (mode == '0')
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
      else if (mode == '1')
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
      else if (mode == '2')
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
      break;
    case 'b':
      mode = USART3->RDR;
      if (mode == '0')
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
      else if (mode == '1')
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
      else if (mode == '2')
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);
      break;
    case 'o':
      mode = USART3->RDR;
      if (mode == '0')
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
      else if (mode == '1')
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
      else if (mode == '2')
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);
      break;
    case 'g':
      mode = USART3->RDR;
      if (mode == '0')
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
      else if (mode == '1')
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
      else if (mode == '2')
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9);
      break;
    }
    flag = 0;
  }
  if (color != 'r' && color != 'b' && color != 'g' && color != 'o')
    transmit_string("Error");
}

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
  // Initialize Clock
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  RCC->APB1ENR = RCC_APB1ENR_USART3EN;

  // Initialize LED pins
  GPIO_InitTypeDef initStr = {GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_6 | GPIO_PIN_7,
                              GPIO_MODE_OUTPUT_PP,
                              GPIO_SPEED_FREQ_LOW,
                              GPIO_NOPULL};
  HAL_GPIO_Init(GPIOC, &initStr); // Initialize LED pins

  /* 4.1 section */
  GPIO_InitTypeDef initStr2 = {GPIO_PIN_10 | GPIO_PIN_11,
                               GPIO_MODE_AF_PP,
                               GPIO_SPEED_FREQ_LOW,
                               GPIO_NOPULL};
  HAL_GPIO_Init(GPIOB, &initStr2);

  // Set alternate function
  GPIOB->AFR[1] &= ~(GPIO_AFRH_AFSEL10_Msk | GPIO_AFRH_AFSEL11_Msk);
  GPIOB->AFR[1] |= (GPIO_AF4_USART3 << GPIO_AFRH_AFSEL10_Pos) | 
  (GPIO_AF4_USART3 << GPIO_AFRH_AFSEL11_Pos);

  // GPIOB->AFR[1] |= (GPIO_AF4_USART3 << GPIO_AFRH_AFSEL10_Pos) | 
  //                   (GPIO_AF4_USART3 << GPIO_AFRH_AFSEL11_Pos);
  // set alternate function mode for pb10 and pb11
  GPIOB->MODER |= GPIO_MODER_MODER10_1 | GPIO_MODER_MODER11_1;

  // Set up USART3
  USART3->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_UE | USART_CR1_RXNEIE;

  // Set baud rate
  USART3->BRR = HAL_RCC_GetHCLKFreq() / 115200;

  // Enable USART3_4_IRQn
  // NVIC_EnableIRQ(USART3_4_IRQn);

  // Enable global interrupts
  while (1)
  {
    HAL_Delay(5000);
    // transmit_string("CMD?");
    // receive_LED();
    // transmit_string("hello world\n");
    transmit_char('r');
  }
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
