/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 * Name: Khoi Nguyen
 * Date: 03/17/2024
 * School: The Univeristy of Utah
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
#include "stm32f0xx_hal.h"
void _Error_Handler(char* file, int line);

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void) {
    /**
     * Part 1 of the Lab5
     */
    HAL_Init();
    SystemClock_Config();

    // Enable GPIOB and GPIOC in the RCC
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    // Enable I2C2 in the RCC
    RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;

    // Initialize LED pins
    GPIO_InitTypeDef initStr = {GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_6 | GPIO_PIN_7,
                                GPIO_MODE_OUTPUT_PP,
                                GPIO_SPEED_FREQ_LOW,
                                GPIO_NOPULL};
    HAL_GPIO_Init(GPIOC, &initStr);  // Initialize LED pins

    // Setting up PB11 and PB13 to alternate function mode
    GPIOB->MODER &= ~(GPIO_MODER_MODER11_0 | GPIO_MODER_MODER13_0);
    GPIOB->MODER |= (GPIO_MODER_MODER11_1 | GPIO_MODER_MODER13_1);

    // Setting up PB11 and PB13 to open-drain output type
    GPIOB->OTYPER |= GPIO_OTYPER_OT_11;
    GPIOB->OTYPER |= GPIO_OTYPER_OT_13;

    // select I2C2_SDA as its alternate function for PB11
    GPIOB->AFR[1] |= (1 << 12);

    // select I2C2_SCL as its alternate function for PB13
    GPIOB->AFR[1] |= (5 << 20);

    // Set up for PB14 
        // to output mode
    GPIOB->MODER |= GPIO_MODER_MODER14_0;
    GPIOB->MODER &= ~(GPIO_MODER_MODER14_1);
        // push-pull output type
    GPIOB->OTYPER &= ~(GPIO_OTYPER_OT_14);
        // initialize/set the pin high
    GPIOB->ODR |= GPIO_ODR_14;

    // Setting up PC0
        // to output mode
    GPIOC->MODER |= GPIO_MODER_MODER0_0;
    GPIOC->MODER &= ~(GPIO_MODER_MODER0_1);
        // push-pull output type
    GPIOB->OTYPER &= ~(GPIO_OTYPER_OT_0);
        // initialize/set the pin high
    GPIOC->ODR |= GPIO_ODR_0;

    // Leave PB15 on input mode
    GPIOB->MODER &= ~(GPIO_MODER_MODER15_0 | GPIO_MODER_MODER15_1);

    // Enable the I2C2 peripheral in the RCC.
    RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;

    // Set the parameters in the TIMINGR register to use 100 kHz standard-mode I2C
    I2C2->TIMINGR = 0x00201D2B;
    
    // I2C2->TIMINGR |= (1 << 28) | 0x13 | (0xF << 8) | (0x2 << 16) | (0x4 << 20);

    // Enable the I2C peripheral using the PE bit in the CR1 register
    I2C2->CR1 |= I2C_CR1_PE;

    // // Set the transaction parameters in the CR2 register
    //     // Set the L3GD20 slave address = 0x6B
    // I2C2->CR2 |= (0x6B << 1);

    // // Set the number of bytes to transmit = 1
    // I2C2->CR2 |= (1 << 16);


    /* Clear the NBYTES and SADD bit fields
     * The NBYTES field begins at bit 16, the SADD at bit 0
     */
    I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));
    /* Set NBYTES = 1 and SADD = 0x69
     * Can use hex or decimal values directly as bitmasks.
     * Remember that for 7-bit addresses, the lowest SADD bit
     * is not used and the mask must be shifted by one.
     */
    I2C2->CR2 |= (1 << 16) | (0x69 << 1);

    // Set the START bit.
    I2C2->CR2 |= I2C_CR2_START;

    // Set the RD_WRN bit to indicate a write operation.
    I2C2->CR2 &= ~(I2C_CR2_RD_WRN);

    // wait until TXIS or NACKF flags are set
    while (!(I2C2->ISR & I2C_ISR_TXIS))
        ;
    if (I2C2->ISR & I2C_ISR_NACKF)
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
    // write who_am_I reg into I2C transmit register
    I2C2->TXDR |= 0x0F;
    while (!(I2C2->ISR & I2C_ISR_TC))
        ; /* loop waiting for TC */
    // Reload the CR2 register
    // setting SADD & NBYTES
    I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));
    I2C2->CR2 |= (1 << 16) | (0x69 << 1);

    // reset RD_WRN to read
    I2C2->CR2 |= I2C_CR2_RD_WRN;
    // reset start bit
    I2C2->CR2 |= I2C_CR2_START;

    // Wait until either of the TXIS (Transmit Register Empty/Ready) or NACKF (Slave Not-Acknowledge) flags are set.
    while (!(I2C2->ISR & I2C_ISR_RXNE))
        ;
    if (I2C2->ISR & I2C_ISR_NACKF)
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
    while (!(I2C2->ISR & I2C_ISR_TC))
        ; /* loop waiting for TC */

    // Check the contents of the RXDR register to see if it matches 0xD4
    if (I2C2->RXDR == 0xD3)
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
    // stop
    I2C2->CR2 |= I2C_CR2_STOP;
    // End of Part 1 Checkoff

    // Part 2 of the Lab5
    // Enable the X and Y sensing axes in the CTRL_REG1 register.
    I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));

    // Set the sensor into “normal or sleep mode” using the PD bit in the CTRL_REG1 register.
    I2C2->CR2 |= (2 << 16) | (0x69 << 1);

    // All other bits in the CTRL_REG1 register should be set to 0. These place the device in the default low-speed mode.
    I2C2->CR2 |= I2C_CR2_START;

    // Initialize the L3GD20 gyroscope sensor to read the X and Y axes.
    

    // I2C2->CR2 |= (2 << 16) | (0x69 << 1);
    // I2C2->CR2 |= I2C_CR2_START;
    // while (!(I2C2->ISR & I2C_ISR_TXIS))
    //     ;
    // I2C2->TXDR |= 0x20;
    // while (!(I2C2->ISR & I2C_ISR_TC))
    //     ;
    // I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));
    // I2C2->CR2 |= (1 << 16) | (0x69 << 1);
    // I2C2->CR2 |= I2C_CR2_RD_WRN;
    // I2C2->CR2 |= I2C_CR2_START;
    // while (!(I2C2->ISR & I2C_ISR_RXNE))
    //     ;
    // while (!(I2C2->ISR & I2C_ISR_TC))
    //     ;
    // if (I2C2->RXDR == 0x0F)
    //     HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
    // I2C2->CR2 |= I2C_CR2_STOP;


}

/** System Clock Configuration
 */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = 16;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        _Error_Handler(__FILE__, __LINE__);
    }

    /**Initializes the CPU, AHB and APB busses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
        _Error_Handler(__FILE__, __LINE__);
    }

    /**Configure the Systick interrupt time
     */
    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

    /**Configure the Systick
     */
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

    /* SysTick_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
void _Error_Handler(char* file, int line) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    while (1) {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT

/**
 * @brief Reports the name of the source file and the source line number
 * where the assert_param error has occurred.
 * @param file: pointer to the source file name
 * @param line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line) {
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
      ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}

#endif

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
