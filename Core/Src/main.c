/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#define dig_0_set()     HAL_GPIO_WritePin(GPIOF, dig_0_Pin, GPIO_PIN_SET)
#define dig_0_reset()   HAL_GPIO_WritePin(GPIOF, dig_0_Pin, GPIO_PIN_RESET)
#define dig_1_set()     HAL_GPIO_WritePin(GPIOF, dig_1_Pin, GPIO_PIN_SET)
#define dig_1_reset()   HAL_GPIO_WritePin(GPIOF, dig_1_Pin, GPIO_PIN_RESET)
#define dig_2_set()     HAL_GPIO_WritePin(GPIOF, dig_2_Pin, GPIO_PIN_SET)
#define dig_2_reset()   HAL_GPIO_WritePin(GPIOF, dig_2_Pin, GPIO_PIN_RESET)
#define dig_3_set()     HAL_GPIO_WritePin(GPIOF, dig_3_Pin, GPIO_PIN_SET)
#define dig_3_reset()   HAL_GPIO_WritePin(GPIOF, dig_3_Pin, GPIO_PIN_RESET)

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim10;

/* USER CODE BEGIN PV */
uint8_t tablica[] = {0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F};
//uint8_t tablica[] = {0x79, 0x31, 0x3F, 0x73, 0x38, 0x3F, 0x76, 0x07, 0x7F, 0x00};
uint32_t counter = 10000;

uint32_t tempCode;
uint8_t bitIndex;
uint8_t cmd;
uint8_t cmdli;
uint32_t code;
uint8_t counter_for_led = 200;
volatile uint8_t tmp_for_right = 0;
volatile uint8_t tmp_for_left = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM1_Init(void);

/* USER CODE BEGIN PFP */

void set_data(uint32_t);
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
  MX_TIM10_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
      HAL_TIM_Base_Start(&htim1);
  __HAL_TIM_SET_COUNTER(&htim1, 0);

    
    uint8_t but1_short_state = 0;
    uint8_t but1_long_state = 0;
    uint32_t but1_time_key1 = 0;
    
    uint8_t but2_short_state = 0;
    uint8_t but2_long_state = 0;
    uint32_t but2_time_key1 = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {   
    set_data(counter);
    uint32_t but1_ms = HAL_GetTick();
    uint8_t but1_key1_state = !HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_8);
    if (but1_key1_state == 0 && !but1_short_state && (but1_ms - but1_time_key1) > 50)  //right data
    {
        but1_short_state = 1;
        but1_long_state = 0;
        but1_time_key1 = but1_ms;
    }    
    else if(but1_key1_state == 0 && !but1_long_state && (but1_ms - but1_time_key1) > 2000) 
    {
        but1_long_state = 1;
        if (counter == 10000)
        {
            counter = 0;
            HAL_GPIO_WritePin(GPIOF, led_right_Pin, GPIO_PIN_SET);              //enable RIGHT BIG RED LED
            HAL_GPIO_WritePin(GPIOF, led_left_Pin, GPIO_PIN_RESET);
            counter_for_led = 2;
         }
         if (counter >0)  //move count back
         {
            counter--;
            counter_for_led -=1;
         }
     }   
     else if(but1_key1_state == 1 && but1_short_state && (but1_ms - but1_time_key1) > 50) 
     {
        but1_short_state = 0;
        but1_time_key1 = but1_ms;
        if(!but1_long_state)
        {    
            counter++;
            counter_for_led ++;
            HAL_TIM_PWM_Stop(&htim10, TIM_CHANNEL_1);   
            HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
        }
      }                
    uint32_t but2_ms = HAL_GetTick();
    uint8_t but2_key1_state = !HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_9);
    
    if(but2_key1_state == 0 && !but2_short_state && (but2_ms - but2_time_key1) > 50)  //left data
    {
        but2_short_state = 1;
        but2_long_state = 0;
        but2_time_key1 = but2_ms;
    }   
    else if(but2_key1_state == 0 && !but2_long_state && (but2_ms - but2_time_key1) > 2000) 
    {
        but2_long_state = 1;
        if (counter == 10000)
        {
          counter = 0;
            HAL_GPIO_WritePin(GPIOF, led_right_Pin, GPIO_PIN_RESET);  
            HAL_GPIO_WritePin(GPIOF, led_left_Pin, GPIO_PIN_SET);   //enable LEFT BIG RED LED
            counter_for_led = 0;
        }
        if (counter >0) //move count back
        {
            counter-=100;
             counter_for_led -=1;
        }
    }    
    else if(but2_key1_state == 1 && but2_short_state && (but2_ms - but2_time_key1) > 50) 
    {
        but2_short_state = 0;
        but2_time_key1 = but2_ms;

        if(!but2_long_state)
        {   if (counter == 10000)
          counter = 0;
            counter+=100;
            counter_for_led ++;
            HAL_TIM_PWM_Stop(&htim10, TIM_CHANNEL_1);   
            HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);

        }
    }
    if (counter_for_led < 199)
       {
            if (counter_for_led %4 ==0 || counter_for_led %4 == 1)
            {
                HAL_GPIO_WritePin(GPIOF, led_left_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(GPIOF, led_right_Pin, GPIO_PIN_RESET);
            }   
            else 
            {
                HAL_GPIO_WritePin(GPIOF, led_left_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(GPIOF, led_right_Pin, GPIO_PIN_SET);
            }
        }
   
        /*
    for (volatile uint32_t i =0; i< 300; i++)
    {
      set_data(123);
    }
    for (volatile uint32_t k =0; k< 300; k++)
    {
      set_data(9456);
    }
    */
    
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 15;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 15999;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 199;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim10, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = 99;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim10, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */
  HAL_TIM_MspPostInit(&htim10);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, dig_D_Pin|dig_C_Pin|dig_B_Pin|dig_A_Pin
                          |dig_E_Pin|dig_F_Pin|dig_G_Pin|dig_DP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, dig_0_Pin|dig_1_Pin|dig_2_Pin|dig_3_Pin
                          |led_right_Pin|led_left_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : dig_D_Pin dig_C_Pin dig_B_Pin dig_A_Pin
                           dig_E_Pin dig_F_Pin dig_G_Pin dig_DP_Pin */
  GPIO_InitStruct.Pin = dig_D_Pin|dig_C_Pin|dig_B_Pin|dig_A_Pin
                          |dig_E_Pin|dig_F_Pin|dig_G_Pin|dig_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : dig_0_Pin dig_1_Pin dig_2_Pin dig_3_Pin
                           led_right_Pin led_left_Pin */
  GPIO_InitStruct.Pin = dig_0_Pin|dig_1_Pin|dig_2_Pin|dig_3_Pin
                          |led_right_Pin|led_left_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PE9 PE8 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */
void set_data(uint32_t number)
{
  dig_0_reset();
  dig_1_reset();
  dig_2_reset();
  dig_3_reset();
  
  uint8_t first = number%10;
  number = number/10;
  uint8_t second = number%10;
  number = number/10;
  uint8_t third = number%10;
  number = number/10;
  uint8_t four = number%10;
  
  dig_3_set();
  GPIOE->ODR |=  tablica[first];
  HAL_Delay(1);
  GPIOE->ODR &= ~tablica[first];
  dig_3_reset();
  
  dig_2_set();
  GPIOE->ODR |=  tablica[second];
  HAL_Delay(1);
  GPIOE->ODR &= ~tablica[second];
  dig_2_reset();
  
  dig_1_set();
  GPIOE->ODR |=  tablica[third];
  HAL_Delay(1);
  GPIOE->ODR &= ~tablica[third];
  dig_1_reset();
  
  dig_0_set();
  GPIOE->ODR |=  tablica[four];
  HAL_Delay(1);
  GPIOE->ODR &= ~tablica[four];
  dig_0_reset();
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == GPIO_PIN_6)
  {
    if (__HAL_TIM_GET_COUNTER(&htim1) > 8000)
    {
      tempCode = 0;
      bitIndex = 0;
    }
    else if (__HAL_TIM_GET_COUNTER(&htim1) > 1700)
    {
      tempCode |= (1UL << (31-bitIndex));   // write 1
      bitIndex++;
    }
    else if (__HAL_TIM_GET_COUNTER(&htim1) > 1000)
    {
      tempCode &= ~(1UL << (31-bitIndex));  // write 0
      bitIndex++;
    }
    if(bitIndex == 32)
    {
      cmdli = ~tempCode; // Logical inverted last 8 bits
      cmd = tempCode >> 8; // Second last 8 bits
      if(cmdli == cmd) // Check for errors
      {
        code = tempCode; // If no bit errors
        // Do your main work HERE
      }
      
              switch (code)
        {
          case (0x906fa05f): //volume up
            tmp_for_left = 0;
            if(counter == 10000) 
                {
                 HAL_GPIO_WritePin(GPIOF, led_right_Pin, GPIO_PIN_RESET);  
                 HAL_GPIO_WritePin(GPIOF, led_left_Pin, GPIO_PIN_SET);
                 counter_for_led = 0;
                 tmp_for_left = 1;
                 counter = 0;
                }
            if (tmp_for_left == 0)
            {
                counter+=100;
                counter_for_led ++;
                HAL_TIM_PWM_Stop(&htim10, TIM_CHANNEL_1);   
                HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
            }
            

            break;

          case (0x906fe01f): //volume down
            counter-=100;
            counter_for_led--;
            HAL_TIM_PWM_Stop(&htim10, TIM_CHANNEL_1);   
            HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
            break;

          case (0x906f807f): //page up
            tmp_for_right = 0;
              if(counter == 10000)
              {
                HAL_GPIO_WritePin(GPIOF, led_left_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(GPIOF, led_right_Pin, GPIO_PIN_SET);    
                counter_for_led = 2;
                tmp_for_right = 1;
                counter = 0;
              }            
           if (tmp_for_right == 0)        
           {
            counter++;
            counter_for_led ++;
            HAL_TIM_PWM_Stop(&htim10, TIM_CHANNEL_1);   
            HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
           }
           
            break;

          case (0x906fc03f): // page down
            counter--;
            counter_for_led --;
            HAL_TIM_PWM_Stop(&htim10, TIM_CHANNEL_1);   
            HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
            break;
            
            case (0x906fe51a): // list BIG LEFT
              if(counter == 0)
              {
                 HAL_GPIO_WritePin(GPIOF, led_right_Pin, GPIO_PIN_RESET);  
                 HAL_GPIO_WritePin(GPIOF, led_left_Pin, GPIO_PIN_SET);
                 counter_for_led = 0;
              }
             break;
             
              case (0x906fac53): // q.view BIG RIGHT
              if(counter == 0)
              {
                HAL_GPIO_WritePin(GPIOF, led_left_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(GPIOF, led_right_Pin, GPIO_PIN_SET);    
                counter_for_led = 2;
              }
              break;
              
              case (0x906f8877): // Power off 0-0
                HAL_GPIO_WritePin(GPIOF, led_right_Pin, GPIO_PIN_RESET); 
                HAL_GPIO_WritePin(GPIOF, led_left_Pin, GPIO_PIN_RESET);
                counter = 10000;
                counter_for_led = 200;
            break;
    
          default :
          break;
        }
      
    bitIndex = 0;
    }
  __HAL_TIM_SET_COUNTER(&htim1, 0);
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
