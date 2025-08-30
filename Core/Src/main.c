/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
    MOTOR_STOPPED,
    MOTOR_FORWARD,
    MOTOR_BACKWARD,
    MOTOR_TURNING_LEFT,
    MOTOR_TURNING_RIGHT
} MotorState_t;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
// Declare this at the top of main.c
uint8_t received_char;
uint16_t pwm_value = 600; // 60% duty cycle
MotorState_t current_motor_state = MOTOR_STOPPED;
/* USER CODE END PV */
typedef struct {
    GPIO_TypeDef* port;
    uint16_t in1_pin;
    uint16_t in2_pin;
} MotorConfig_t;

const MotorConfig_t motor_config[4] = {
    {GPIOA, GPIO_PIN_0, GPIO_PIN_1},   // M1 - Left
    {GPIOA, GPIO_PIN_2, GPIO_PIN_3},   // M2 - Left
    {GPIOB, GPIO_PIN_0, GPIO_PIN_1},   // M3 - Right
    {GPIOB, GPIO_PIN_10, GPIO_PIN_11}  // M4 - Right
};
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */


/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Set_Motor_Speeds(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4) {
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, m1); // Motor 1
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, m2); // Motor 2
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, m3); // Motor 3
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, m4); // Motor 4
}
void Set_Motor_Direction(uint8_t motor_index, uint8_t forward) {
    if (motor_index >= 4) return;

    if (forward) {
        HAL_GPIO_WritePin(motor_config[motor_index].port, motor_config[motor_index].in1_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(motor_config[motor_index].port, motor_config[motor_index].in2_pin, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(motor_config[motor_index].port, motor_config[motor_index].in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor_config[motor_index].port, motor_config[motor_index].in2_pin, GPIO_PIN_SET);
    }
}
void Stop_Motor(uint8_t motor_index) {
    if (motor_index >= 4) return;

    HAL_GPIO_WritePin(motor_config[motor_index].port, motor_config[motor_index].in1_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(motor_config[motor_index].port, motor_config[motor_index].in2_pin, GPIO_PIN_RESET);
}

void Stop_Motors() {
    Set_Motor_Speeds(0, 0, 0, 0); // Set all PWM to 0

    // Stop all motors using helper function
    for (int i = 0; i < 4; i++) {
        Stop_Motor(i);
    }

    current_motor_state = MOTOR_STOPPED;
}

void Move_Forward() {
    // Stop motors first if changing from backward
    if (current_motor_state == MOTOR_BACKWARD) {
        Stop_Motors();
        HAL_Delay(100); // Small delay to let motors stop completely
    }

    Set_Motor_Speeds(600, 600, 600, 600);

    // Set all motors forward
    for (int i = 0; i < 4; i++) {
        Set_Motor_Direction(i, 1); // 1 = forward
    }

    current_motor_state = MOTOR_FORWARD;
}

void Move_Backward() {
    // Stop motors first if changing from forward
    if (current_motor_state == MOTOR_FORWARD) {
        Stop_Motors();
        HAL_Delay(100); // Small delay to let motors stop completely
    }

    Set_Motor_Speeds(600, 600, 600, 600);

    // Set all motors backward
    for (int i = 0; i < 4; i++) {
        Set_Motor_Direction(i, 0); // 0 = backward
    }

    current_motor_state = MOTOR_BACKWARD;
}
void Turn_Left() {
    switch (current_motor_state) {
        case MOTOR_FORWARD:
        case MOTOR_TURNING_RIGHT:
            // Forward left turn - stop left motors, right motors continue forward
            Set_Motor_Speeds(0, 0, 700, 700); // Left motors stop, right motors fast
            Stop_Motor(0); // M1 off
            Stop_Motor(1); // M2 off
            Set_Motor_Direction(2, 1); // M3 forward
            Set_Motor_Direction(3, 1); // M4 forward
            break;

        case MOTOR_BACKWARD:
            // Backward left turn - stop left motors, right motors continue backward
            Set_Motor_Speeds(0, 0, 700, 700); // Left motors stop, right motors fast
            Stop_Motor(0); // M1 off
            Stop_Motor(1); // M2 off
            Set_Motor_Direction(2, 0); // M3 backward
            Set_Motor_Direction(3, 0); // M4 backward
            break;

        case MOTOR_TURNING_LEFT:
            // Already turning left, maintain current behavior
            return;

        default: // MOTOR_STOPPED
            // Stationary left turn - left motors backward, right motors forward
            Set_Motor_Speeds(600, 600, 600, 600);
            Set_Motor_Direction(0, 0); // M1 backward
            Set_Motor_Direction(1, 0); // M2 backward
            Set_Motor_Direction(2, 1); // M3 forward
            Set_Motor_Direction(3, 1); // M4 forward
            break;
    }

    current_motor_state = MOTOR_TURNING_LEFT;
}

void Turn_Right() {
    switch (current_motor_state) {
        case MOTOR_FORWARD:
        case MOTOR_TURNING_LEFT:
            // Forward right turn - stop right motors, left motors continue forward
            Set_Motor_Speeds(700, 700, 0, 0); // Left motors fast, right motors stop
            Set_Motor_Direction(0, 1); // M1 forward
            Set_Motor_Direction(1, 1); // M2 forward
            Stop_Motor(2); // M3 off
            Stop_Motor(3); // M4 off
            break;

        case MOTOR_BACKWARD:
            // Backward right turn - stop right motors, left motors continue backward
            Set_Motor_Speeds(700, 700, 0, 0); // Left motors fast, right motors stop
            Set_Motor_Direction(0, 0); // M1 backward
            Set_Motor_Direction(1, 0); // M2 backward
            Stop_Motor(2); // M3 off
            Stop_Motor(3); // M4 off
            break;

        case MOTOR_TURNING_RIGHT:
            // Already turning right, maintain current behavior
            return;

        default: // MOTOR_STOPPED
            // Stationary right turn - right motors backward, left motors forward
            Set_Motor_Speeds(600, 600, 600, 600);
            Set_Motor_Direction(0, 1); // M1 forward
            Set_Motor_Direction(1, 1); // M2 forward
            Set_Motor_Direction(2, 0); // M3 backward
            Set_Motor_Direction(3, 0); // M4 backward
            break;
    }

    current_motor_state = MOTOR_TURNING_RIGHT;
}
// You can define Move_Backward(), Turn_Left(), Turn_Right() in similar way

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();


  // Start PWM channels
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);



 // Start UART receive interrupt
  // In main(), after starting PWM timers:
  Stop_Motors();  // Initialize all motors to stopped state
  HAL_UART_Receive_IT(&huart1, &received_char, 1);  // Start UART interrupt

  while (1) {
    // Main loop does nothing; all control via UART interrupt
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
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
  htim3.Init.Prescaler = 83;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  sConfigOC.Pulse = 600;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
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
  htim4.Init.Prescaler = 83;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  sConfigOC.Pulse = 600;
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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB10 PB11 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        switch (received_char) {
            case 'F': // Forward
            case 'f':
                Move_Forward();
                break;

            case 'B': // Backward
            case 'b':
                Move_Backward();
                break;

            case 'L': // Turn Left
            case 'l':
                Turn_Left();
                break;

            case 'R': // Turn Right
            case 'r':
                Turn_Right();
                break;

            case 'S': // Stop
            case 's':
                Stop_Motors();
                break;

            case '0': // Emergency stop
                Stop_Motors();
                break;

            default:
                // Unknown command - could add error feedback here
                break;
        }

        // Re-enable UART receive interrupt for next command
        HAL_UART_Receive_IT(&huart1, &received_char, 1);
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
