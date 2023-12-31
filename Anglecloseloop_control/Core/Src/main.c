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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include <stdio.h>
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define _constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#define PI 3.14
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
float voltage_limit = 6;
float voltage_power_supply = 12.6;
float shaft_angle = 0, open_loop_timestamp = 0;
float zero_electric_angle = 0, Ualpha, Ubeta = 0, Ua = 0, Ub = 0, Uc = 0, dc_a = 0, dc_b = 0, dc_c = 0;
float Kp = 0.2;
float _normalizeAngle(float angle);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void systick_CountMode(void)
{
  SysTick->LOAD = 0xFFFFFF - 1; // set reload register
  SysTick->VAL = 0;
  SysTick->CTRL = SysTick_CTRL_ENABLE_Msk; // Enable SysTick Timer
}

int PP = 7, DIR = -1;
float _electricalAngle()
{
  return _normalizeAngle((float)(DIR * PP) * GetAngle_without_track() - zero_electric_angle);
}

// 归一化角度到 [0,2PI]
float _normalizeAngle(float angle)
{
  float a = fmod(angle, 2 * PI); // 取余运算可以用于归一化，列出特殊值例子算便知
  return a >= 0 ? a : (a + 2 * PI);
}

void setPWM(float Ua, float Ub, float Uc)
{
  // 限制上限
  Ua = _constrain(Ua, 0.f, voltage_limit);
  Ub = _constrain(Ub, 0.f, voltage_limit);
  Uc = _constrain(Uc, 0.f, voltage_limit);
  // 计算占空比
  // 限制占空比在0~1
  dc_a = _constrain(Ua / voltage_power_supply, 0.f, 1.f);
  dc_b = _constrain(Ub / voltage_power_supply, 0.f, 1.f);
  dc_c = _constrain(Uc / voltage_power_supply, 0.f, 1.f);

  char buffer[12];
  sprintf(buffer, "%f", dc_b);
  HAL_UART_Transmit(&huart1, "\r\n", strlen("\r\n"), 0xFFFF);
  HAL_UART_Transmit(&huart1, (uint8_t *)buffer, strlen(buffer), 0xFFFF);

  __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, dc_a * 2399);
  __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, dc_b * 2399);
  __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, dc_c * 2399);
}

void setPhaseVoltage(float Uq, float Ud, float angle_el)
{
  angle_el = _normalizeAngle(angle_el + zero_electric_angle);
  // 帕克逆变换
  Ualpha = -Uq * sin(angle_el);
  Ubeta = Uq * cos(angle_el);

  // 克拉克逆变换
  Ua = Ualpha + voltage_power_supply / 2;
  Ub = (sqrt(3) * Ubeta - Ualpha) / 2 + voltage_power_supply / 2;
  Uc = (-Ualpha - sqrt(3) * Ubeta) / 2 + voltage_power_supply / 2;
  setPWM(Ua, Ub, Uc);
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
  /* USER CODE BEGIN 1 */
  float motor_target = 3.14;
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
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // 开启三个PWM通道输出
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_UART_Transmit(&huart1, "helloWorld\r\n", strlen("helloWorld\r\n"), 0xFFFF);
  HAL_Delay(1000);
  systick_CountMode();
  bsp_as5600Init();
  zero_electric_angle = _electricalAngle();

  char buffer[12];
  sprintf(buffer, "%f", zero_electric_angle);
  HAL_UART_Transmit(&huart1, "\r\n", strlen("\r\n"), 0xFFFF);
  HAL_UART_Transmit(&huart1, (uint8_t *)buffer, strlen(buffer), 0xFFFF);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    float Sensor_Angle = GetAngle();
    char buffer[12];
    sprintf(buffer, "%f", GetAngle());
    HAL_UART_Transmit(&huart1, "\r\n", strlen("\r\n"), 0xFFFF);
    HAL_UART_Transmit(&huart1, (uint8_t *)buffer, strlen(buffer), 0xFFFF);
    setPhaseVoltage(_constrain(Kp * (motor_target - DIR * Sensor_Angle) * 180 / _PI, -6, 6), 0, _electricalAngle());
//		setPhaseVoltage(20, 0, _electricalAngle());
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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
