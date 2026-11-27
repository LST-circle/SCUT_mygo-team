/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : iBUS 遥控协议解包 + 串口调试
  *
  * 接线：
  *   接收机 iA6B  B/VCC +  -> 5V
  *   接收机 iA6B  B/VCC -  -> G
  *   接收机 iA6B  CH4   S  -> PA3   (USART2_RX)
  *   CH340        RX       -> PA9   (USART1_TX)
  *   CH340        GND      -> G
  *
  * 串口助手：设备管理器里 CH340 的 COM 口，115200 8N1
  *
  * 说明：所有自定义代码都在 USER CODE 区内，
  *       在 CubeMX 里修改并重新 Generate Code 不会丢失。
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdint.h>
#include <stdarg.h>
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
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static volatile int16_t  g_ch[14];    /* 解出的通道（PPM，单位 us，1000~2000）*/
static volatile uint16_t g_ok;        /* 完整帧计数 */
static volatile uint16_t g_err;       /* 异常帧计数（备用）*/
static volatile uint32_t g_tick;      /* 最近一帧的时刻 */
static volatile uint32_t g_bytes;     /* 收到的边沿数（诊断用）*/
static volatile uint16_t s_last;      /* 上一次边沿的时间戳(us) */
static volatile uint8_t  s_idx;       /* 当前帧里的第几个通道 */
static volatile uint8_t  g_failsafe;  /* 1 = 遥控失联 */
static uint32_t t_print;              /* 打印计时 */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* printf 重定向到 USART1（Keil：Options -> Target 勾 Use MicroLIB） */
int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 10);
  return ch;
}

/* 不依赖 MicroLIB 的打印函数：先格式化到缓冲区，再用 HAL 发出去。
   即使没勾 Use MicroLIB 也能正常输出。 */
static void dbg(const char *fmt, ...)
{
  char buf[128];
  va_list ap;
  int n;

  va_start(ap, fmt);
  n = vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);

  if (n > 0)
  {
    if (n > (int)sizeof(buf)) n = (int)sizeof(buf);
    HAL_UART_Transmit(&huart1, (uint8_t *)buf, (uint16_t)n, 20);
  }
}

/* ================= PPM 解码 =================
   接收机 CH1 输出 PPM：每个通道用"上升沿到上升沿"的间隔表示，
   1000~2000us 对应通道最小值~最大值，帧尾有一个 >3ms 的同步间隔。

   接线：接收机 CH1 的 S  ->  PA0
*/
static void PPM_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_TIM2_CLK_ENABLE();

  /* PA0：上升沿外部中断；下拉，防止信号线拔掉后悬空拾取噪声 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* TIM2：1 MHz 自由计数器，用来打时间戳 */
  TIM2->PSC = 7;              /* 8MHz / 8 = 1MHz，1 个计数 = 1us */
  TIM2->ARR = 0xFFFF;
  TIM2->EGR = TIM_EGR_UG;
  TIM2->CR1 = TIM_CR1_CEN;

  HAL_NVIC_SetPriority(EXTI0_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}

/* PC13 板载 LED（低电平点亮），用来指示失联 */
static void LED_Init(void)
{
  GPIO_InitTypeDef g = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  g.Pin   = GPIO_PIN_13;
  g.Mode  = GPIO_MODE_OUTPUT_PP;
  g.Pull  = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &g);

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);   /* 灭 */
}

/* 把 1000~2000us 归一化到 -1000~1000（整数，避免 printf 打浮点） */
static int norm_x1000(int16_t us)
{
  int v = ((int)us - 1500) * 2;      /* 1000->-1000, 1500->0, 2000->+1000 */

  if (v >  1000) v =  1000;
  if (v < -1000) v = -1000;
  if (v > -30 && v < 30) v = 0;      /* 3% 死区，消除摇杆回中漂移 */

  return v;
}

/* 每个上升沿进一次 */
void EXTI0_IRQHandler(void)
{
  uint16_t now, d;

  if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_0) == RESET) return;
  __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_0);

  now = (uint16_t)TIM2->CNT;
  d   = (uint16_t)(now - s_last);    /* 两次上升沿的间隔(us) */
  s_last = now;
  g_bytes++;                          /* 边沿计数，诊断用 */

  if (d > 3000u)                      /* 帧同步：间隔超过 3ms */
  {
    s_idx = 0;
  }
  else if (d < 600u)                  /* 帧首的短脉冲，不是通道值，丢掉 */
  {
    /* 忽略 */
  }
  else if (s_idx < 14u)
  {
    g_ch[s_idx++] = (int16_t)d;       /* 这就是通道值(us) */
    if (s_idx == 6u)
    {
      g_ok++;                         /* 收满 6 个通道才算一帧有效 */
      g_tick = HAL_GetTick();         /* 只有有效帧才刷新"最后在线时间" */
    }
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

  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  PPM_Init();                                   /* PA0 输入捕获 + TIM2 时基 */
  LED_Init();                                   /* PC13 失联指示 */
  dbg("\r\n==== PPM test start ====\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  while (1)
  {
    /* USER CODE BEGIN 3 */
    if (HAL_GetTick() - t_print >= 200U)
    {
      t_print = HAL_GetTick();

      /* 失联判断：超过 100ms 没有新的一帧 */
      g_failsafe = (uint8_t)((HAL_GetTick() - g_tick) > 100U);

      if (g_failsafe)
      {
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);      /* 失联：LED 闪 */
      }
      else
      {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);  /* 正常：LED 灭 */
      }

      dbg("fs=%u edges=%lu\r\n",
          (unsigned)g_failsafe, (unsigned long)g_bytes);
      dbg("c1=%d c2=%d c3=%d c4=%d c5=%d c6=%d\r\n",
          g_ch[0], g_ch[1], g_ch[2], g_ch[3], g_ch[4], g_ch[5]);
      dbg("v1=%d v2=%d v3=%d v4=%d\r\n",
          norm_x1000(g_ch[0]), norm_x1000(g_ch[1]),
          norm_x1000(g_ch[2]), norm_x1000(g_ch[3]));
    }
    /* USER CODE END 3 */
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

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

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
  huart1.Init.BaudRate = 115200;
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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
  /* 打开 USART2 接收中断 */
  HAL_NVIC_SetPriority(USART2_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}
#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */
