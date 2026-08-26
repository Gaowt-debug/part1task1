/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>

/* ============================ 系统功能总览 ============================
 * 硬件：STM32F103C8T6，HSE 8MHz ×PLL9 = 72MHz，APB1=36MHz
 *
 * 功能 1 —— CAN 接收控制呼吸灯频率：
 *   CAN(500kbps, ID 0x111) → FIFO0 中断回调入队 can_receive_mail
 *   → Task1 解析最后 1~2 字节为频率值 → 队列 ledlight_control
 *   → Task2 换算步进 light_change → 软件定时器(50ms) ledlight_update_
 *   → 三角波更新 TIM1_CH1 PWM 占空比（PA8 呼吸灯）
 *
 * 功能 2 —— USART1 不定长接收 + 回显：
 *   DMA 收数据到缓冲池 → IDLE 中断收尾（算帧长/入队/切缓冲）
 *   → 队列 usart1_receive_data 传缓冲区编号 → Task3
 *   → 拼接 "Receive Data:"+数据 DMA 回发（信号量防重入）
 *
 * 任务优先级：Task1(Normal) > Task3(Normal1) > Task2(BelowNormal7)
 * ==================================================================== */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
	uint16_t id;
	uint8_t data[8];
	uint8_t len;
} can_receive_pack;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* Definitions for Task1 */
osThreadId_t Task1Handle;
const osThreadAttr_t Task1_attributes = {
  .name = "Task1",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task2 */
osThreadId_t Task2Handle;
const osThreadAttr_t Task2_attributes = {
  .name = "Task2",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal7,
};
/* Definitions for Task3 */
osThreadId_t Task3Handle;
const osThreadAttr_t Task3_attributes = {
  .name = "Task3",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for can_receive_mail */
osMessageQueueId_t can_receive_mailHandle;
const osMessageQueueAttr_t can_receive_mail_attributes = {
  .name = "can_receive_mail"
};
/* Definitions for ledlight_control */
osMessageQueueId_t ledlight_controlHandle;
const osMessageQueueAttr_t ledlight_control_attributes = {
  .name = "ledlight_control"
};
/* Definitions for usart1_receive_data */
osMessageQueueId_t usart1_receive_dataHandle;
const osMessageQueueAttr_t usart1_receive_data_attributes = {
  .name = "usart1_receive_data"
};
/* Definitions for ledlight_update */
osTimerId_t ledlight_updateHandle;
const osTimerAttr_t ledlight_update_attributes = {
  .name = "ledlight_update"
};
/* USER CODE BEGIN PV */


/*==================== USART1 DMA 接收缓冲池 ====================*/
/* 环形缓冲池：5 个缓冲区轮流给 DMA 写入。IDLE 中断收到一帧后切换到
 * 下一个缓冲区，避免 DMA 覆盖 Task3 还未读完的数据 */
uint8_t usart1_receive_pool[5][33]={0};   // 接收缓冲池（每格最大 32 字节 + 1 字节余量）
uint8_t usart1_receive_len[3]={0};        // 各缓冲区本次实际接收的帧长（字节），与缓冲池编号对应
uint8_t usart1_receive_pool_idx=0;        // 当前 DMA 正在写入的缓冲区编号（0~4 环形递增）

/*==================== UART 发送同步信号量 ====================*/
/* 二值信号量（初值 1）：Task3 发送前 acquire，DMA 发送完成中断里
 * release。保证同一时刻只有一次 DMA 发送在进行，防止下次发送覆盖
 * 上次还没发完的数据 */
osSemaphoreId_t uart_tx_sem;

/*==================== 呼吸灯控制变量 ====================*/
/* volatile：Task2 写 / 软件定时器回调读，跨上下文访问必须加 */
volatile int light_change=100;  // 每 50ms 亮度变化步进 = 100000 / f
volatile int f=1000;            // 当前呼吸频率（ms），数值越小呼吸越快
volatile int f_new=1000;        // Task2 从队列取出的新频率，与 f 比较去重用







/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_CAN_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
void StartTask1(void *argument);
void StartTask2(void *argument);
void StartTask3(void *argument);
void ledlight_update_(void *argument);

/* USER CODE BEGIN PFP */
/**
 * @brief CAN FIFO0 接收回调（中断上下文）
 *        FIFO0 收到报文时触发：取出报文打包成 can_receive_pack，
 *        入队 can_receive_mail 交给 Task1 处理。
 *        注意：队列满时 osMessageQueuePut 非阻塞直接丢弃本帧。
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  can_receive_pack message = {0};
  CAN_RxHeaderTypeDef rx_header = {0};
  uint8_t raw_buf[8];

  if (hcan->Instance != CAN1)      // 只处理 CAN1，防止误触发
    return;

  /* 从 FIFO0 取出报文（原始 8 字节数据 + 报文头） */
  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, raw_buf);

  /* 按标准帧/扩展帧取对应 ID，记录 DLC 长度并拷贝有效数据 */
  message.id = (rx_header.IDE == CAN_ID_STD) ? rx_header.StdId : rx_header.ExtId;
  message.len = rx_header.DLC;
  memcpy(message.data, raw_buf, message.len);

  /* PC13 翻转：调试用，收到一帧 CAN 闪一次板载 LED */
HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);

  /* 打包入队，交给 Task1 解析（非阻塞，timeout=0） */
  osMessageQueuePut(can_receive_mailHandle, &message, 0U, 0U);
}

/**
 * @brief CAN 发送一帧标准数据帧（当前预留未使用）
 * @param stdId 目标标准 ID；pData 数据指针；len 数据长度（>8 截断为 8）
 * @retval 1 发送成功（已进入邮箱），0 失败（无空闲邮箱或写入出错）
 */
uint8_t CAN_sendmail(uint32_t stdId, uint8_t *pData, uint8_t len)
{
  CAN_TxHeaderTypeDef txHeader = {0};
  uint32_t txMailbox = 0;
  uint8_t tempData[8] = {0};

  /* 组装发送报文头：标准帧 + 数据帧 */
  txHeader.StdId = stdId;
  txHeader.IDE = CAN_ID_STD;
  txHeader.RTR = CAN_RTR_DATA;
  txHeader.DLC =
      (len > 8) ? 8 : len;              // CAN 单帧最多 8 字节
  memcpy(tempData, pData, txHeader.DLC); // 拷贝到局部缓冲，防止源数据变动

  /* 三个发送邮箱全忙则放弃本次发送 */
  if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0)
  {
    return 0;
  }
  if (HAL_CAN_AddTxMessage(&hcan, &txHeader, tempData, &txMailbox) != HAL_OK)
  {
    return 0;
  }
  return 1;
}
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
	MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_CAN_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
 /* 启动 TIM1 CH1 PWM 输出（PA8，呼吸灯用，占空比由 ledlight_update 定时器回调动态调整） */
 HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* 创建 UART 发送完成信号量：max=1, 初值=1（二值信号量），
   * 用于 Task3 与 DMA 发送完成中断之间的同步 */
  uart_tx_sem = osSemaphoreNew(1, 1, NULL);
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of ledlight_update */
  ledlight_updateHandle = osTimerNew(ledlight_update_, osTimerPeriodic, NULL, &ledlight_update_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */

  /* 启动呼吸灯周期软件定时器：每 50ms 执行一次 ledlight_update_，
   * 在回调中累加/回减 PWM 比较值，实现呼吸效果 */
  osTimerStart(ledlight_updateHandle,pdMS_TO_TICKS(50));
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of can_receive_mail */
  can_receive_mailHandle = osMessageQueueNew (6, sizeof(can_receive_pack), &can_receive_mail_attributes);

  /* creation of ledlight_control */
  ledlight_controlHandle = osMessageQueueNew (1, sizeof(uint16_t), &ledlight_control_attributes);

  /* creation of usart1_receive_data */
  usart1_receive_dataHandle = osMessageQueueNew (3, sizeof(uint8_t), &usart1_receive_data_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* 三个队列的分工：
   * can_receive_mail   (6 深度)  : CAN 回调 → Task1，传 can_receive_pack 结构体
   * ledlight_control   (1 深度)  : Task1 → Task2，传呼吸频率 f（注意按 uint16_t 创建）
   * usart1_receive_data(3 深度)  : USART1 IDLE 中断 → Task3，传缓冲区编号（索引而非数据本体）
   */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Task1 */
  Task1Handle = osThreadNew(StartTask1, NULL, &Task1_attributes);

  /* creation of Task2 */
  Task2Handle = osThreadNew(StartTask2, NULL, &Task2_attributes);

  /* creation of Task3 */
  Task3Handle = osThreadNew(StartTask3, NULL, &Task3_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 4;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_9TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_8TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
/* ============ CAN 过滤器配置 ============ */
/* 波特率：APB1=36MHz / Prescaler 4 / (1+9+8)TQ = 500kbps */
CAN_FilterTypeDef sFilterConfig = {0};
uint32_t id = 0x0111;   // 期望接收的标准 ID
uint32_t mask = 0x0fff; // 掩码：低 12 位必须匹配（含 RTR/IDE 位）
/* 32bit 过滤器模式位段：STID 位于 bit[31:21]，故左移 21 位对齐 */
uint32_t id32 = id << 21;
uint32_t mask32 = mask << 21;
sFilterConfig.FilterBank = 0;                        // 使用过滤器组 0
sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;    // 掩码模式
sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;   // 32 位宽度
sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0; // 命中报文存入 FIFO0
sFilterConfig.FilterActivation = ENABLE;
sFilterConfig.FilterIdHigh = (uint16_t)(id32 >> 16);   // 过滤 ID 高 16 位
sFilterConfig.FilterIdLow  = (uint16_t)(id32 & 0xFFFF);// 过滤 ID 低 16 位
sFilterConfig.FilterMaskIdHigh = (uint16_t)(mask32 >> 16);
sFilterConfig.FilterMaskIdLow  = (uint16_t)(mask32 & 0xFFFF);

HAL_CAN_ConfigFilter(&hcan,&sFilterConfig);

/* 使能 FIFO0 接收挂起中断：收到报文触发 HAL_CAN_RxFifo0MsgPendingCallback */
HAL_CAN_ActivateNotification(&hcan,CAN_IT_RX_FIFO0_MSG_PENDING);

HAL_CAN_Start(&hcan);   // 启动 CAN 外设
  /* USER CODE END CAN_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 72-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
	/* ============ USART1 空闲中断 + DMA 不定长接收 ============ */
	/* 使能 IDLE 空闲中断：一帧数据发完后总线空闲（1 字节时间无数据）
	 * 触发中断，在 USART1_IRQHandler 里收尾（详见 stm32f1xx_it.c） */
	__HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);
	/* 启动 DMA 接收：最多收 32 字节，写入缓冲池当前格，实际帧长由 IDLE 中断算出 */
	HAL_UART_Receive_DMA(&huart1,usart1_receive_pool[usart1_receive_pool_idx],32);
  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/**
 * @brief USART1 DMA 发送完成回调（中断上下文）
 *        一帧数据 DMA 发完后释放信号量，通知 Task3 可以发起下一次发送
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    osSemaphoreRelease(uart_tx_sem);   // 释放发送权
  }
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartTask1 */
/**
  * @brief  Function implementing the Task1 thread.
  *         CAN 报文解析任务：阻塞等待 CAN 接收队列，把 ID 0x111 报文
  *         中的数据解析为呼吸频率（小端 16bit），转发给 Task2。
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartTask1 */
void StartTask1(void *argument)
{
  /* USER CODE BEGIN 5 */
  can_receive_pack receive_pack = {0};  // 从队列取出的 CAN 报文
  int can_light_f_ms = 0;               // 解析出的呼吸频率值（ms）
  /* Infinite loop */
  for (;;)
  {
    /* 阻塞等待 CAN 回放入队的报文（队列空时挂起，不占 CPU） */
    xQueueReceive(can_receive_mailHandle, &receive_pack, HAL_MAX_DELAY);

    if (receive_pack.id == 0x111)   // 只处理目标 ID 的报文
    {
      can_light_f_ms = 0;
      if (receive_pack.len == 1)
      {
        /* 单字节数据：直接作为频率值（0~255） */
        can_light_f_ms = receive_pack.data[receive_pack.len - 1];
      }
      else if (receive_pack.len > 1)
      {
        /* 多字节数据：取最后两字节按小端拼成 16bit（低字节在前）
         * 例：发 0xE8 0x03 → 1000 */
        can_light_f_ms = receive_pack.data[receive_pack.len - 1] + receive_pack.data[receive_pack.len - 2] * 256;
      }
      /* 非阻塞发送给 Task2 更新呼吸频率（队列满则丢弃本值） */
      xQueueSend(ledlight_controlHandle, &can_light_f_ms, 0);
    }
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask2 */
/**
* @brief Function implementing the Task2 thread.
*        呼吸频率更新任务：阻塞等待 Task1 发来的新频率，
*        换算成每 50ms 的亮度步进值，供定时器回调使用。
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask2 */
void StartTask2(void *argument)
{
  /* USER CODE BEGIN StartTask2 */
  /* Infinite loop */
  for (;;)
  {

    /* 阻塞等待新频率值（队列空时挂起） */
    xQueueReceive(ledlight_controlHandle, &f_new, HAL_MAX_DELAY);

    /* 去重 + 过滤非法值：频率没变或为 0（会导致除零）则忽略 */
    if (f_new != f && f_new != 0)
    {
      f = f_new;                    // 更新当前频率
      /* 换算亮度步进：三角波全程 0→999→0 共 2000 级，
       * 一次完整呼吸周期 = 2000/step 个 50ms，即周期 ≈ f ms */
      light_change = 100000 / f;
    }

  }
  /* USER CODE END StartTask2 */
}

/* USER CODE BEGIN Header_StartTask3 */
/**
* @brief Function implementing the Task3 thread.
*        串口回显任务：阻塞等待 IDLE 中断入队的缓冲区编号，
*        把收到的数据拼上报头 "Receive Data:" 后 DMA 回发。
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask3 */
void StartTask3(void *argument)
{
  /* USER CODE BEGIN StartTask3 */
  uint8_t tosend[47]={0};   // 发送缓冲：13 字节报头 + 最多 32 字节数据 + 换行 + 结尾 0
  uint8_t *receive;         // 指向缓冲池中本次收到的数据
  uint8_t receive_idx;      // IDLE 中断传来的缓冲区编号
  uint8_t receive_len;      // 本帧实际长度
  uint16_t offset;          // memcpy 拼接游标：当前写入位置
  /* Infinite loop */
  for(;;)
  {
/* 阻塞等待 IDLE 中断投递的缓冲区编号（队列空时挂起） */
osMessageQueueGet(usart1_receive_dataHandle,&receive_idx,NULL,HAL_MAX_DELAY);

/* 清空发送缓冲，防止上次残留 */
for(char i=0;i<47;i++)
{
tosend[i]=0;
}

/* 按编号从缓冲池取出数据和帧长 */
receive=usart1_receive_pool[receive_idx];
receive_len=usart1_receive_len[receive_idx];

/* 用 memcpy+offset 拼接（不用 strcat：数据里可能有 0x00）：
 * "Receive Data:" + 原始数据 + "\n" */
offset = 0;
memcpy(tosend + offset, "Receive Data:", 13);
offset += 13;
memcpy(tosend + offset, receive, receive_len);
offset += receive_len;
memcpy(tosend + offset, "\n", 1);
offset += 1;

/* 拿到发送权才能发（DMA 忙则阻塞），发完由 TxCplt 回调释放信号量 */
osSemaphoreAcquire(uart_tx_sem, HAL_MAX_DELAY);
HAL_UART_Transmit_DMA(&huart1, tosend, offset);

    osDelay(1);
  }
  /* USER CODE END StartTask3 */
}

/* ledlight_update_ function */
/**
 * @brief 呼吸灯刷新回调（软件定时器，每 50ms 执行一次，定时器服务任务上下文）
 *        用三角波生成占空比 0~999 往返变化：
 *        上升沿：light += step；到达 999 后折返
 *        下降沿：light -= step；到达 0 后折返
 *        一个完整呼吸周期 = 2000/step × 50ms
 */
void ledlight_update_(void *argument)
{
  /* USER CODE BEGIN ledlight_update_ */
  static int light; // 当前亮度（PWM 比较值 0~999），static 跨回调保持
  static char per;  // 方向标志：1=渐亮（递增），0=渐暗（递减）
  
  /* 按当前方向更新亮度：步进 light_change 由 Task2 根据频率 f 换算 */
  if (per == 1)
  {
    light += light_change;   // 渐亮
  }
  else
  {
    light -= light_change;   // 渐暗
  }

  /* 越界折返（while 处理步进大于剩余区间时可能一次越过多个周期的情况）：
   * light < 0  → 反射成正数，转为渐亮
   * light > 999 → 关于 999 镜像反射，转为渐暗 */
  while (light < 0 || light > 999)
  {
    if (light < 0)
    {
      light = -light;        // 反射回正区间
      per = 1;               // 切换为渐亮
    }
    else
    {
      light = 1998 - light;  // 关于 999 折返（999 - (light-999)）
      per = 0;               // 切换为渐暗
    }
  }

  /* 更新 TIM1 CH1 的 PWM 比较值 → 改变占空比 → 亮度变化 */
__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,light);
  /* USER CODE END ledlight_update_ */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
