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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define AUDIO_SAMPLE_RATE    48000
#define AUDIO_BUFFER_SAMPLES 480      // 10ms @ 48kHz
#define AUDIO_CHANNELS       2        // Stereo
#define AUDIO_BUFFER_SIZE    (AUDIO_BUFFER_SAMPLES * AUDIO_CHANNELS)

#define TEST_TONE_FREQ       1000.0f  // 1kHz test tone
#ifndef M_PI
#define M_PI                 3.14159265358979323846f
#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;

SAI_HandleTypeDef hsai_BlockA1;
DMA_HandleTypeDef hdma_sai1_a;

UART_HandleTypeDef huart7;

/* USER CODE BEGIN PV */
// Audio buffer (double buffer for DMA circular mode)
// Format: [L0][R0][L1][R1]...[Ln][Rn]
// MUST be in RAM_D2 for DMA1 access (DTCMRAM is NOT DMA-accessible on STM32H7!)
static int32_t audio_buffer[AUDIO_BUFFER_SIZE * 2] __attribute__((section(".dma_buffer"), aligned(4)));

// Phase accumulator for continuous sine wave
static float sine_phase = 0.0f;

// Callback counters for debug
static volatile uint32_t half_complete_count = 0;
static volatile uint32_t full_complete_count = 0;
static volatile uint32_t error_count = 0;
static volatile uint32_t last_sai_error = 0;

// UART TX buffer
static char uart_buf[128];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_UART7_Init(void);
static void MX_SAI1_Init(void);
/* USER CODE BEGIN PFP */
static void Fill_Audio_Buffer(int32_t *buffer, uint32_t samples);
static void UART_Print(const char *str);
static void Print_DMA_Config(void);
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

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_UART7_Init();
  MX_SAI1_Init();
  /* USER CODE BEGIN 2 */

  // Initialize PE3 as output for WeAct onboard LED (active LOW)
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  // Startup message
  UART_Print("\r\n=== WeAct I2S Audio Test ===\r\n");
  UART_Print("SAI1 Config: 48kHz, 24-bit, Stereo\r\n");
  UART_Print("Test tone: 1kHz sine wave\r\n\r\n");

  // LED ON to indicate startup (PE3 active LOW on WeAct board)
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);  // LED ON (active low)

  // Pre-fill the entire buffer with audio data
  Fill_Audio_Buffer(&audio_buffer[0], AUDIO_BUFFER_SAMPLES);
  Fill_Audio_Buffer(&audio_buffer[AUDIO_BUFFER_SIZE], AUDIO_BUFFER_SAMPLES);

  // Debug: Print SAI configuration
  sprintf(uart_buf, "SAI1 AudioFreq: %lu Hz\r\n", hsai_BlockA1.Init.AudioFrequency);
  UART_Print(uart_buf);
  sprintf(uart_buf, "Buffer size: %d samples (%d bytes)\r\n", AUDIO_BUFFER_SIZE * 2, (int)sizeof(audio_buffer));
  UART_Print(uart_buf);

  // Start SAI DMA transmission (circular mode)
  HAL_StatusTypeDef status = HAL_SAI_Transmit_DMA(&hsai_BlockA1, (uint8_t*)audio_buffer, AUDIO_BUFFER_SIZE * 2);

  if (status == HAL_OK) {
    UART_Print("DMA Started OK!\r\n");
    // Verify SAI and DMA states
    sprintf(uart_buf, "SAI State after start: %d (2=BUSY_TX)\r\n", HAL_SAI_GetState(&hsai_BlockA1));
    UART_Print(uart_buf);
    sprintf(uart_buf, "DMA State: %d (2=READY, 3=BUSY)\r\n", hdma_sai1_a.State);
    UART_Print(uart_buf);

    // Print detailed DMA configuration for verification
    Print_DMA_Config();
  } else {
    sprintf(uart_buf, "DMA Start FAILED! Status: %d\r\n", status);
    UART_Print(uart_buf);
    sprintf(uart_buf, "SAI Error Code: 0x%08lX\r\n", HAL_SAI_GetError(&hsai_BlockA1));
    UART_Print(uart_buf);
  }

  // Small delay then check state again
  HAL_Delay(100);
  sprintf(uart_buf, "After 100ms - SAI State: %d, Errors: %lu\r\n",
          HAL_SAI_GetState(&hsai_BlockA1), error_count);
  UART_Print(uart_buf);

  // Variables for main loop
  uint32_t last_tick = HAL_GetTick();
  uint32_t last_half = 0;
  uint32_t last_full = 0;
  uint32_t test_start = HAL_GetTick();
  uint8_t test_complete = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // Print status every second
    if (HAL_GetTick() - last_tick >= 1000) {
      last_tick = HAL_GetTick();

      // LED now toggled in Half callback at 50Hz for DMA sync visual

      // Calculate callbacks per second
      uint32_t half_per_sec = half_complete_count - last_half;
      uint32_t full_per_sec = full_complete_count - last_full;
      last_half = half_complete_count;
      last_full = full_complete_count;

      // Expected: 50 half + 50 full per second (20ms buffer cycle @ 48kHz stereo)
      sprintf(uart_buf, "[%lus] Half: %lu/s, Full: %lu/s, Errors: %lu\r\n",
              HAL_GetTick() / 1000, half_per_sec, full_per_sec, error_count);
      UART_Print(uart_buf);

      // Check SAI status
      HAL_SAI_StateTypeDef sai_state = HAL_SAI_GetState(&hsai_BlockA1);
      if (sai_state != HAL_SAI_STATE_BUSY_TX) {
        sprintf(uart_buf, "WARNING: SAI State = %d (expected BUSY_TX=2)\r\n", sai_state);
        UART_Print(uart_buf);
        sprintf(uart_buf, "SAI Error: 0x%08lX, Last: 0x%08lX\r\n",
                HAL_SAI_GetError(&hsai_BlockA1), last_sai_error);
        UART_Print(uart_buf);
      }

      // 60-second test summary
      if (!test_complete && (HAL_GetTick() - test_start >= 60000)) {
        test_complete = 1;
        UART_Print("\r\n========== 60 SECOND TEST COMPLETE ==========\r\n");
        sprintf(uart_buf, "Total Half callbacks: %lu (expected ~3000)\r\n", half_complete_count);
        UART_Print(uart_buf);
        sprintf(uart_buf, "Total Full callbacks: %lu (expected ~3000)\r\n", full_complete_count);
        UART_Print(uart_buf);
        sprintf(uart_buf, "Total Errors: %lu\r\n", error_count);
        UART_Print(uart_buf);

        // Verify timing accuracy
        float half_rate = (float)half_complete_count / 60.0f;
        float full_rate = (float)full_complete_count / 60.0f;
        sprintf(uart_buf, "Avg Half rate: %.1f/s (expected 50)\r\n", half_rate);
        UART_Print(uart_buf);
        sprintf(uart_buf, "Avg Full rate: %.1f/s (expected 50)\r\n", full_rate);
        UART_Print(uart_buf);

        if (error_count == 0 && half_complete_count >= 2900 && full_complete_count >= 2900) {
          UART_Print(">>> TEST PASSED: DMA working correctly! <<<\r\n");
        } else {
          UART_Print(">>> TEST FAILED: Check errors above <<<\r\n");
        }
        UART_Print("==============================================\r\n\r\n");
      }
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 12;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOMEDIUM;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SAI1|RCC_PERIPHCLK_UART7;
  PeriphClkInitStruct.PLL3.PLL3M = 5;
  PeriphClkInitStruct.PLL3.PLL3N = 68;
  PeriphClkInitStruct.PLL3.PLL3P = 28;
  PeriphClkInitStruct.PLL3.PLL3Q = 3;
  PeriphClkInitStruct.PLL3.PLL3R = 2;
  PeriphClkInitStruct.PLL3.PLL3RGE = RCC_PLL3VCIRANGE_2;
  PeriphClkInitStruct.PLL3.PLL3VCOSEL = RCC_PLL3VCOWIDE;
  PeriphClkInitStruct.PLL3.PLL3FRACN = 6660;  // For exact 12.288 MHz MCLK (was 6329)
  PeriphClkInitStruct.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLL3;
  PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_PLL3;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00909FCE;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SAI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SAI1_Init(void)
{

  /* USER CODE BEGIN SAI1_Init 0 */

  /* USER CODE END SAI1_Init 0 */

  /* USER CODE BEGIN SAI1_Init 1 */

  /* USER CODE END SAI1_Init 1 */
  hsai_BlockA1.Instance = SAI1_Block_A;
  hsai_BlockA1.Init.Protocol = SAI_FREE_PROTOCOL;
  hsai_BlockA1.Init.AudioMode = SAI_MODEMASTER_TX;
  hsai_BlockA1.Init.DataSize = SAI_DATASIZE_24;
  hsai_BlockA1.Init.FirstBit = SAI_FIRSTBIT_MSB;
  hsai_BlockA1.Init.ClockStrobing = SAI_CLOCKSTROBING_FALLINGEDGE;
  hsai_BlockA1.Init.Synchro = SAI_ASYNCHRONOUS;
  hsai_BlockA1.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
  hsai_BlockA1.Init.NoDivider = SAI_MCK_OVERSAMPLING_DISABLE;
  hsai_BlockA1.Init.MckOverSampling = SAI_MCK_OVERSAMPLING_DISABLE;
  hsai_BlockA1.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
  hsai_BlockA1.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_48K;
  hsai_BlockA1.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
  hsai_BlockA1.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockA1.Init.CompandingMode = SAI_NOCOMPANDING;
  hsai_BlockA1.Init.TriState = SAI_OUTPUT_NOTRELEASED;
  hsai_BlockA1.Init.PdmInit.Activation = DISABLE;
  hsai_BlockA1.Init.PdmInit.MicPairsNbr = 1;
  hsai_BlockA1.Init.PdmInit.ClockEnable = SAI_PDM_CLOCK1_ENABLE;
  hsai_BlockA1.FrameInit.FrameLength = 64;
  hsai_BlockA1.FrameInit.ActiveFrameLength = 32;
  hsai_BlockA1.FrameInit.FSDefinition = SAI_FS_CHANNEL_IDENTIFICATION;
  hsai_BlockA1.FrameInit.FSPolarity = SAI_FS_ACTIVE_LOW;
  hsai_BlockA1.FrameInit.FSOffset = SAI_FS_FIRSTBIT;
  hsai_BlockA1.SlotInit.FirstBitOffset = 0;
  hsai_BlockA1.SlotInit.SlotSize = SAI_SLOTSIZE_32B;
  hsai_BlockA1.SlotInit.SlotNumber = 2;
  hsai_BlockA1.SlotInit.SlotActive = 0x00000003;
  if (HAL_SAI_Init(&hsai_BlockA1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SAI1_Init 2 */

  /* USER CODE END SAI1_Init 2 */

}

/**
  * @brief UART7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART7_Init(void)
{

  /* USER CODE BEGIN UART7_Init 0 */

  /* USER CODE END UART7_Init 0 */

  /* USER CODE BEGIN UART7_Init 1 */

  /* USER CODE END UART7_Init 1 */
  huart7.Instance = UART7;
  huart7.Init.BaudRate = 115200;
  huart7.Init.WordLength = UART_WORDLENGTH_8B;
  huart7.Init.StopBits = UART_STOPBITS_1;
  huart7.Init.Parity = UART_PARITY_NONE;
  huart7.Init.Mode = UART_MODE_TX_RX;
  huart7.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart7.Init.OverSampling = UART_OVERSAMPLING_16;
  huart7.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart7.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart7.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart7) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart7, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart7, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart7) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART7_Init 2 */

  /* USER CODE END UART7_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMAMUX1_OVR_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMAMUX1_OVR_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMAMUX1_OVR_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ADC_BYPASS_GPIO_Port, ADC_BYPASS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DAC0_RST_Pin|DAC1_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ADC_OSR_Pin|DAC1_AMUTEI_Pin|LED_STATUS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DAC0_AMUTEI_GPIO_Port, DAC0_AMUTEI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : ADC_BYPASS_Pin */
  GPIO_InitStruct.Pin = ADC_BYPASS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ADC_BYPASS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DAC0_RST_Pin DAC1_RST_Pin */
  GPIO_InitStruct.Pin = DAC0_RST_Pin|DAC1_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : DAC1_AMUTEO_Pin */
  GPIO_InitStruct.Pin = DAC1_AMUTEO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DAC1_AMUTEO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ADC_OSR_Pin DAC1_AMUTEI_Pin LED_STATUS_Pin */
  GPIO_InitStruct.Pin = ADC_OSR_Pin|DAC1_AMUTEI_Pin|LED_STATUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : DAC0_AMUTEI_Pin */
  GPIO_InitStruct.Pin = DAC0_AMUTEI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DAC0_AMUTEI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DAC0_AMUTEO_Pin */
  GPIO_InitStruct.Pin = DAC0_AMUTEO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DAC0_AMUTEO_GPIO_Port, &GPIO_InitStruct);

  /*AnalogSwitch Config */
  HAL_SYSCFG_AnalogSwitchConfig(SYSCFG_SWITCH_PC3, SYSCFG_SWITCH_PC3_CLOSE);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
 * @brief Print string via UART (blocking)
 */
static void UART_Print(const char *str) {
  HAL_UART_Transmit(&huart7, (uint8_t*)str, strlen(str), 100);
}

/**
 * @brief Print DMA configuration details for verification
 */
static void Print_DMA_Config(void) {
  char buf[128];

  UART_Print("\r\n--- DMA1 Stream 0 Configuration ---\r\n");

  // Get DMA1_Stream0 CR register
  uint32_t cr = DMA1_Stream0->CR;

  // Direction: Bits 7:6 (00=P2M, 01=M2P, 10=M2M)
  uint32_t dir = (cr >> 6) & 0x03;
  sprintf(buf, "Direction: %s (0x%02lX)\r\n",
          dir == 0x01 ? "Memory-to-Peripheral [OK]" :
          dir == 0x00 ? "Peripheral-to-Memory" : "Memory-to-Memory",
          dir);
  UART_Print(buf);

  // Circular mode: Bit 8
  uint32_t circ = (cr >> 8) & 0x01;
  sprintf(buf, "Circular Mode: %s\r\n", circ ? "ENABLED [OK]" : "DISABLED");
  UART_Print(buf);

  // Memory increment: Bit 10
  uint32_t minc = (cr >> 10) & 0x01;
  sprintf(buf, "Memory Increment: %s\r\n", minc ? "ENABLED [OK]" : "DISABLED");
  UART_Print(buf);

  // Peripheral increment: Bit 9
  uint32_t pinc = (cr >> 9) & 0x01;
  sprintf(buf, "Periph Increment: %s\r\n", pinc ? "ENABLED" : "DISABLED [OK]");
  UART_Print(buf);

  // Memory data size: Bits 14:13 (00=Byte, 01=HalfWord, 10=Word)
  uint32_t msize = (cr >> 13) & 0x03;
  sprintf(buf, "Memory Data Size: %s (0x%02lX)\r\n",
          msize == 0x02 ? "32-bit Word [OK]" :
          msize == 0x01 ? "16-bit HalfWord" : "8-bit Byte",
          msize);
  UART_Print(buf);

  // Peripheral data size: Bits 12:11
  uint32_t psize = (cr >> 11) & 0x03;
  sprintf(buf, "Periph Data Size: %s (0x%02lX)\r\n",
          psize == 0x02 ? "32-bit Word [OK]" :
          psize == 0x01 ? "16-bit HalfWord" : "8-bit Byte",
          psize);
  UART_Print(buf);

  // Priority: Bits 17:16 (00=Low, 01=Medium, 10=High, 11=VeryHigh)
  uint32_t pl = (cr >> 16) & 0x03;
  const char* prio_str[] = {"Low", "Medium", "High", "Very High"};
  sprintf(buf, "Priority: %s\r\n", prio_str[pl]);
  UART_Print(buf);

  // Interrupts: TCIE (bit 4), HTIE (bit 3), TEIE (bit 2), DMEIE (bit 1)
  uint32_t tcie = (cr >> 4) & 0x01;
  uint32_t htie = (cr >> 3) & 0x01;
  sprintf(buf, "Interrupts: TCIE=%s, HTIE=%s\r\n",
          tcie ? "ON [OK]" : "OFF", htie ? "ON [OK]" : "OFF");
  UART_Print(buf);

  // DMAMUX Configuration
  UART_Print("\r\n--- DMAMUX1 Channel 0 ---\r\n");
  uint32_t dmamux_ccr = DMAMUX1_Channel0->CCR;
  uint32_t req_id = dmamux_ccr & 0xFF;  // Request ID in bits 7:0
  sprintf(buf, "Request ID: %lu (expected 87 for SAI1_A) %s\r\n",
          req_id, req_id == 87 ? "[OK]" : "[MISMATCH!]");
  UART_Print(buf);

  // Buffer address verification
  UART_Print("\r\n--- Memory Addresses ---\r\n");
  uint32_t buf_addr = (uint32_t)audio_buffer;
  sprintf(buf, "Audio Buffer: 0x%08lX ", buf_addr);
  UART_Print(buf);

  // Check if in RAM_D2 (0x30000000 - 0x30047FFF)
  if (buf_addr >= 0x30000000 && buf_addr < 0x30048000) {
    UART_Print("(RAM_D2) [OK - DMA accessible]\r\n");
  } else if (buf_addr >= 0x20000000 && buf_addr < 0x20020000) {
    UART_Print("(DTCMRAM) [ERROR - NOT DMA accessible!]\r\n");
  } else if (buf_addr >= 0x24000000 && buf_addr < 0x24080000) {
    UART_Print("(AXI SRAM) [WARNING - Use MDMA only]\r\n");
  } else {
    UART_Print("(Unknown region)\r\n");
  }

  // DMA peripheral address (SAI1 data register)
  uint32_t par = DMA1_Stream0->PAR;
  sprintf(buf, "SAI1_A DR: 0x%08lX\r\n", par);
  UART_Print(buf);

  // DMA memory address
  uint32_t m0ar = DMA1_Stream0->M0AR;
  sprintf(buf, "DMA M0AR: 0x%08lX\r\n", m0ar);
  UART_Print(buf);

  // Number of data items
  uint32_t ndtr = DMA1_Stream0->NDTR;
  sprintf(buf, "NDTR (items): %lu\r\n", ndtr);
  UART_Print(buf);

  // Stream enabled?
  uint32_t en = cr & 0x01;
  sprintf(buf, "Stream Enabled: %s\r\n", en ? "YES" : "NO");
  UART_Print(buf);

  UART_Print("------------------------------------\r\n\r\n");
}

/**
 * @brief Fill audio buffer with 1kHz sine wave
 * @param buffer Pointer to buffer (stereo interleaved)
 * @param samples Number of stereo samples to generate
 */
static void Fill_Audio_Buffer(int32_t *buffer, uint32_t samples) {
  const float phase_inc = (2.0f * M_PI * TEST_TONE_FREQ) / AUDIO_SAMPLE_RATE;

  for (uint32_t i = 0; i < samples; i++) {
    // Generate 24-bit sample (left-aligned in 32-bit word)
    // Max value for 24-bit = 0x7FFFFF
    int32_t sample = (int32_t)(sinf(sine_phase) * 0x7FFFFF);

    // Stereo: same signal on both channels
    buffer[i * 2]     = sample;  // Left
    buffer[i * 2 + 1] = sample;  // Right

    // Advance phase
    sine_phase += phase_inc;
    if (sine_phase >= 2.0f * M_PI) {
      sine_phase -= 2.0f * M_PI;
    }
  }
}

/**
 * @brief SAI TX Half Complete callback (first half of buffer sent)
 * Called every 10ms (half of 20ms buffer cycle)
 */
void HAL_SAI_TxHalfCpltCallback(SAI_HandleTypeDef *hsai) {
  if (hsai->Instance == SAI1_Block_A) {
    half_complete_count++;
    // Toggle LED at 50Hz for visual DMA sync (appears as dimmed LED)
    HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_3);
    // Refill first half of buffer
    Fill_Audio_Buffer(&audio_buffer[0], AUDIO_BUFFER_SAMPLES);
  }
}

/**
 * @brief SAI TX Complete callback (second half of buffer sent)
 */
void HAL_SAI_TxCpltCallback(SAI_HandleTypeDef *hsai) {
  if (hsai->Instance == SAI1_Block_A) {
    full_complete_count++;
    // Refill second half of buffer
    Fill_Audio_Buffer(&audio_buffer[AUDIO_BUFFER_SIZE], AUDIO_BUFFER_SAMPLES);
  }
}

/**
 * @brief SAI Error callback
 */
void HAL_SAI_ErrorCallback(SAI_HandleTypeDef *hsai) {
  error_count++;
  last_sai_error = HAL_SAI_GetError(hsai);

  // Immediate debug output
  char err_buf[80];
  sprintf(err_buf, "!!! SAI ERROR: 0x%08lX ", last_sai_error);
  HAL_UART_Transmit(&huart7, (uint8_t*)err_buf, strlen(err_buf), 100);

  // Decode error flags
  if (last_sai_error & 0x01) HAL_UART_Transmit(&huart7, (uint8_t*)"OVR ", 4, 10);
  if (last_sai_error & 0x02) HAL_UART_Transmit(&huart7, (uint8_t*)"UDR ", 4, 10);
  if (last_sai_error & 0x20) HAL_UART_Transmit(&huart7, (uint8_t*)"WCKCFG ", 7, 10);
  if (last_sai_error & 0x80) HAL_UART_Transmit(&huart7, (uint8_t*)"DMA ", 4, 10);
  HAL_UART_Transmit(&huart7, (uint8_t*)"\r\n", 2, 10);
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
