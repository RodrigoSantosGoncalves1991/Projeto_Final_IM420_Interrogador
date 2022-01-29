/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define OPERACAO_SENSORIAMENTO 0x00001
#define OPERACAO_CARACTERIZACAO 0x00002
#define INICIA_VARREDURA 0x00003
#define RESOLUCAO_DAC 4096
#define TAM_MSG 20
#define DELAY_MICROSECONDS 100
#define FLAG_PRIMEIRA_AMOSTRA 9000
#define LIMIAR_POTENCIA_NULA 682
#define TAM_ARRAY_AMOSTRAS 400

#define TAM_X 65
#define TAM_Y 129
//#define DELAY_MICROSECONDS 25
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

DAC_HandleTypeDef hdac1;

UART_HandleTypeDef hlpuart1;

TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;

/* Definitions for calculoComprimentoOndaBragg */
osThreadId_t calculoComprimentoOndaBraggHandle;
const osThreadAttr_t calculoComprimentoOndaBragg_attributes = {
  .name = "calculoComprimentoOndaBragg",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 1024 * 4
};
/* Definitions for caracterizacaoDoEspectro */
osThreadId_t caracterizacaoDoEspectroHandle;
const osThreadAttr_t caracterizacaoDoEspectro_attributes = {
  .name = "caracterizacaoDoEspectro",
  .priority = (osPriority_t) osPriorityHigh1,
  .stack_size = 1024 * 4
};
/* Definitions for transmissaoDeDadosPC */
osThreadId_t transmissaoDeDadosPCHandle;
const osThreadAttr_t transmissaoDeDadosPC_attributes = {
  .name = "transmissaoDeDadosPC",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 1024 * 4
};
/* Definitions for filaVarredura */
osMessageQueueId_t filaVarreduraHandle;
const osMessageQueueAttr_t filaVarredura_attributes = {
  .name = "filaVarredura"
};
/* Definitions for filaValorInterrogado */
osMessageQueueId_t filaValorInterrogadoHandle;
const osMessageQueueAttr_t filaValorInterrogado_attributes = {
  .name = "filaValorInterrogado"
};
/* Definitions for filaCaracterizacao */
osMessageQueueId_t filaCaracterizacaoHandle;
const osMessageQueueAttr_t filaCaracterizacao_attributes = {
  .name = "filaCaracterizacao"
};
/* USER CODE BEGIN PV */
/* Declara o objeto RTOS sinalEventFlag */
osEventFlagsId_t sinalEventFlag;

typedef struct {
  char cSentidoVarredura;
  uint16_t uiIndicePrimeiraAmostra, uiNumAmostrasColetadas, uiArrayAmostras[TAM_ARRAY_AMOSTRAS];
} DADOS_VARREDURA_OBJ_t;

typedef struct {
  char cSentidoVarredura;
  uint16_t uiValorInterrogado;
} VALOR_INTERROGADO_OBJ_t;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_TIM16_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM17_Init(void);
void startCalculoComprimentoOndaBragg(void *argument);
void startCaracterizacaoDoEspectro(void *argument);
void startTransmissaoDeDadosPC(void *argument);

/* USER CODE BEGIN PFP */

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
  MX_LPUART1_UART_Init();
  MX_TIM16_Init();
  MX_ADC1_Init();
  MX_DAC1_Init();
  MX_TIM17_Init();
  /* USER CODE BEGIN 2 */

  //Inicia timer 16
  HAL_TIM_Base_Start_IT(&htim16);


  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of filaVarredura */
  filaVarreduraHandle = osMessageQueueNew (10, sizeof(DADOS_VARREDURA_OBJ_t), &filaVarredura_attributes);

  /* creation of filaValorInterrogado */
  filaValorInterrogadoHandle = osMessageQueueNew (10, sizeof(VALOR_INTERROGADO_OBJ_t), &filaValorInterrogado_attributes);

  /* creation of filaCaracterizacao */
  filaCaracterizacaoHandle = osMessageQueueNew (10, sizeof(DADOS_VARREDURA_OBJ_t), &filaCaracterizacao_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of calculoComprimentoOndaBragg */
  calculoComprimentoOndaBraggHandle = osThreadNew(startCalculoComprimentoOndaBragg, NULL, &calculoComprimentoOndaBragg_attributes);

  /* creation of caracterizacaoDoEspectro */
  caracterizacaoDoEspectroHandle = osThreadNew(startCaracterizacaoDoEspectro, NULL, &caracterizacaoDoEspectro_attributes);

  /* creation of transmissaoDeDadosPC */
  transmissaoDeDadosPCHandle = osThreadNew(startTransmissaoDeDadosPC, NULL, &transmissaoDeDadosPC_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* Inicializa o objeto RTOS Event Flag*/
  sinalEventFlag = osEventFlagsNew(NULL);
  /* USER CODE END RTOS_THREADS */

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_LPUART1|RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_PCLK1;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */
  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config
  */
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_DISABLE;
  sConfig.DAC_DMADoubleDataMode = DISABLE;
  sConfig.DAC_SignedFormat = DISABLE;
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_Trigger2 = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_BOTH;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 1000000;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 17000-1;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 9999;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 170-1;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 65534;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED_AZUL_Pin|LED_VERDE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : uiBotaoProcessamento_Pin uiBotaoCaracterizacao_Pin */
  GPIO_InitStruct.Pin = uiBotaoProcessamento_Pin|uiBotaoCaracterizacao_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_AZUL_Pin LED_VERDE_Pin */
  GPIO_InitStruct.Pin = LED_AZUL_Pin|LED_VERDE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
/* ******************************************************/
/* Nome da ISR: Seleção de operação do interrogador     */
/* Descrição da ISR:   Chamada de interrupção de GPIO   */
/*                     utilizada para selecionar as ta- */
/*                     refas de processamento ou carac- */
/*                     terização.                       */
/* Parâmetros de entrada: GPIO_Pin                      */
/* é o pino detectado pela interrupção externa do micro-*/
/* controlador.                                         */
/* Parâmetros de saída: não tem                         */
/* ******************************************************/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (uiBotaoProcessamento_Pin == GPIO_Pin)
  {
	HAL_GPIO_WritePin(GPIOC, LED_AZUL_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, LED_VERDE_Pin, GPIO_PIN_SET);
	/*Limpa o Event Flag da operação de caracterização e coloca a tarefa startCaracterizacaoDoEspectro para dormir*/
    osEventFlagsClear(sinalEventFlag, OPERACAO_CARACTERIZACAO);
    /*Seta o Event Flag da operação de sensoriamento e acorda a tarefa startCalculoComprimentoOndaBragg*/
	osEventFlagsSet(sinalEventFlag, OPERACAO_SENSORIAMENTO);
  }
  else if (uiBotaoCaracterizacao_Pin == GPIO_Pin)
  {
	HAL_GPIO_WritePin(GPIOC, LED_VERDE_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, LED_AZUL_Pin, GPIO_PIN_SET);
	/*Limpa o Event Flag da operação de sensoriamento e coloca a tarefa startCalculoComprimentoOndaBragg para dormir*/
    osEventFlagsClear(sinalEventFlag, OPERACAO_SENSORIAMENTO);
    /*Seta o Event Flag da operação de caracterização e acorda a tarefa startCaracterizacaoDoEspectro*/
    osEventFlagsSet(sinalEventFlag, OPERACAO_CARACTERIZACAO);
  }
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_startCalculoComprimentoOndaBragg */
/* ******************************************************/
/* Nome da Tarefa: Cálculo do comprimento de onda de    */
/* Bragg												*/
/* Descrição da Tarefa: Recebe dado na Struct           */
/* DADOS_VARREDURA_OBJ_t via message queue, que contém  */
/* as amostras do sinal de potencia refletido, posição  */
/* da primeira amostra, número de amostras e sentido da */
/* varredura. Com esses dados rastreia o ponto de máxima*/
/* potência de reflexão e obtém o comprimento de onda da*/
/* fbg sensora.											*/
/* Parâmetros de entrada: Recebe via fila de mensagens  */
/* filaVarreduraHandle, os dados da rotina de           */
/* interrupção de varredura e os armazena na Struct     */
/* arrayVarredura                                       */
/* Parâmetros de saída: fila de mensagens dada por      */
/* filaValorInterrogadoHandle onde é carregada a Struct */
/* resultadoInterrogacao                                */
/* ******************************************************/
/* USER CODE END Header_startCalculoComprimentoOndaBragg */
void startCalculoComprimentoOndaBragg(void *argument)
{
  /* USER CODE BEGIN 5 */
  /*Declara Struct de dado que irá receber via fila de mensagens os dados da rotina de interrupção de varredura*/
  DADOS_VARREDURA_OBJ_t arrayVarredura;
  /*Declara Struct de dado que irá receber resultado da interrogação de comprimento de onda do Struct de varredura*/
  VALOR_INTERROGADO_OBJ_t resultadoInterrogacao;
  /*Declaração de variáveis auxiliares, uiDataMax será usado como valor máximo de referência e uiValorInterrogado como variável para capturar comprimento de onda máximo*/
  uint16_t uiDataMax, uiValorInterrogado;
  /*Variável que recebe situação da fila de mensagens para verificar se a fila está disponível*/
  osStatus_t status;
  /* Infinite loop */
  for(;;)
  { /*Aguarda a tarefa ser acordada por event flag, que no caso será disparado por interrupção de GPIO do pino PC13*/
    osEventFlagsWait(sinalEventFlag, OPERACAO_SENSORIAMENTO, osFlagsNoClear, osWaitForever);
    /*Carrega status da fila de mensagens e verifica a disponibilidade de elementos na fila*/
    status = osMessageQueueGet(filaVarreduraHandle, &arrayVarredura, NULL, 0U);
    if (osOK == status)
    { /*Inicializa uiDataMax com menor valor possível do comprimento de onda relativo*/
      uiDataMax = 0;
      /*Busca por valor de comprimento de onda relativo, associado a valor de máxima potência de convolução do espectro refletido*/
	  for (uint16_t uiI = 0; arrayVarredura.uiNumAmostrasColetadas > uiI; uiI++)
	  { /*Carrega uiData com valor do uiArrayAmostras na posição uiI*/
	    uint16_t uiData = arrayVarredura.uiArrayAmostras[uiI];
	    /*Verifica se uiData é maior, caso positivo atualiza uiDataMax e salva uiI em uiValorInterrogado*/
	    if(uiDataMax < uiData)
	    {
	      uiDataMax = uiData;
	      uiValorInterrogado = uiI;
	    }
	  }
	  /*Verifica sentido do struct de varredura recebido se crescente a translação para obter o valor do comprimento de onda absoluto é uma soma com o uiIndicePrimeiraAmostra que é o comprimento de onda de referência, no caso da varredura decrescente é uma subtração entre o comprimento de onda de referência e o comprimento de onda relativo*/
	  if ('c' == arrayVarredura.cSentidoVarredura)
	  {
	    uiValorInterrogado = uiValorInterrogado + arrayVarredura.uiIndicePrimeiraAmostra;
	  }
	  else
	  {
	    uiValorInterrogado = arrayVarredura.uiIndicePrimeiraAmostra - uiValorInterrogado;
	  }
	  /*Armazena sentido da varredura na variável Struct VALOR_INTERROGADO_OBJ_t chamada resultadoInterrogacao*/
	  resultadoInterrogacao.cSentidoVarredura = arrayVarredura.cSentidoVarredura;
	  /*Armazena o comprimento de onda absoluto em resultadoInterrogacao*/
	  resultadoInterrogacao.uiValorInterrogado = uiValorInterrogado;
	  /*Armazena a Struct resultadoInterrogacao na fila de mensagens filaValorInterrogadoHandle, que será recebida na tarefa de Transmissão de dados PC*/
	  osMessageQueuePut(filaValorInterrogadoHandle, &resultadoInterrogacao, 0U, 0U);
    }
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_startCaracterizacaoDoEspectro */
/* ******************************************************/
/* Nome da Tarefa: Caracterização do espectro da fbg    */
/* sensora                                              */
/* Descrição da Tarefa: Obtenção do perfil espectral de */
/* potência da fbg sensora através de varredura e       */
/* amostragem do espectro de convolução resultante e    */
/* calculado por meio da deconvolução com as amostras da*/
/* fbg sintonizável e amostragem da convolução          */
/* proveniente do dado na Struct DADOS_VARREDURA_OBJ_t. */
/* Parâmetros de entrada: Recebe via fila de mensagens  */
/* filaVarreduraHandle, os dados da rotina de           */
/* interrupção de varredura e os armazena na variável   */
/* arrayVarredura do tipo Struct DADOS_VARREDURA_OBJ_t  */
/* Parâmetros de saída: fila de mensagens dada por      */
/* filaCaracterizacaoHandle onde é carregada a  variável*/
/* Struct arrayVarredura após receber o espectro da FBG */
/* sensora                                              */
/* ******************************************************/
/* USER CODE END Header_startCaracterizacaoDoEspectro */
void startCaracterizacaoDoEspectro(void *argument)
{
  /* USER CODE BEGIN startCaracterizacaoDoEspectro */
  /*Declara Struct de dado que irá receber via fila de mensagens os dados da rotina de interrupção de varredura*/
  DADOS_VARREDURA_OBJ_t arrayVarredura;
  /*Declara e inicializa o array a seguir com as amostras da FBG sintonizável*/
  float fFbgSintonizavel[TAM_X] = {  1, 1, 2, 2, 3, 4, 5, 6, 8, 10, 13, 16, 19, 23, 27,
	                        		  32, 38, 44, 50, 57, 64, 72, 79, 87, 94, 101, 108, 114,
	                                  119, 123, 126, 127, 128, 127, 126, 123, 119, 114, 108,
	                                  101, 94, 87, 79, 72, 64, 57, 50, 44, 38, 32, 27, 23, 19,
	                                  16, 13, 10, 8, 6, 5, 4, 3, 2, 2, 1, 1
	  	  	  	  	  	              };
  /*inicializa o array da FBG sensora*/
  float fFbgSensora[TAM_X];
  /*inicializa o array do espectro de convolução que irá receber o espectro de convolução de arrayVarredura, mas passará por uma mudança de escala, para ficar na mesma escala de fFbgSintonizavel*/
  float fConvolucaoEspectros[TAM_Y];
  /*Variável que recebe situação da fila de mensagens para verificar se a fila está disponível*/
  osStatus_t status;
  /* Infinite loop */
  for(;;)
  { /*Aguarda a tarefa ser acordada por event flag, que no caso será disparado por interrupção de GPIO do pino PC12*/
	osEventFlagsWait(sinalEventFlag, OPERACAO_CARACTERIZACAO, osFlagsNoClear, osWaitForever);
	/*Carrega status da fila de mensagens e verifica a disponibilidade de elementos na fila*/
	status = osMessageQueueGet(filaVarreduraHandle, &arrayVarredura, NULL, 0U);
	if (osOK == status)
	{ /*Inicializa uiDataMax com menor valor possível do comprimento de onda relativo*/
	  uint16_t uiDataMax = 0;
	  /* uiImax é uma variável para capturar comprimento de onda máximo*/
	  uint16_t uiImax = 0;
	  /*Carrega espectro de convolução da reflexão das FBG's em fConvolucaoEspectros, mas antes efetua sua mudança de escala, para a mesma escala de fFbgSintonizavel*/
	  for (uint8_t uiI = 0; TAM_Y > uiI; uiI++)
	  {
	    uint16_t uiData = arrayVarredura.uiArrayAmostras[uiI];
		fConvolucaoEspectros[uiI] = (float) (uiData - 682.0) * (297462.0 / 3412.0);
		if(uiDataMax < uiData)  /*Busca por valor de comprimento de onda relativo, associado a valor de máxima potência de convolução do espectro refletido*/
		{
		  uiDataMax = uiData;
		  uiImax = uiI;
		}
	  }
	  if ('c' == arrayVarredura.cSentidoVarredura) /*Atualiza índice de primeira amostra para as varreduras crescente e descrescente para ser ajustado em relação a largura de banda do espectro da FBG sensora*/
	  {
	    uint16_t uiImaxLocalVerdadeiro = uiImax + arrayVarredura.uiIndicePrimeiraAmostra;
		arrayVarredura.uiIndicePrimeiraAmostra = uiImaxLocalVerdadeiro - (TAM_X -1 ) / 2;
	  }
	  else
	  {
	    uint16_t uiImaxLocalVerdadeiro = arrayVarredura.uiIndicePrimeiraAmostra - uiImax;
		arrayVarredura.uiIndicePrimeiraAmostra = uiImaxLocalVerdadeiro + (TAM_X -1 ) / 2;
	  }
	  /*No código a seguir entre as linhas 737 e 750 está a implementação do algoritmo de Deconvolução*/
	  uint8_t uiK = 0;
	  fFbgSensora[0] = fConvolucaoEspectros[0] / fFbgSintonizavel[0];
	  while (TAM_Y - TAM_X + 1 > uiK)
	  {
	    uiK++;
		uint8_t uiJ = 1;
		float fRetropropagacao = 0;
		while (uiK + 1 > uiJ)
		{
		  fRetropropagacao = fRetropropagacao + fFbgSensora[uiK - uiJ] * fFbgSintonizavel[uiJ];
		  uiJ++;
		}
		fFbgSensora[uiK] = (fConvolucaoEspectros[uiK] - fRetropropagacao) / fFbgSintonizavel[0];
	   }
	   for (uint16_t uiI = 0; TAM_X > uiI; uiI++)
	   { /*Realiza o casting dos valores float obtidos da fFbgSensora por meio da convolução para o tipo uint16_t*/
	     arrayVarredura.uiArrayAmostras[uiI]	= (uint16_t) fFbgSensora[uiI];
	   }
	   arrayVarredura.uiNumAmostrasColetadas = TAM_X;
	   /*Carrega o valor caracterizado na fila de mensagens filaCaracterizacaoHandle depois de colocar em arrayVarredura*/
	   osMessageQueuePut(filaCaracterizacaoHandle, &arrayVarredura, 0U, 0U);
	}
    osDelay(1);
  }
  /* USER CODE END startCaracterizacaoDoEspectro */
}

/* USER CODE BEGIN Header_startTransmissaoDeDadosPC */
/* ******************************************************/
/* Nome da Tarefa: Transmissão de dados PC              */
/* Descrição da Tarefa: Envio dos dados de deslocamento */
/* de comprimento de onda e espectro caracterizado      */
/* obtidos através das tarefas do interrogador para     */
/* exibição em computador.                              */
/* Parâmetros de entrada: recebe dados de duas filas de */
/* mensagens distintas filaValorInterrogadoHandle e     */
/* filaCaracterizacaoHandle.                            */
/* Parâmetros de saída: Envia dados pela UART para      */
/* outro dispositivo.                                   */
/* ******************************************************/
/* USER CODE END Header_startTransmissaoDeDadosPC */
void startTransmissaoDeDadosPC(void *argument)
{
  /* USER CODE BEGIN startTransmissaoDeDadosPC */
  /* Inicializa as variáveis structs que irão receber os dados das filas de mensagem*/
  DADOS_VARREDURA_OBJ_t arrayCaracterizacao;
  VALOR_INTERROGADO_OBJ_t resultadoInterrogacao;
  /* Variáveis do tipo caractere que irão ser utilizadas para transmisão dos dados via uart*/
  char cMsg[TAM_MSG], cOperacao, cFimPacote;
  osStatus_t status;
  /* Infinite loop */
  for(;;)
  { /*Captura qual tarefa está acordada e seleciona qual rotina de transmissão irá utilizar para enviar os dados*/
    uint8_t uiFlags = osEventFlagsGet(sinalEventFlag);
	if (OPERACAO_SENSORIAMENTO == uiFlags)
	{ /*Transmissão dos dados da tarefa de Cálculo do comprimento de onda de Bragg*/
	  status = osMessageQueueGet(filaValorInterrogadoHandle, &resultadoInterrogacao, NULL, 0U);
	  if (osOK == status)
	  {
	    cOperacao = 'S';
	    cFimPacote = '@';
	    sprintf(cMsg, "%c-%hu\n", cOperacao, resultadoInterrogacao.uiValorInterrogado);
	    HAL_UART_Transmit(&hlpuart1, (uint8_t *)cMsg, strlen(cMsg), HAL_MAX_DELAY);
	  	sprintf(cMsg, "%c-%hu\n", cFimPacote, 0x0000);
	  	HAL_UART_Transmit(&hlpuart1, (uint8_t *)cMsg, strlen(cMsg), HAL_MAX_DELAY);
	  }
	}
	else if (OPERACAO_CARACTERIZACAO == uiFlags)
	{ /*Transmissão dos dados da tarefa de caracterização da FBG sensora*/
	  status = osMessageQueueGet(filaCaracterizacaoHandle, &arrayCaracterizacao, NULL, 0U);
	  if (osOK == status)
	  {
	    cOperacao = 'C';
	    cFimPacote = '@';
	    for(uint8_t uiI = 0; arrayCaracterizacao.uiNumAmostrasColetadas > uiI; uiI++)
	    {
	      uint16_t uiData = arrayCaracterizacao.uiArrayAmostras[uiI];
	      if ('c' == arrayCaracterizacao.cSentidoVarredura)
	      {
	        uint16_t uiIndiceLocalVerdadeiro = uiI + arrayCaracterizacao.uiIndicePrimeiraAmostra;
	    	sprintf(cMsg, "%c-%hu-%hu\n", cOperacao, uiData, uiIndiceLocalVerdadeiro);
	      }
	      else
	      {
	    	uint16_t uiIndiceLocalVerdadeiro = arrayCaracterizacao.uiIndicePrimeiraAmostra - uiI;
	    	sprintf(cMsg, "%c-%hu-%hu\n", cOperacao, uiData, uiIndiceLocalVerdadeiro);
	      }
	      HAL_UART_Transmit(&hlpuart1, (uint8_t *)cMsg, strlen(cMsg), HAL_MAX_DELAY);
	    }
	    sprintf(cMsg, "%c-%hu-%hu\n", cFimPacote, 0x0000, 0x0000);
	    HAL_UART_Transmit(&hlpuart1, (uint8_t *)cMsg, strlen(cMsg), HAL_MAX_DELAY);
	    }
	}
	osDelay(1);
  }
  /* USER CODE END startTransmissaoDeDadosPC */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
/* ******************************************************/
/* Nome da ISR: Varredura Conversão A/D e D/A           */
/* Descrição da ISR:    Rotina de interrupção onde o    */
/*                      conversor D/A move o espectro   */
/*                      da fbg sintonizável e o conver- */
/*                      sor A/D captura os valores de   */
/*                      variação do espectro de potência*/
/*                      refletida pelas duas fbgs.      */
/* Parâmetros de entrada: não tem                       */
/*                                                      */
/* Parâmetros de saída: fila de mensagens dada por      */
/* filaVarreduraHandle onde são carregadas as Structs   */
/* arrayVarreduraCrescente e arrayVarreduraDecrescente  */
/* ******************************************************/
/* USER CODE END Header_StartVarreduraConversaoADeDA */
  if (&htim16 == htim)
  {
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	/*Declara variável auxiliar de aquisição de potência do sinal e inicializa contadores de número de amostras não nulas*/
	uint16_t uiPotenciaSinal, uiContCresc = 0, uiContDecresc = 0;
	/*Declara Structs dos dados de varredura crescente e decrescente*/
	DADOS_VARREDURA_OBJ_t arrayVarreduraCrescente, arrayVarreduraDecrescente;
	/*Inicializa o dado de indice da primeira amostra com um valor fixo acima dos existentes na varredura*/
	arrayVarreduraCrescente.uiIndicePrimeiraAmostra = FLAG_PRIMEIRA_AMOSTRA;
	arrayVarreduraDecrescente.uiIndicePrimeiraAmostra = FLAG_PRIMEIRA_AMOSTRA;
	/*Inicializa o dado de sentido da varredura com os caracteres que indicam com qual sentido da varredura a Struct irá trabalhar*/
	arrayVarreduraCrescente.cSentidoVarredura = 'c';
	arrayVarreduraDecrescente.cSentidoVarredura = 'd';
	/*Inicializa o Timer 17 */
	HAL_TIM_Base_Start(&htim17);
	/*Inicializa o módulo DAC*/
	HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
	/*Loop da varredura e inicialização das variáveis de controle, uiI conta de 0 até 8191*/
	for (uint16_t uiI = 0, uiK = 0, uiM = 4095; 2 * (RESOLUCAO_DAC) > uiI; uiI++)
    { /*Variável de controle uiI do loop no if abaixo verifica se a varredura ainda é crescente, caso maior que 4095 inicia a varredura decrescente*/
	  if (RESOLUCAO_DAC > uiI)
	  {/*Carrega o valor númerico da variável contadora uiK no conversor DA, observe que temos uma progressão em rampa crescente nos valores de uiK*/
	    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, uiK);
	    /*Atraso implementado a partir do Timer 17 sendo utilizado como contador, prescaler configurado para cada contagem durar 1us, utilizado para aguardar resposta do arduino*/
	    __HAL_TIM_SET_COUNTER(&htim17,0);
	    while (__HAL_TIM_GET_COUNTER(&htim17) < DELAY_MICROSECONDS);
	    /*Inicializa conversor AD*/
	    HAL_ADC_Start(&hadc1);
	    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	  	/*Captura amostra do conversor AD*/
	  	uiPotenciaSinal = HAL_ADC_GetValue(&hadc1);
	  	/*É verificado se o valor de potência do sinal é maior do que 682 ou seja se é não nulo, pois DAC do arduino DUE tem offset de 0,55V e se uiArrayAmostras atingiu seu valor máximo*/
	    if (LIMIAR_POTENCIA_NULA < uiPotenciaSinal && TAM_ARRAY_AMOSTRAS > uiContCresc)
	  	{ /*Verifica flag de primeira amostra, se condição verdadeira armazena índice da primeira amostra não nula*/
	  	  if (FLAG_PRIMEIRA_AMOSTRA == arrayVarreduraCrescente.uiIndicePrimeiraAmostra)
	  	  {
	  	    arrayVarreduraCrescente.uiIndicePrimeiraAmostra = uiK;
	  	  }
	  	  /*Amostras de potencia não nulas são armazenadas no array do Struct */
	  	  arrayVarreduraCrescente.uiArrayAmostras[uiContCresc] = uiPotenciaSinal;
	  	  /*Variável contadora de amostras não nulas da varredura crescente é incrementada*/
	  	  uiContCresc++;
	  	}
	    uiK++;
	  }
	  else
	  { /*Varredura descrescente, código funciona de maneira análoga ao bloco anterior com a distinção que uiM é decrementado*/
	    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, uiM);

	  	__HAL_TIM_SET_COUNTER(&htim17,0);
	  	while (__HAL_TIM_GET_COUNTER(&htim17) < DELAY_MICROSECONDS);

	    HAL_ADC_Start(&hadc1);
	  	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	  	uiPotenciaSinal = HAL_ADC_GetValue(&hadc1);
	  	if (LIMIAR_POTENCIA_NULA < uiPotenciaSinal && TAM_ARRAY_AMOSTRAS > uiContDecresc)
	  	{
	  	  if (FLAG_PRIMEIRA_AMOSTRA == arrayVarreduraDecrescente.uiIndicePrimeiraAmostra)
	  	  {
	  		arrayVarreduraDecrescente.uiIndicePrimeiraAmostra = uiM;
	  	  }
	  	  arrayVarreduraDecrescente.uiArrayAmostras[uiContDecresc] = uiPotenciaSinal;
	  	  uiContDecresc++;
	  	}
	    uiM--;
	  }

    } /*Structs das varreduras no sentido crescente e decrescente recebem a contagem de amostras coletas em cada varredura*/
	arrayVarreduraCrescente.uiNumAmostrasColetadas = uiContCresc;
	arrayVarreduraDecrescente.uiNumAmostrasColetadas = uiContDecresc;
	/*Fila de mensagem da varredura recebe dois structs, sendo as varreduras nos sentido crescente e decrescente*/
	osMessageQueuePut(filaVarreduraHandle, &arrayVarreduraCrescente, 0U, 0U);
	osMessageQueuePut(filaVarreduraHandle, &arrayVarreduraDecrescente, 0U, 0U);
  }
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7) {
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
