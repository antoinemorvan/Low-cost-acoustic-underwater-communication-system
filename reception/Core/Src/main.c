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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "stdint.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_BUF_LEN 4096
#define MSG_BUF_LEN 140	//264
#define THRESHOLD 4
#define ZERO_LEN 100
#define FILTER_SIZE 7
#define NB_MAX_MSG 16

#define FC 40000 // Carrier frequency in Hz
#define DUREE 100e-6 // duration in seconds
#define NB_SAMPLES 140 // number of point for a 100 microseconds signal
#define SYMBOL_LEN 35 // number of sample needed to represent a full symbol
#define NB_SYM_SEQ 8 // number of maximal symbol in an half of a 4096 buffer for 8-bit communication at 40kHz

#define NB_SYM 4
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint16_t adc_buf[ADC_BUF_LEN];
uint16_t msg_buf[MSG_BUF_LEN];
uint16_t msg_buf2[MSG_BUF_LEN];
uint16_t index_msg[NB_MAX_MSG];
uint16_t msgs_buffers[NB_SYM_SEQ][MSG_BUF_LEN];


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//float trainingseq[] = {992, 992, 992, 992, 992, 992, 992, 992, 992, 1985, 1985, 1985, 1985, 1985, 1985, 1985, 1985, 992, 992, 992, 992, 992, 992, 992, 992, 0, 0, 0, 0, 0, 0, 0, 0, 992, 992, 992, 992, 992, 992, 992, 992, 992, 1985, 1985, 1985, 1985, 1985, 1985, 1985, 1985, 992, 992, 992, 992, 992, 992, 992, 992, 0, 0, 0, 0, 0, 0, 0, 0};
float trainingseq[] = {1985, 1985, 1985, 1985, 992, 992, 992, 992, 992, 992, 992, 992, 992, 992, 992, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 992, 992, 992, 992, 992, 992, 992, 992, 992, 992, 992, 992, 1985, 1985, 1985, 1985, 1985, 1985, 1985, 1985, 1985, 1985, 1985, 992, 992, 992, 992, 992, 992, 992, 992, 992, 992, 0, 0, 0, 0, 0, 0, 0, 0, 992, 992, 992, 992, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 992, 992, 992, 992, 992, 992, 992, 992, 992, 992, 1985, 1985, 1985, 1985, 1985, 1985, 1985, 1985, 0, 0, 0, 0, 992, 992, 992, 992, 992, 992, 992, 992, 992, 992, 992, 1985, 1985, 1985, 1985, 1985, 1985, 1985, 1985, 1985, 1985, 992, 992, 992, 992, 992, 992, 992, 992};
int traininglen = 132;
int correla=0;

float Tmsg[NB_SAMPLES]; // Time list
float Lsin[NB_SAMPLES]; // List of cosinus values for a 100 microseconds signal
float Lcos[NB_SAMPLES];
float I[NB_SAMPLES];
float Q[NB_SAMPLES];
float tI[NB_SYM_SEQ][NB_SAMPLES];
float tQ[NB_SYM_SEQ][NB_SAMPLES];
float IntegI[NB_SYM];
float IntegQ[NB_SYM];
float tIntegI[NB_SYM_SEQ][NB_SYM];
float tIntegQ[NB_SYM_SEQ][NB_SYM];
char msg_halfbuffer[NB_SYM_SEQ][2*NB_SYM]; // place where we will store the decoded messages



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  float interval = DUREE / (NB_SAMPLES - 1);

  for (int i = 0; i < NB_SAMPLES; i++) {
          Tmsg[i] = i * interval;
          Lsin[i] = sin(2 * M_PI * FC * Tmsg[i]);
          Lcos[i] = cos(2 * M_PI * FC * Tmsg[i]);
      } // à changer avec des listes de valeurs dejà determinées faites sur python (pb sur la flexibilité du code)


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
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, ADC_BUF_LEN);
  /* USER CODE END 2 */

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

//void find_training_sequence_and_copy(uint16_t *buffer, int buffer_length, float *training_sequence, int training_length, uint16_t *new_buffer, int new_buffer_length) {
//    int max_corr = 0;
//    int index = -1;
//
//    for (int i = 0; i <= buffer_length - training_length; i++) {
//        double corr = 0;
//        for (int j = 0; j < training_length; j++) {
////            float normalized_adc_value = (buffer[i + j] / 4096.0) * 3.3; // Normalise la valeur ADC à [0, 3.3]
////            corr += (normalized_adc_value * training_sequence[j]);
//            corr += (buffer[i +j] * training_sequence[j]);
//
//        }
//
//        if (corr > max_corr && corr > THRESHOLD) {
//            max_corr = corr;
//            index = i;
//        }
//    }
//
//    if (index != -1 && index + new_buffer_length <= buffer_length) {
//        for (int k = 0; k < new_buffer_length; k++) {
//            new_buffer[k] = buffer[index + k];
//        }
//    }
//    correla = max_corr ;
//}
//
//void find_training_sequence_and_copy2(uint16_t *buffer, int buffer_length, float *training_sequence, int training_length, uint16_t *new_buffer, int new_buffer_length) {
//    int min_corr = 100000000;
//    int index = -1;
//
//    for (int i = 0; i <= buffer_length - training_length; i++) {
//        double corr = 0;
//        for (int j = 0; j < training_length; j++) {
////            float normalized_adc_value = (buffer[i + j] / 4096.0) * 3.3; // Normalise la valeur ADC à [0, 3.3]
////            corr += (normalized_adc_value * training_sequence[j]);
//            corr += (buffer[i +j] - training_sequence[j])*(buffer[i +j] - training_sequence[j]);
//
//        }
//
//        if (corr < min_corr) {
//            min_corr = corr;
//            index = i;
//        }
//    }
//
//    if (index != -1 && index + new_buffer_length <= buffer_length) {
//        for (int k = 0; k < new_buffer_length; k++) {
//            new_buffer[k] = buffer[index + k];
//        }
//    }
//    correla = min_corr ;
//}
//
//void calcul_correl(uint16_t *buffer, int buffer_length, float *training_seq, int training_length, double *correlation){
//	for (int i=0; i <= buffer_length-training_length;i++){
//		double sum = 0;
//		for (int j=0; j < training_length; j++){
//			sum += buffer[i+j]*training_seq[j];
//		}
//		correlation[i]=sum;
//	}
//}

//void find_training_sequence(uint16_t *buffer, int buffer_length, float *training_seq, int training_length, uint16_t *new_buffer, int new_buffer_length){
//	double correlation[buffer_length-training_length + 1];
//	calcul_correl(buffer, buffer_length, training_seq, training_length, correlation);
//
//	double max_correlation = correlation[0];
//	int max_correlation_index = 0;
//	for (int i=1; i <= buffer_length-training_length; i++){
//		if (correlation[i] > max_correlation){
//			max_correlation = correlation[i];
//			max_correlation_index = i;
//		}
//	}
//	correla = max_correlation;
//
//	if (max_correlation_index != -1 && max_correlation_index + new_buffer_length <= buffer_length) {
//	        for (int k = 0; k < new_buffer_length; k++) {
//	            new_buffer[k] = buffer[max_correlation_index + k];
//	        }
//	    }

//	if (max_correlation > THRESHOLD){
//
//		return max_correlation_index;
//	}
//	else {
//		return -1;
//	}
//}

void find_zero(uint16_t *buffer, int buffer_length, uint16_t *new_buffer, int new_buffer_length, int indxstart){
	int zero_cnt = 0;
	int index = 0;
	int msg_cnt = 0;

	for (int i=0 + indxstart; i < buffer_length + indxstart; i++){ // Due to the ping-pong buffering method we only use half of the ADC buffer
		if (buffer[i] < 200){
			zero_cnt ++;
			if (zero_cnt > ZERO_LEN){ // if the sequence of zeros is long enough
				index = i - zero_cnt + 1; // find beginning of the sequence
				index_msg[msg_cnt]=index; // record the index of the beginning of this sequence
				msg_cnt ++; // add 1 to the message counter
				zero_cnt = 0; // reset the counter of zeros
				i = index+264; // advance to the next message since we know where the message begin and where its length
				if (i >= buffer_length + indxstart)
					i = buffer_length;
				if (index + new_buffer_length <= buffer_length + indxstart && msg_cnt < 8) { // copies the message in the msg_buffer
					for (int k = 0; k < new_buffer_length; k++) {
						msgs_buffers[msg_cnt][k] = buffer[index + 132 + k];
						}
					}
			}
		}
		else {
			zero_cnt = 0;
		}
	}
	correla = index;

	if (index + new_buffer_length <= buffer_length + indxstart) { // copies the message in the msg_buffer
		for (int k = 0; k < new_buffer_length; k++) {
			new_buffer[k] = buffer[index + 132 + k];

		}
	}
}

double trapz(double *y, double dx, int n) { // trapeze method for integration with dx=100/132=0,757 microseconds (time gap between two points)
    double integ = 0.0;
    for (int i = 0; i < n - 1; i++) {
        integ += (y[i] + y[i+1]) * dx / 2.0;
    }
    return integ;
}

void integ(float *IQ, float *integIQ){
	int cpt=0;
	for (int i = 0; i < MSG_BUF_LEN; i += SYMBOL_LEN) {
		double integrale = 0;
	    double sing_symb[33];
	    for (int j=0; j<33; j++){
	    	sing_symb[j]=IQ[i+j];
	    }
	    integrale = trapz(sing_symb, 0.757e-6, SYMBOL_LEN);
	    integIQ[cpt] = integrale;
	    cpt += 1;
	}
}

void  decision(){ // decision block
	for (int k=1; k < NB_SYM_SEQ; k++){
		char sym_dec[3];
		char msg_dec[2*NB_SYM];
		for (int l = 0; l < NB_SYM; l++){
			if (tIntegI[k][l] < 0 && tIntegQ[k][l] < 0)
				strcpy(sym_dec,"00");
			if (tIntegI[k][l] < 0 && tIntegQ[k][l] > 0)
				strcpy(sym_dec, "10");
			if (tIntegI[k][l] > 0 && tIntegQ[k][l] > 0)
				strcpy(sym_dec, "11");
			if (tIntegI[k][l] > 0 && tIntegQ[k][l] < 0)
				strcpy(sym_dec, "01");
			if (l > 0)
				strcat(msg_dec, sym_dec);
			else
				strcpy(msg_dec, sym_dec);
		}
		strcpy(msg_halfbuffer[k],msg_dec);
	}
}
void demod_sig(uint16_t *buffer, int buffer_length, uint16_t *new_buffer, int new_buffer_length, int indxstart){

	find_zero(buffer, buffer_length, new_buffer, new_buffer_length, indxstart);
	for (int i=0; i < new_buffer_length; i++){
			I[i]= (new_buffer[i]-1000)*Lcos[i]; // In-Phase signal with a 0V centered signal (may not be necessary given the signal)
			Q[i]= (new_buffer[i]-1000)*Lsin[i]; // Quadrature signal with a 0V centered signal
		}
	integ(I, IntegI);
	integ(Q, IntegQ);

}

void demod_sig2(uint16_t *buffer, int buffer_length, uint16_t *new_buffer, int new_buffer_length, int indxstart){

	find_zero(buffer, buffer_length, new_buffer, new_buffer_length, indxstart);
	for (int k=1; k < NB_SYM_SEQ; k++){
		for (int i=0; i < new_buffer_length; i++){
					tI[k][i]= (msgs_buffers[k][i]-1300)*Lcos[i]; // In-Phase signal with a 0V centered signal (may not be necessary given the signal)
					tQ[k][i]= (msgs_buffers[k][i]-1300)*Lsin[i]; // Quadrature signal with a 0V centered signal
				}
		integ(tI[k], tIntegI[k]);
		integ(tQ[k], tIntegQ[k]);
	}
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
//	find_training_sequence_and_copy2(adc_buf, ADC_BUF_LEN, trainingseq, traininglen, msg_buf, MSG_BUF_LEN);
//	find_training_sequence(adc_buf, ADC_BUF_LEN, trainingseq, traininglen, msg_buf, MSG_BUF_LEN);
//	find_zero(adc_buf, ADC_BUF_LEN, msg_buf, MSG_BUF_LEN);
	demod_sig2(adc_buf, ADC_BUF_LEN/2, msg_buf, MSG_BUF_LEN, 0); // fonctionne sur tout le buffer => à changer
	decision();
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
//	find_training_sequence_and_copy2(adc_buf, ADC_BUF_LEN,trainingseq, traininglen, msg_buf, MSG_BUF_LEN);
//	find_training_sequence(adc_buf, ADC_BUF_LEN, trainingseq, traininglen, msg_buf, MSG_BUF_LEN);
//	find_zero(adc_buf, ADC_BUF_LEN, msg_buf, MSG_BUF_LEN);
	demod_sig2(adc_buf, ADC_BUF_LEN/2, msg_buf, MSG_BUF_LEN, ADC_BUF_LEN/2);
	decision();
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
