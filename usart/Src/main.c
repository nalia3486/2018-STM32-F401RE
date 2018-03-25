/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

uint8_t komunikat[23];
int buf_end = 0;          //indeks konca zapisu
int buf_start = 0;     // indeks poczatku zapisu
uint8_t buf_size = 255;
uint8_t buf_rx[255];      // bufor kolowy
uint8_t Received;     //zmienna przechowujaca dane odebrane z usarta
uint8_t flaga = 0;   //flaga
uint8_t przerwanie = 0;  // flaga informujaca o uruchomieniu przerwania usarta
uint8_t i = 0, k = 11;
uint8_t data[255];     //buf_rxlica przechowujaca wiadomosc ktora zostanie nadana.
uint16_t size = 0;    //Rozmiar wysylanej wiadomosci
int zapalona, zgaszona, miganie;
uint8_t info = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if(huart->Instance == USART2) {
		przerwanie = 1;
	}
}
void sprawdz() {
	if (Received == (uint8_t) 13) { // jesli zostal wcisniety enter to znaczy ze cala komenda jest wprowadzona
		reset(data, 255);
		if (porownaj(buf_rx,  "LED[ON];", 7)) {
			size = sprintf(data, " -> LED[ON]\n\r");
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
		}else  if (porownaj(buf_rx, "LED[OFF];",8)) {
			size = sprintf(data, " -> LED[OFF]\n\r");
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
		}else if (porownaj(buf_rx, "LED[BLINK,", 9)) {
			i = 10;
			zapalona = odczyt(buf_rx);
			i = k;
			zgaszona = odczyt(buf_rx);
			i = k;
			miganie = odczyt1(buf_rx);

			int temp = buf_start + k;
			if (temp >= buf_size)
				temp = temp - buf_size;

			if (buf_rx[temp] != (uint8_t) ';') {
				size = sprintf(data, " blad: komenda nie rozpoznana\n\r");
			} else {
				size = sprintf(data, " -> Miganie\n\r");
				for (int i = 0; i < (uint16_t) miganie; i++) {
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
					HAL_Delay((uint16_t) zgaszona);
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
					HAL_Delay((uint16_t) zapalona);
				}
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
				info = 0;
			}
		}
		else {
			size = sprintf(data, " blad: komenda nie rozpoznana\n\r");
		}
		HAL_UART_Transmit_IT(&huart2, data, size);// Rozpoczecie nadawania danych z wykorzystaniem przerwan
		buf_start = buf_end; //aktualiacja indeksu od ktorego zacznie sie nowy komunikat
	} else { // jesli enter nie jest wcisniety, nalezy kontynuowac odbieranie kolejnych znakow komendy i zapisywac je do bufora

		buf_rx[buf_end] = Received;   //dodawanie znaku komunikatu do bufora
		size = sprintf(data, &buf_rx[buf_end]);
		HAL_UART_Transmit_IT(&huart2, data, 1); // wysylanie komunikatu na ekran
		buf_end++;     // inkrementacja indeksu konca komunikatu w buforze
		if (buf_end == buf_size)
			buf_end = 0;   // kiedy bufor zostanie zapelniony indeks sie zeruje
	}
}

int porownaj(char *tekst1, char *tekst2, int dl) {
	int pom = buf_start;
	for (int i = 0; i < dl; i++) {
		if (tekst1[pom] != tekst2[i]) {
			return 0;
		}
		pom++;
		if (pom == buf_size) {
			pom = 0;
		}
	}
	return 1;
}

void reset(char *tekst, int dl) {
	for (int i = 0; i < dl; i++)
		tekst[i] = '\0';
}

int8_t potega(int8_t j){
	int8_t wynik=1;
	for (int i=0; i<j; i++){
		wynik=wynik*10;
	}
	return wynik;
}
int odczyt(char *buf_rx) {
	int buf_znak = buf_start + i;
	int temp1 = i + buf_start;
	if (buf_znak >= buf_size)
		buf_znak = buf_znak - buf_size;
	int8_t j = 0;
	int pomoc = 0;
	while (buf_rx[buf_znak] != (uint8_t) ',') {
		j++;
		i++;
		buf_znak++;
		if (buf_znak >= buf_size)
			buf_znak = 0;
	}
	k = i + 1;
	i = i - j;
	j = j - j;
	buf_znak = temp1;
	if (buf_znak >= buf_size)
		buf_znak = buf_znak - buf_size;

	while (j >= 0 && buf_rx[buf_znak] != (uint8_t) ',') {
		pomoc = pomoc + (atoi(&buf_rx[buf_znak]) * potega(j));
		j--;
		i++;
		buf_znak++;
		if (buf_znak >= buf_size)
			buf_znak = 0;
	}
	return pomoc;
}

int odczyt1(char *buf_rx) {
	int buf_znak = buf_start + i;
	int temp1 = i + buf_start;
	if (buf_znak >= buf_size)
		buf_znak = buf_znak - buf_size;
	int8_t j = 0;
	int pomoc = 0;
	while (buf_rx[buf_znak] != (uint8_t) ']') {
		j++;
		i++;
		buf_znak++;
		if (buf_znak >= buf_size)
			buf_znak = 0;
	}
	k = i + 1;
	i = i - j;
	j = j - j;
	buf_znak = temp1;
	if (buf_znak >= buf_size)
		buf_znak = buf_znak - buf_size;

	while (j >= 0 && buf_rx[buf_znak] != (uint8_t) ']') {
		pomoc = pomoc + (atoi(&buf_rx[buf_znak]) *potega(j));
		j--;
		i++;
		buf_znak++;
		if (buf_znak >= buf_size)
			buf_znak = 0;
	}
	return pomoc;
}
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		if (flaga == 0) {
			uint16_t witaj = sprintf(komunikat, "Hello, I am STM32 !!!\n\r");
			HAL_UART_Transmit_IT(&huart2, komunikat, witaj); // Rozpoczecie nadawania danych z wykorzystaniem przerwan
			flaga = 1;
		}
		if (przerwanie == 1) {
			size = 0;
			sprawdz();
			przerwanie = 0;
		}
		HAL_UART_Receive_IT(&huart2, &Received, 1);


  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

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

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
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
void assert_failed(uint8_t* file, uint32_t line)
{
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
