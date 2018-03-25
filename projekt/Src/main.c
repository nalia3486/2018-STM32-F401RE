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
#include "tm_stm32_delay.h"
#include "defines.h"
#include "SSD1306_procedury.h"
#include "Obrazki_128_64.h"
#include "function.h"

#include "bmp180.h"
#include <math.h>
#include <string.h>

#define buf_size 255 //rozmiar bufora ko³owego
#define start 0x3A //adres pocz¹tku ramki
#define dest_addr 0x55 //adres docelowy
#define sour_addr 0x7F //adres Ÿród³owy
#define end1 0x0D //znak koñca ramki
#define end2 0x0A //znak koñca ramki

#define wyswietl_temp 0x22 //komenda wyœwietlania temperatury
#define wyswietl_pressure 0x23 //komenda wyœwietlania temperatury

const int led_on_off = 0x11; // komenda led on/led off
const int miganie = 0x10; //miganie dioda

int flaga; //flaga wykorzystywana do wyœwietlania temperatury co okreœlony czas (SysTick)
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

uint16_t PomiarADC; //zmienna przechowuje pomiar
float Temperature;
float Vsense;

const float V25 = 0.76; // [Volts]
const float Avg_slope = 0.0025; //[Volts/degree]
const float SupplyVoltage = 3.3; // [Volts]
const float ADCResolution = 4095.0;

int wartoscADC1 = 0;
int wartoscADC2 = 0;
char TempCelsiusDisplay[] = "    .   C "; //inicjalizacja bufora wyœwietlania
char PressureDisplay[] = "     hPa "; //przykladowa wartosc

uint8_t Received;     //zmienna przechowujaca dane odebrane z usarta
uint8_t przerwanie = 0;  // flaga informujaca o uruchomieniu przerwania usarta
uint8_t buf_rx[buf_size]; //bufor ko³owy
uint8_t buf_end = 0; //indeks koñca zapisu
uint8_t buf_start = 0; //indeks poczatku zapisu
uint8_t CMD_buf_rx[buf_size]; //bufor pomocniczy
uint8_t CMD_BUF_INDEX = 0; // indeks bufora
uint16_t zapalona; //jak dlugo dioda zapalona
uint16_t zgaszona; //jak dlugo dioda zgaszona

uint8_t buf_tx[buf_size]; //bufor ko³owy
uint8_t buf_xx[buf_size];
uint8_t buf_tx_end = 0; //indeks koñca zapisu
uint8_t buf_tx_start = 0; //indeks poczatku zapisu
uint8_t tx_transmit = 0;

uint8_t data[buf_size];
uint8_t flag = 0;

float volatile bmp180Pressure;
float volatile bmp180Temperature;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C3_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
	PomiarADC = HAL_ADC_GetValue(&hadc1); // Pobranie zmierzonej wartosci
	Vsense = (SupplyVoltage * PomiarADC) / ADCResolution; // Przeliczenie wartosci zmierzonej na napiecie
	Temperature = ((Vsense - V25) / Avg_slope) + 25; // Obliczenie temperatury
	wartoscADC1 = (int) Temperature; //temperatura, wartosc przed przecinkiem
	wartoscADC2 = ((int) (Temperature * 100)) % 100; //temperatura, wartosc po przecinku
}

//metoda wywolywana po odborze
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART2) {
		buf_end++;
		if (buf_end >= buf_size) {
			buf_end = 0;
		}
		HAL_UART_Receive_IT(&huart2, &buf_rx[buf_end], 1);
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART2) {
		//przerwanie wywolywane po wyslaniu znaku
		if (buf_tx_end != buf_tx_start) {
			int ile1 = 0;
			while (buf_tx_end != buf_tx_start) { /*dopóki w buforze jest coœ do wyslania,
				czego jeszcze nie wys³aliœmy, bêdziemy to dodawac do tablicy, a pozniej zostana wyslane*/
				buf_xx[ile1++] = buf_tx[buf_tx_end++]; //zapisanie do bufora pomocniczego
				if (buf_tx_end >= buf_size) {
					buf_tx_end = 0;
				}
			}
			HAL_UART_Transmit_IT(&huart2, buf_xx, ile1);
		} else {
			tx_transmit = 0; //jesli nie ma juz nic do wyslania
		}
	}
}
void sprawdz(void) { //funkcja, która sprawdza czy zosta³a przes³ana odpowiednia ramka
	if (flag == 5) { //czy odebrano znak koñca ramki
		uint16_t dlugosc = CMD_buf_rx[3]; //wyliczenie dlugosci ramki
		uint16_t crc = crc1(&CMD_buf_rx[1], dlugosc + 2); //suma kontrolna
		if (crc >= 0) {
			if (CMD_BUF_INDEX == dlugosc + 10) { //czy w buforze znajduje sie ramka o odpowiedniej dlugosci
				dioda(); //sprawdzenie czy komenda dioda
				if (CMD_buf_rx[4] == wyswietl_temp) { //czy komenda wyswietlenia temperatury
					send(&TempCelsiusDisplay, 10);
				}
				if (CMD_buf_rx[4] == wyswietl_pressure) { //czy komenda wyswietlenia cisnienia
					send(&PressureDisplay, 9);
				}
				flag = 0;
				CMD_BUF_INDEX = 0;
			}
		}
	}
}
void copy2(void) {
	buf_start++;
	if (buf_start >= buf_size) {
		buf_start = 0;
	}
}
void copy(void) {
	if (buf_start != buf_end) {
		if (flag == 1) {
			if (buf_rx[buf_start] == dest_addr) { //czy adres docelowy urzadzenia
				CMD_buf_rx[CMD_BUF_INDEX++] = buf_rx[buf_start];
				flag = 2;
			} else {
				CMD_BUF_INDEX = 0;
				flag = 0;
			}
		} else if (flag == 2) {
			if (buf_rx[buf_start] == sour_addr) { //czy adres Ÿród³owy
				CMD_buf_rx[CMD_BUF_INDEX++] = buf_rx[buf_start];
				flag = 3;
			} else {
				CMD_BUF_INDEX = 0;
				flag = 0;
			}
		} else if (flag == 3) {
			if (buf_rx[buf_start] == end1) { //czy znak koñca ramki (pierwszy z dwóch)
				CMD_buf_rx[CMD_BUF_INDEX++] = buf_rx[buf_start];
				flag = 4;
			}
			if (buf_rx[buf_start] != start) {
				CMD_buf_rx[CMD_BUF_INDEX++] = buf_rx[buf_start];
			} else {
				CMD_BUF_INDEX = 0;
				flag = 0;
			}
		} else if (flag == 4) {
			if (buf_rx[buf_start] == end2) { //czy znak koñca ramki
				CMD_buf_rx[CMD_BUF_INDEX++] = buf_rx[buf_start];
				flag = 5;
				sprawdz(); //sprawdzam czy cala ramka zostala przeslana i czy jest ona poprawna
			} else {
				flag = 0;
				CMD_BUF_INDEX = 0;
			}
		}
		if (buf_rx[buf_start] == start) { //czy pocz¹tek ramki
			if (flag == 0) {
				CMD_buf_rx[CMD_BUF_INDEX++] = buf_rx[buf_start]; //zapisanie do bufora pomocniczego
				flag = 1; //ustawiam flage na 1, co oznacza, ze teraz powinien przyjsc destination address
			}
		}
		copy2();
	}
}
void dioda(void) {
	if (CMD_buf_rx[4] == led_on_off) { //czy komenda w³¹czenia/wy³¹czenia diody
		if (CMD_buf_rx[5] == 0x01) { //warunek na zapalenie diody
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
		} else {
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
		}
	}
	if (CMD_buf_rx[4] == miganie) { //warunek na miganie diody
		zapalona = ((uint16_t) CMD_buf_rx[6] << 8) | CMD_buf_rx[7]; //ile czasu w³¹czona
		zgaszona = ((uint16_t) CMD_buf_rx[8] << 8) | CMD_buf_rx[9]; //ile czasu wy³¹czona
		for (int i = 0; i < CMD_buf_rx[5]; i++) {
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
			HAL_Delay(zapalona);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
			HAL_Delay(zgaszona);
		}
	}
}
void send(char *dane, uint16_t rozmiar) {
	//dodawanie danych do bufora
	for (int x = 0; x < rozmiar; x++) {
		buf_tx[buf_tx_start++] = dane[x];
		if (buf_tx_start >= buf_size) {
			buf_tx_start = 0;
		}
	}
	int ile;
	__disable_irq(); // zablokowanie przerwania
	if (tx_transmit == 0){
		tx_transmit = 1;
		ile = 0;
		while (buf_tx_start != buf_tx_end) {
			buf_xx[ile++] = buf_tx[buf_tx_end++]; //zapis do bufora pomocniczego
			if (buf_tx_end >= buf_size) {
				buf_tx_end = 0;
			}
		}
		HAL_UART_Transmit_IT(&huart2, buf_xx, ile);
	}
	__enable_irq(); //odblokowanie przerwania
}
void wyswietlTemp(void) {
	if (wartoscADC1 < 0) { //jesli temperatura ujemna
		TempCelsiusDisplay[0] = '-';
	} else if (wartoscADC1 > 0) { //jesli temperatura dodatnia
		TempCelsiusDisplay[0] = '+';
	} //konwersja ascii
	  //cyfry po przecinku
	TempCelsiusDisplay[5] = wartoscADC2 / 10 + 48;
	TempCelsiusDisplay[6] = wartoscADC2 % 10 + 48;
	if (wartoscADC1 < 10 && wartoscADC1 > -10) { //gdy liczba jednocyfrowa
		TempCelsiusDisplay[3] = wartoscADC1 + 48;
	} else if (wartoscADC1 < 100 && wartoscADC1 > -100) { //gdy dwucyfrowa
		TempCelsiusDisplay[2] = wartoscADC1 / 10 + 48;
		TempCelsiusDisplay[3] = wartoscADC1 % 10 + 48;
	} else { //trzycyfrowa
		TempCelsiusDisplay[1] = wartoscADC1 / 100 + 48;
		TempCelsiusDisplay[2] = (wartoscADC1 / 10) % 10 + 48;
		TempCelsiusDisplay[3] = wartoscADC1 % 10 + 48;
	}

}
void wyswietlPressure(void) {
	PressureDisplay[0] = ((int) bmp180Pressure / 1000) + 48;
	PressureDisplay[1] = ((int) bmp180Pressure / 100) % 10 + 48;
	PressureDisplay[2] = ((int) bmp180Pressure % 100) % 10 + 48;
	PressureDisplay[3] = ((int) bmp180Pressure) % 10 + 48;
}
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void) {

	/* USER CODE BEGIN 1 */
	char status = FALSE;
	//numer_ekranu = 0;
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
	MX_I2C3_Init();
	MX_ADC1_Init();
	MX_USART2_UART_Init();
	MX_I2C1_Init();

	/* USER CODE BEGIN 2 */
	TM_DELAY_Init();
	HAL_ADC_Start_IT(&hadc1);
	//uruchomienie przerwania
	HAL_UART_Receive_IT(&huart2, &buf_rx[buf_end], 1);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	HAL_StatusTypeDef status1;

	//czujnik ciœnienia i temperatury
	bmp180Device bmp180Device = { 0 };
	bmp180Device.handle = &hi2c1;
	bmp180Device.oversampling = 3;
	status1 = bmp180ReadCoefficients(&bmp180Device);

	status = SSD1306_Inicjacja(); //inicjacja wyœwietlacza SSD1306
	if (status == TRUE) {
		wyswietlTemp();
		temp(TempCelsiusDisplay, PressureDisplay);
	}
	while (1) {

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		copy();
		if (flaga == 1) {
			bmp180Pressure = 0;
			bmp180Temperature = 0;

			status1 = bmp180ReadMeasurements(&bmp180Device);

			bmp180Pressure = bmp180Device.pressure; //pobranie cisnienia
			bmp180Temperature = bmp180Device.temperature; //pobranie temperatury

			bmp180Pressure = round(bmp180Pressure * 1000.0); //zaokr¹glenie do najbli¿szej liczby ca³kowitej
			bmp180Temperature = round(bmp180Temperature * 1000.0);

			wyswietlTemp();
			wyswietlPressure();
			temp1(TempCelsiusDisplay, PressureDisplay);
			flaga = 0;
		}
	}
	/* USER CODE END 3 */

}

/** System Clock Configuration
 */
void SystemClock_Config(void) {

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;

	/**Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE()
	;

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
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
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

/* ADC1 init function */
static void MX_ADC1_Init(void) {

	ADC_ChannelConfTypeDef sConfig;

	/**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = ENABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = ENABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* I2C1 init function */
static void MX_I2C1_Init(void) {

	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 100000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* I2C3 init function */
static void MX_I2C3_Init(void) {

	hi2c3.Instance = I2C3;
	hi2c3.Init.ClockSpeed = 100000;
	hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c3.Init.OwnAddress1 = 0;
	hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c3.Init.OwnAddress2 = 0;
	hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c3) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* USART2 init function */
static void MX_USART2_UART_Init(void) {

	huart2.Instance = USART2;
	huart2.Init.BaudRate = 9600;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
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
static void MX_GPIO_Init(void) {

	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE()
	;
	__HAL_RCC_GPIOH_CLK_ENABLE()
	;
	__HAL_RCC_GPIOA_CLK_ENABLE()
	;
	__HAL_RCC_GPIOB_CLK_ENABLE()
	;

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
void _Error_Handler(char * file, int line) {
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
