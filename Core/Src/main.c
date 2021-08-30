/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "fifo.h"
#include "usbd_cdc_if.h"
#include "microrl_cmd.h"
#include "vfd.h"
#include "d3231.h"

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
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi2;

/* USER CODE BEGIN PV */

uint8_t brightness;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void vfd_update(void) {
	uint8_t data = 0b11000000; // command 3, set address to 0
	HAL_GPIO_WritePin(PT6315_STB_GPIO_Port, PT6315_STB_Pin, 0);
	HAL_SPI_Transmit(&hspi2, &data, 1, 0xffffffff);
	HAL_SPI_Transmit(&hspi2, vfd.arr1, sizeof(vfd.arr1), 0xffffffff);
	HAL_GPIO_WritePin(PT6315_STB_GPIO_Port, PT6315_STB_Pin, 1);
}

void do_microrl(void)
{
	while (!fifo_is_empty())
	{
		uint8_t buf = fifo_pop();
		microrl_print_char(buf);
	}
}

void do_vfd_init(void)
{
	HAL_GPIO_WritePin(PT6315_STB_GPIO_Port, PT6315_STB_Pin, 1); // put CS high

	HAL_Delay(300);
	HAL_GPIO_WritePin(HV_EN_GPIO_Port, HV_EN_Pin, 1);

	for (int i = 0; i < sizeof(vfd.arr1); i++) {
		vfd.arr1[i] = 0xFF;
	}
	uint8_t data;

	data = 0b01000001; // command 2, write to LED port
	HAL_GPIO_WritePin(PT6315_STB_GPIO_Port, PT6315_STB_Pin, 0);
	HAL_SPI_Transmit(&hspi2, &data, 1, 0xffffffff);
	HAL_Delay(10);

	data = 0b1111; // disable LEDs

	HAL_SPI_Transmit(&hspi2, &data, 1, 0xffffffff);
	HAL_GPIO_WritePin(PT6315_STB_GPIO_Port, PT6315_STB_Pin, 1);

	data = 0b01000000; // command 2, write to Display port
	HAL_GPIO_WritePin(PT6315_STB_GPIO_Port, PT6315_STB_Pin, 0);
	HAL_SPI_Transmit(&hspi2, &data, 1, 0xffffffff);
	HAL_GPIO_WritePin(PT6315_STB_GPIO_Port, PT6315_STB_Pin, 1);
	HAL_Delay(10);
	vfd_update();
	HAL_Delay(10);
	// init display, 11 digits 17 segments
	data = 0b00000111; // command 1, 11 digits 17 segments
	HAL_GPIO_WritePin(PT6315_STB_GPIO_Port, PT6315_STB_Pin, 0);
	HAL_SPI_Transmit(&hspi2, &data, 1, 0xffffffff);
	HAL_GPIO_WritePin(PT6315_STB_GPIO_Port, PT6315_STB_Pin, 1);
	HAL_Delay(10);

	for (uint8_t i = 0; i <= 0b111; i++) {
		data = 0b10000000; // command 4
		data |= 1 << 3; // enable/disable display
		data |= i; // set brightness
		HAL_GPIO_WritePin(PT6315_STB_GPIO_Port, PT6315_STB_Pin, 0);
		HAL_SPI_Transmit(&hspi2, &data, 1, 0xffffffff);
		HAL_GPIO_WritePin(PT6315_STB_GPIO_Port, PT6315_STB_Pin, 1);
		HAL_Delay(250);
		do_microrl();
	}

	for (int i = 0; i < 11; i++) {
		for (int b = 0; b < 3; b++) // erasing from right to left
		{
			vfd.arr2[i][b] = 0;
		}
		vfd_update();
		HAL_Delay(150);
		do_microrl();
	}
	HAL_Delay(500);
	do_microrl();

	//erase everything... just in case
	clr_vfd();

	// fill everything
	for (int j = 1; j < 15; j++) {
		uint32_t temp = 1 << j;
		for (int i = 1; i < 11; i++) {
			for (int b = 0; b < 3; b++) {
				vfd.arr2[i][b] |= (temp >> (b << 3)) & 0xFF;
			}
		}
		vfd_update();
		HAL_Delay(100);
		do_microrl();
	}

	const uint8_t arr[][2] = {
			{ 6, 0 },
			{ 0, 0 },
			{ 0, 1 },
			{ 0, 4 },
			{ 0, 3 },
			{ 0, 5 },
			{ 0, 2 },
			{ 0, 6 },
			{ 1, 16 },
			{ 1, 15 },
			{ 2, 16 },
			{ 2, 15 },
			{ 3, 16 },
			{ 3, 15 },
			{ 4, 16 },
			{ 4, 15 },
			{ 5, 16 },
			{ 5, 15 },
			{ 6, 16 },
			{ 6, 15 },
			{ 8, 16 },
			{ 8, 15 },
			{ 9, 16 },
			{ 10, 16 },
			{ 10, 15 },
	};

	for (int j = 0; j < sizeof(arr) / 2; j++) {
		for (int b = 0; b < 3; b++)
			vfd.arr2[arr[j][0]][b] |= ((1 << arr[j][1]) >> (b << 3)) & 0xFF;
		vfd_update();
		HAL_Delay(70);
		do_microrl();
	}

	HAL_Delay(300);

	//erase everything... just in case
	clr_vfd();

	vfd_update();
}

void do_brightness(void)
{
	uint8_t data;
	static uint32_t last_time = 0;
	if (HAL_GPIO_ReadPin(PB2_GPIO_Port, PB2_Pin))
	{
		// update only if pause shorter than 2 sec
		if (HAL_GetTick() - last_time < 2000)
		{
			brightness = (brightness - 1)&0b111;
			d3231_set_A2M2(0b111-brightness);
		}

		save_vfd();
		clr_vfd();
		uint32_t bits = 0;
		for (int i = 2; i < 1 + 2 + brightness; i++)
			bits |= 1<<i;
		symbols_vfd(bits);
		str2vfd("brightness");
		vfd_update();

		data = 0b10000000; // command 4
		data |= 1<<3; // enable/disable display
		data |= brightness&0b111; // set brightness
		HAL_GPIO_WritePin(PT6315_STB_GPIO_Port, PT6315_STB_Pin, 0);
		HAL_SPI_Transmit(&hspi2, &data, 1, 0xffffffff);
		HAL_GPIO_WritePin(PT6315_STB_GPIO_Port, PT6315_STB_Pin, 1);
		HAL_Delay(20);
		while(HAL_GPIO_ReadPin(PB2_GPIO_Port, PB2_Pin)); // wait release
		HAL_Delay(100);
		last_time = HAL_GetTick();
		restore_vfd();
		vfd_update();
	}
}

void do_clock(void)
{
	static uint32_t last_time = 0;
	if (HAL_GetTick() - last_time < 50)
		return;
	last_time = HAL_GetTick();

	if (show_clock)
	{
		uint8_t * time = d3231_get_time();
		uint8_t clock [4];

		static uint8_t old_seconds = 0;
		clock[0] = time[1] & 0xF;
		clock[1] = (time[1] >> 4) & 0xF;
		clock[2] = time[2] & 0xF;
		clock[3] = (time[2] >> 4) & 0xF;

		if (time[0] != old_seconds)
		{
			old_seconds = time[0];

			//erase everything...
			clr_vfd();


			for (int i = 0; i < 4; i++)
			{
				uint16_t buf = get_char(clock[i]);

				vfd.arr2[4+i][0] = buf & 0xFF;
				vfd.arr2[4+i][1] = (buf>>8)&0xFF;
			}

			if ((time[0]&0b1) == 0)
			{
				for (int b = 0; b < 3; b++)
					vfd.arr2[6][b] |= ((1<<0)>>(b<<3))&0xFF;
			}

			vfd_update();
		}
	}
}

void do_text(void)
{
	static uint32_t last_time = 0;
	if (HAL_GetTick() - last_time < 100)
		return;
	last_time = HAL_GetTick();

	if (show_clock)
		return;

	clr_vfd();
	str2vfd(txt2disp);
	vfd_update();
}

void do_leds(void)
{
	static uint32_t last_time = 0;
	static uint8_t tick_counter = 0;
	if (HAL_GetTick() - last_time < 500)
		return;
	last_time = HAL_GetTick();
	if (use_leds)
	{
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

		uint8_t data = 0b01000001; // command 2, write to LED port
		HAL_GPIO_WritePin(PT6315_STB_GPIO_Port, PT6315_STB_Pin, 0);
		HAL_SPI_Transmit(&hspi2, &data, 1, 0xffffffff);

		data = ~(1<<((tick_counter++ >> 1)&0b11));

		HAL_SPI_Transmit(&hspi2, &data, 1, 0xffffffff);
		HAL_GPIO_WritePin(PT6315_STB_GPIO_Port, PT6315_STB_Pin, 1);
	}
	else
	{
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 1);
		uint8_t data = 0b01000001; // command 2, write to LED port
		HAL_GPIO_WritePin(PT6315_STB_GPIO_Port, PT6315_STB_Pin, 0);
		HAL_SPI_Transmit(&hspi2, &data, 1, 0xffffffff);

		data = 0b1111; // disable all leds

		HAL_SPI_Transmit(&hspi2, &data, 1, 0xffffffff);
		HAL_GPIO_WritePin(PT6315_STB_GPIO_Port, PT6315_STB_Pin, 1);
	}


}

void do_temperature(void)
{
	uint16_t buf;
	// show temperature
	if (HAL_GPIO_ReadPin(PB1_GPIO_Port, PB1_Pin))
	{
		//erase everything...
		clr_vfd();

		uint8_t td3231 = *d3231_get_temp();
		uint8_t td [6];

		td[0] = 'C';
		td[1] = 176; //°
		uint8_t i = 2;
		while (td3231)
		{
			td[i++] = td3231 %10;
			td3231 /= 10;
		}
		if (i>2)
			td[i] = td3231&(1<<7)?'-':'+';

		for (int j = 0; j <= i; j++)
		{
			buf = get_char(td[j]);

			vfd.arr2[j+1][0] = buf & 0xFF;
			vfd.arr2[j+1][1] = (buf>>8)&0xFF;
		}

		vfd_update();
		HAL_Delay(20);
		while(HAL_GPIO_ReadPin(PB1_GPIO_Port, PB1_Pin)); // wait release
		HAL_Delay(1000);
		show_clock = true;
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
	MX_I2C1_Init();
	MX_SPI2_Init();
	MX_USB_DEVICE_Init();
	/* USER CODE BEGIN 2 */
	HAL_GPIO_WritePin(USB_PU_GPIO_Port, USB_PU_Pin, 1); // we have initialized USB, pull it up!
	d3231_init(&hi2c1);
	init_microrl(); // we are ready for microrl!

	do_vfd_init(); // nice demo

	d3231_get_all();

	brightness = 0b111-d3231_get_A2M2(); // alarm2 minutes as EEPROM, default max

	uint8_t data;

	data = 0b10000000; // command 4
	data |= 1<<3; // enable/disable display
	data |= brightness&0b111; // set brightness
	HAL_GPIO_WritePin(PT6315_STB_GPIO_Port, PT6315_STB_Pin, 0);
	HAL_SPI_Transmit(&hspi2, &data, 1, 0xffffffff);
	HAL_GPIO_WritePin(PT6315_STB_GPIO_Port, PT6315_STB_Pin, 1);


	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		do_microrl();
		do_brightness();
		do_clock();
		do_text();
		do_leds();
		do_temperature();
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

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
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
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
	PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
	hi2c1.Init.ClockSpeed = 100000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief SPI2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI2_Init(void)
{

	/* USER CODE BEGIN SPI2_Init 0 */

	/* USER CODE END SPI2_Init 0 */

	/* USER CODE BEGIN SPI2_Init 1 */

	/* USER CODE END SPI2_Init 1 */
	/* SPI2 parameter configuration*/
	hspi2.Instance = SPI2;
	hspi2.Init.Mode = SPI_MODE_MASTER;
	hspi2.Init.Direction = SPI_DIRECTION_2LINES;
	hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
	hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
	hspi2.Init.NSS = SPI_NSS_SOFT;
	hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
	hspi2.Init.FirstBit = SPI_FIRSTBIT_LSB;
	hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi2.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN SPI2_Init 2 */

	/* USER CODE END SPI2_Init 2 */

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
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, SPI2_nRF_CSn_Pin|PT6315_STB_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(USB_PU_GPIO_Port, USB_PU_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(HV_EN_GPIO_Port, HV_EN_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : LED_Pin */
	GPIO_InitStruct.Pin = LED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : PB1_Pin PB2_Pin */
	GPIO_InitStruct.Pin = PB1_Pin|PB2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : SPI2_nRF_CSn_Pin HV_EN_Pin PT6315_STB_Pin */
	GPIO_InitStruct.Pin = SPI2_nRF_CSn_Pin|HV_EN_Pin|PT6315_STB_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : USB_PU_Pin */
	GPIO_InitStruct.Pin = USB_PU_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(USB_PU_GPIO_Port, &GPIO_InitStruct);

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
