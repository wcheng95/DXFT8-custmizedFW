/**
 ******************************************************************************
 * @file    USB_Host/HID_Standalone/Src/main.c
 * @author  MCD Application Team
 * @version V1.0.2
 * @date    18-November-2015
 * @brief   USB host HID Mouse and Keyboard demo main file
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
 *
 * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
 * You may not use this file except in compliance with the License.
 * You may obtain a copy of the License at:
 *
 *        http://www.st.com/software_license_agreement_liberty_v2
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f7xx_hal_rcc.h"
#include "stm32746g_discovery_ts.h"
#include "arm_math.h"
#include "stm32746g_discovery_lcd.h"
#include "SDR_Audio.h"
#include "Display.h"
#include "Process_DSP.h"
#include "stm32f7xx_hal_tim.h"
#include "Codec_Gains.h"
#include "button.h"

#include "decode_ft8.h"
#include "gen_ft8.h"
#include "log_file.h"
#include "traffic_manager.h"
#include "button.h"
#include "DS3231.h"

#include "SiLabs.h"

#include "options.h"

TIM_HandleTypeDef hTim2;
uint32_t current_time, start_time, ft8_time;

int master_decoded;
int QSO_xmit;
int Xmit_DSP_counter;
int slot_state = 0;
int target_slot;
int target_freq;
int slot_number;

int quiet_counter;

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void Error_Handler(void);
static void CPU_CACHE_Enable(void);
static void HID_InitApplication(void);

static void update_synchronization(void)
{
	current_time = HAL_GetTick();
	ft8_time = current_time - start_time;

	if (ft8_time % 15000 <= 160 || FT_8_counter > 90)
	{
		ft8_flag = 1;
		FT_8_counter = 0;
		ft8_marker = 1;
		decode_flag = 0;

		if (QSO_xmit == 1 && target_slot == slot_state)
		{
			setup_to_transmit_on_next_DSP_Flag();
			update_log_display(1);
			QSO_xmit = 0;
		}
	}
}

int main(void)
{
	CPU_CACHE_Enable();

	HAL_Init();

	/* Configure the System clock to have a frequency of 200 MHz */
	SystemClock_Config();
	HAL_Delay(10);
	start_audio_I2C();
	HAL_Delay(10);
	PTT_Out_Init();
	HAL_Delay(10);
	Init_BoardVersionInput();
	HAL_Delay(10);
	Check_Board_Version();
	HAL_Delay(10);
	DeInit_BoardVersionInput();
	HAL_Delay(10);
	HID_InitApplication(); // really sets up LCD Display, leftover from example
	HAL_Delay(10);
	BSP_TS_Init(BSP_LCD_GetXSize(), BSP_LCD_GetYSize());
	HAL_Delay(10);
	initalize_constants();
	HAL_Delay(10);
	init_DSP();
	HAL_Delay(10);
	SD_Initialize();
	HAL_Delay(10);
	Read_Station_File();
	HAL_Delay(10);
	setup_display();
	HAL_Delay(10);
	Options_Initialize();
	HAL_Delay(10);
	EXT_I2C_Init();
	HAL_Delay(10);
	DS3231_init();
	HAL_Delay(10);
	display_Real_Date(0, 240);
	HAL_Delay(10);
	start_Si5351();
	HAL_Delay(10);
	cursor = 192; // 1500 Hz
	Set_Cursor_Frequency();
	HAL_Delay(10);
	show_variable(400, 25, (int)NCO_Frequency);
	HAL_Delay(10);
	show_short(405, 255, AGC_Gain);
	HAL_Delay(10);

	Xmit_Mode = 0;

	HAL_Delay(10);

	start_duplex(0);
	HAL_Delay(10);
	set_codec_input_gain();
	HAL_Delay(10);
	receive_sequence();
	HAL_Delay(10);
	Set_HP_Gain(30);
	HAL_Delay(10);

	Init_Log_File();
	HAL_Delay(10);
	FT8_Sync();
	HAL_Delay(10);

	while (1)
	{
		if (DSP_Flag == 1)
		{
			I2S2_RX_ProcessBuffer(buff_offset);

			if (xmit_flag == 1)
			{
				if (ft8_xmit_delay >= 20)
				{
					if (Tune_On == 0)
					{
						if (ft8_xmit_counter < 79)
						{
							if (Xmit_DSP_counter % 4 == 0)
							{
								//ft8_shift = ft8_hz * (double)tones[ft8_xmit_counter];
								set_FT8_Tone(tones[ft8_xmit_counter]);
								ft8_xmit_counter++;
							}
						}

						Xmit_DSP_counter++;

						if (ft8_xmit_counter == 79)
						{
							xmit_flag = 0;
							ft8_receive_sequence();
							receive_sequence();
							ft8_xmit_delay = 0;
							if (Beacon_On == 0)
								clear_qued_message();
						}
					}
					else
					{
						ft8_shift = 0;
					}
				}
				else
				{
					ft8_xmit_delay++;

					if (ft8_xmit_delay == 16){
						output_enable(SI5351_CLK0, 1);
					}
				}
			}

			display_RealTime(100, 240);

			if (Tune_On == 1)
			{
				display_Real_Date(0, 240);
			}

			DSP_Flag = 0;
		}

		if (decode_flag == 1 && Tune_On == 0 && xmit_flag == 0)
		{
			// toggle the slot state
			slot_state = (slot_state == 0) ? 1 : 0;
			clear_decoded_messages();

			master_decoded = ft8_decode();
			if (master_decoded > 0)
			{
				quiet_counter = 0;
				display_messages(master_decoded);
				if (Beacon_On == 1)
					service_Beacon_mode(master_decoded);
				else
				if (Beacon_On == 0)
					service_QSO_mode(master_decoded);
			}
			else {
				if(quiet_counter < 2) quiet_counter++;

				if((quiet_counter == 2) && (Beacon_On == 1)) {
					service_Beacon_mode(1); //Initiate a CQ when the band is quiet
				}
			}

			decode_flag = 0;
		} // end of servicing FT_Decode

		if (FT_8_counter > 0 && FT_8_counter < 90)
			Process_Touch();

		if (Tune_On == 0 && FT8_Touch_Flag == 1 && Beacon_On == 0)
			process_selected_Station(master_decoded, FT_8_TouchIndex);

		update_synchronization();
	}
}

/**
 * @brief  HID application Init
 * @param  None
 * @retval None
 */
static void HID_InitApplication(void)
{
	/* Configure Key button */
	BSP_PB_Init(BUTTON_TAMPER, BUTTON_MODE_GPIO);

	/* Configure LED1 */
	BSP_LED_Init(LED1);

	/* Initialize the LCD */
	BSP_LCD_Init();

	/* LCD Layer Initialization */
	BSP_LCD_LayerDefaultInit(1, LCD_FB_START_ADDRESS);

	/* Select the LCD Layer */
	BSP_LCD_SelectLayer(1);

	/* Enable the display */
	BSP_LCD_DisplayOn();
}

/**
 * @brief  User Process
 * @param  phost: Host Handle
 * @param  id: Host Library user message ID
 * @retval None
 */

/**
 * @brief This function provides accurate delay (in milliseconds) based
 *        on SysTick counter flag.
 * @note This function is declared as __weak to be overwritten in case of other
 *       implementations in user file.
 * @param Delay: specifies the delay time length, in milliseconds.
 * @retval None
 */

void HAL_Delay(__IO uint32_t Delay)
{
	while (Delay)
	{
		if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)
		{
			Delay--;
		}
	}
}

/**
 * @brief  System Clock Configuration
 *         The system Clock is configured as follow :
 *            System Clock source            = PLL (HSE)
 *            SYSCLK(Hz)                     = 200000000
 *            HCLK(Hz)                       = 200000000
 *            AHB Prescaler                  = 1
 *            APB1 Prescaler                 = 4
 *            APB2 Prescaler                 = 2
 *            HSE Frequency(Hz)              = 25000000
 *            PLL_M                          = 25
 *            PLL_N                          = 400
 *            PLL_P                          = 2
 *            PLLSAI_N                       = 384
 *            PLLSAI_P                       = 8
 *            VDD(V)                         = 3.3
 *            Main regulator output voltage  = Scale1 mode
 *            Flash Latency(WS)              = 7
 * @param  None
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

	/* Enable HSE Oscillator and activate PLL with HSE as source */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSIState = RCC_HSI_OFF;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 25;
	RCC_OscInitStruct.PLL.PLLN = 400;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 8;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/* Activate the OverDrive to reach the 200 Mhz Frequency */
	if (HAL_PWREx_EnableOverDrive() != HAL_OK)
	{
		Error_Handler();
	}

	/* Select PLLSAI output as USB clock source */
	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CLK48;
	PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLLSAIP;
	PeriphClkInitStruct.PLLSAI.PLLSAIN = 192;
	PeriphClkInitStruct.PLLSAI.PLLSAIQ = 4;
	PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV4;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
	 clocks dividers */
	RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
static void Error_Handler(void)
{
	/* User may add here some code to deal with this error */
	while (1)
	{
	}
}

/**
 * @brief  CPU L1-Cache enable.
 * @param  None
 * @retval None
 */
static void CPU_CACHE_Enable(void)
{
	/* Enable I-Cache */
	SCB_EnableICache();

	/* Enable D-Cache */
	SCB_EnableDCache();
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
