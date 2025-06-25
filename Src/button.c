/*
 * button.c
 *
 *  Created on: Feb 18, 2016
 *      Author: user
 */

#include <Display.h>
#include <stdlib.h>
#include "button.h"
#include "stm32746g_discovery_lcd.h"
#include "SDR_Audio.h"
#include "Codec_Gains.h"
#include "decode_ft8.h"
#include "main.h"
#include "stm32fxxx_hal.h"
#include "Process_DSP.h"
#include "log_file.h"
#include "gen_ft8.h"
#include "traffic_manager.h"
#include "DS3231.h"
#include "SiLabs.h"
#include "options.h"

int Tune_On; // 0 = Receive, 1 = Xmit Tune Signal
int Beacon_On;
int Arm_Tune;
int Auto_Sync;
uint16_t start_freq;
int BandIndex;
int QSO_Fix;
int CQ_Mode_Index;
int Free_Index;
int Free2_Index=1;
int Send_Free;

int SkipGrid=1;

int AGC_Gain = 20;
int ADC_DVC_Gain = 180;
int ADC_DVC_Off = 90;

int Band_Minimum;

extern int Auto_QSO_State;

char display_frequency[] = "14.075";

FreqStruct sBand_Data_default[] =
	{
		{// 40,
		 7074, "7.074"},
		{// 30,
		 10136, "10.136"},
		{// 20,
		 14074, "14.075"},
		{// 17,
		 18100, "18.101"},
		{// 15,
		 21074, "21.075"},
		{// 12,
		 24915, "24.916"},
		{// 10,
		 28074, "28.075"}};

FreqStruct sBand_Data_external[] =
	{
		{// 40,
		 0, " "},
		{// 30,
		 0, " "},
		{// 20,
		 0, " "},
		{// 17,
		 0, " "},
		{// 15,
		 0, " "},
		{// 12,
		 0, " "},
		{// 10,
		 0, " "}};

FreqStruct sBand_Data[] =
	{
		{// 40,
		 7074, "7.074"},
		{// 30,
		 10136, "10.136"},
		{// 20,
		 14074, "14.075"},
		{// 17,
		 18100, "18.101"},
		{// 15,
		 21074, "21.075"},
		{// 12,
		 24915, "24.916"},
		{// 10,
		 28074, "28.075"}};

#define numButtons 83

ButtonStruct sButtonData[] = {
	{// button 0  inhibit xmit either as beacon or answer CQ
	 /*text0*/ "Clr ",
	 /*text1*/ "Clr ",
	 /*blank*/ "    ",
	 /*Active*/ 1,
	 /*Displayed*/ 1,
	 /*state*/ 0,
	 /*x*/ 0,
	 /*y*/ line2,
	 /*w*/ button_width,
	 /*h*/ 30},

	{// button 1 control Beaconing / CQ chasing
	 /*text0*/ "QSO ",
	 /*text1*/ "Becn",
	 /*blank*/ "    ",
	 /*Active*/ 1,
	 /*Displayed*/ 1,
	 /*state*/ 0,
	 /*x*/ 55,
	 /*y*/ line2,
	 /*w*/ button_width,
	 /*h*/ 30},

	{// button 2 control Tune
	 /*text0*/ "Tune",
	 /*text1*/ "Tune",
	 /*blank*/ "    ",
	 /*Active*/ 1,
	 /*Displayed*/ 1,
	 /*state*/ 0,
	 /*x*/ 110,
	 /*y*/ line2,
	 /*w*/ button_width,
	 /*h*/ 30},

	{// button 3 display R/T status
	 /*text0*/ " CQ ",
	 /*text1*/ "Free",
	 /*blank*/ "    ",
	 /*Active*/ 1,
	 /*Displayed*/ 1,
	 /*state*/ 0,
	 /*x*/ 195,
	 /*y*/ line2,
	 /*w*/ button_width, // setting the width and height to 0 turns off touch response , display only
	 /*h*/ 30},

	{// button 4 QSO Response Freq 0 fixed, 1 Match received station frequency
	 /*text0*/ "Fixd",
	 /*text1*/ "Rcvd",
	 /*blank*/ "   ",
	 /*Active*/ 1,
	 /*Displayed*/ 1,
	 /*state*/ 0,
	 /*x*/ 250,
	 /*y*/ line2,
	 /*w*/ button_width,
	 /*h*/ 30},

	{// button 5 Sync FT8
	 /*text0*/ "Sync",
	 /*text1*/ "Sync",
	 /*blank*/ "    ",
	 /*Active*/ 1,
	 /*Displayed*/ 1,
	 /*state*/ 0,
	 /*x*/ 305,
	 /*y*/ line2,
	 /*w*/ button_width,
	 /*h*/ 30

	},

	{// button 6 Lower Gain
	 /*text0*/ " G- ",
	 /*text1*/ " G- ",
	 /*blank*/ "    ",
	 /*Active*/ 2,
	 /*Displayed*/ 1,
	 /*state*/ 0,
	 /*x*/ 360,
	 /*y*/ line2,
	 /*w*/ button_width,
	 /*h*/ 30

	},

	{// button 7 Raise Gain
	 /*text0*/ " G+ ",
	 /*text1*/ " G+ ",
	 /*blank*/ "    ",
	 /*Active*/ 2,
	 /*Displayed*/ 1,
	 /*state*/ 0,
	 /*x*/ 430,
	 /*y*/ line2,
	 /*w*/ button_width,
	 /*h*/ 30

	},

	{// button 8 Lower Cursor
	 /*text0*/ "F- ",
	 /*text1*/ "F- ",
	 /*blank*/ "    ",
	 /*Active*/ 2,
	 /*Displayed*/ 1,
	 /*state*/ 0,
	 /*x*/ 360,
	 /*y*/ line0,
	 /*w*/ button_width,
	 /*h*/ 30

	},

	{// button 9 Raise Cursor
	 /*text0*/ "F+ ",
	 /*text1*/ "F+ ",
	 /*blank*/ "    ",
	 /*Active*/ 2,
	 /*Displayed*/ 1,
	 /*state*/ 0,
	 /*x*/ 450,
	 /*y*/ line0,
	 /*w*/ button_width,
	 /*h*/ 30

	},

	{// button 10 Band Down
	 /*text0*/ "Band-",
	 /*text1*/ "Band-",
	 /*blank*/ "    ",
	 /*Active*/ 0,
	 /*Displayed*/ 1,
	 /*state*/ 0,
	 /*x*/ 270,
	 /*y*/ 40,
	 /*w*/ button_width,
	 /*h*/ 30

	},

	{// button 11 Band Up
	 /*text0*/ "Band+",
	 /*text1*/ "Band+",
	 /*blank*/ "     ",
	 /*Active*/ 0,
	 /*Displayed*/ 1,
	 /*state*/ 0,
	 /*x*/ 420,
	 /*y*/ 40,
	 /*w*/ button_width,
	 /*h*/ 30

	},

	{// button 12 Xmit for Calibration
	 /*text0*/ "Xmit",
	 /*text1*/ "Xmit",
	 /*blank*/ "    ",
	 /*Active*/ 0,
	 /*Displayed*/ 1,
	 /*state*/ 0,
	 /*x*/ 420,
	 /*y*/ 70,
	 /*w*/ button_width,
	 /*h*/ 30

	},

	{// button 13 Save Calibration Data
	 /*text0*/ "Save",
	 /*text1*/ "Save",
	 /*blank*/ "    ",
	 /*Active*/ 0,
	 /*Displayed*/ 1,
	 /*state*/ 0,
	 /*x*/ 360,
	 /*y*/ 70,
	 /*w*/ button_width,
	 /*h*/ 30

	},

	{// button 14 Save RTC Time
	 /*text0*/ "Set ",
	 /*text1*/ "Set ",
	 /*blank*/ "    ",
	 /*Active*/ 0,
	 /*Displayed*/ 1,
	 /*state*/ 0,
	 /*x*/ RTC_Button,
	 /*y*/ RTC_line2,
	 /*w*/ button_width,
	 /*h*/ 30

	},

	{// button 15 Hours
	 /*text0*/ "Hr- ",
	 /*text1*/ "Hr- ",
	 /*blank*/ "    ",
	 /*Active*/ 0,
	 /*Displayed*/ 1,
	 /*state*/ 0,
	 /*x*/ RTC_Sub,
	 /*y*/ RTC_line0,
	 /*w*/ button_width,
	 /*h*/ 30

	},

	{// button 16 Hours
	 /*text0*/ "Hr+ ",
	 /*text1*/ "Hr+ ",
	 /*blank*/ "    ",
	 /*Active*/ 0,
	 /*Displayed*/ 1,
	 /*state*/ 0,
	 /*x*/ RTC_Add,
	 /*y*/ RTC_line0,
	 /*w*/ button_width,
	 /*h*/ 30},

	{// button 17 minutes
	 /*text0*/ "Min-",
	 /*text1*/ "Min-",
	 /*blank*/ "    ",
	 /*Active*/ 0,
	 /*Displayed*/ 1,
	 /*state*/ 0,
	 /*x*/ RTC_Sub,
	 /*y*/ RTC_line1,
	 /*w*/ button_width,
	 /*h*/ 30},

	{// button 18 minutes
	 /*text0*/ "Min+",
	 /*text1*/ "Min+",
	 /*blank*/ "    ",
	 /*Active*/ 0,
	 /*Displayed*/ 1,
	 /*state*/ 0,
	 /*x*/ RTC_Add,
	 /*y*/ RTC_line1,
	 /*w*/ button_width,
	 /*h*/ 30},

	{// button 19 Seconds
	 /*text0*/ "Sec-",
	 /*text1*/ "Sec-",
	 /*blank*/ "    ",
	 /*Active*/ 0,
	 /*Displayed*/ 1,
	 /*state*/ 0,
	 /*x*/ RTC_Sub,
	 /*y*/ RTC_line2,
	 /*w*/ button_width,
	 /*h*/ 30

	},

	{// button 20 Seconds
	 /*text0*/ "Sec+",
	 /*text1*/ "Sec+",
	 /*blank*/ "    ",
	 /*Active*/ 0,
	 /*Displayed*/ 1,
	 /*state*/ 0,
	 /*x*/ RTC_Add,
	 /*y*/ RTC_line2,
	 /*w*/ button_width,
	 /*h*/ 30

	},

	{// button 21 Day
	 /*text0*/ "Day-",
	 /*text1*/ "Day-",
	 /*blank*/ "    ",
	 /*Active*/ 0,
	 /*Displayed*/ 1,
	 /*state*/ 0,
	 /*x*/ RTC_Sub,
	 /*y*/ RTC_line3,
	 /*w*/ button_width,
	 /*h*/ 30},

	{// button 22 Day
	 /*text0*/ "Day+",
	 /*text1*/ "Day+",
	 /*blank*/ "    ",
	 /*Active*/ 0,
	 /*Displayed*/ 1,
	 /*state*/ 0,
	 /*x*/ RTC_Add,
	 /*y*/ RTC_line3,
	 /*w*/ button_width,
	 /*h*/ 30},

	{// button 23 Month
	 /*text0*/ "Mon-",
	 /*text1*/ "Mon-",
	 /*blank*/ "    ",
	 /*Active*/ 0,
	 /*Displayed*/ 1,
	 /*state*/ 0,
	 /*x*/ RTC_Sub,
	 /*y*/ RTC_line4,
	 /*w*/ button_width,
	 /*h*/ 30},

	{// button 24 Month
	 /*text0*/ "Mon+",
	 /*text1*/ "Mon+",
	 /*blank*/ "    ",
	 /*Active*/ 0,
	 /*Displayed*/ 1,
	 /*state*/ 0,
	 /*x*/ RTC_Add,
	 /*y*/ RTC_line4,
	 /*w*/ button_width,
	 /*h*/ 30

	},

	{// button 25 Year
	 /*text0*/ "Yr- ",
	 /*text1*/ "Yr- ",
	 /*blank*/ "    ",
	 /*Active*/ 0,
	 /*Displayed*/ 1,
	 /*state*/ 0,
	 /*x*/ RTC_Sub,
	 /*y*/ RTC_line5,
	 /*w*/ button_width,
	 /*h*/ 30},

	{// button 26 Year
	 /*text0*/ "Yr+ ",
	 /*text1*/ "Yr+ ",
	 /*blank*/ "    ",
	 /*Active*/ 0,
	 /*Displayed*/ 1,
	 /*state*/ 0,
	 /*x*/ RTC_Add,
	 /*y*/ RTC_line5,
	 /*w*/ button_width,
	 /*h*/ 30

	},

	{// button 27 Save  RTC Date
	 /*text0*/ "Set ",
	 /*text1*/ "Set ",
	 /*blank*/ "    ",
	 /*Active*/ 0,
	 /*Displayed*/ 1,
	 /*state*/ 0,
	 /*x*/ RTC_Button,
	 /*y*/ RTC_line5,
	 /*w*/ button_width,
	 /*h*/ 30},

	 {// button 28 display R/T status
		 /*text0*/ "Rx",
		 /*text1*/ "Tx",
		 /*blank*/ "  ",
		 /*Active*/ 1,
		 /*Displayed*/ 1,
		 /*state*/ 0,
		 /*x*/ 165,
		 /*y*/ line2,
		 /*w*/ 0, // setting the width and height to 0 turns off touch response , display only
		 /*h*/ 0},


	{// button 29 Standard CQ
		 /*text0*/ "CQ",
		 /*text1*/ "CQ",
		 /*blank*/ "  ",
		 /*Active*/ 0,
		 /*Displayed*/ 1,
		 /*state*/ 1,
		 /*x*/ 240,
		 /*y*/ 105,
		 /*w*/ button_width,
		 /*h*/ 30},

	{// button 30 CQ DX
		 /*text0*/ "DX",
		 /*text1*/ "DX",
		 /*blank*/ "  ",
		 /*Active*/ 0,
		 /*Displayed*/ 1,
		 /*state*/ 0,
		 /*x*/ 275,
		 /*y*/ 105,
		 /*w*/ button_width,
		 /*h*/ 30},

	{// button 31 CQ POTA
		 /*text0*/ "POTA",
		 /*text1*/ "POTA",
		 /*blank*/ "    ",
		 /*Active*/ 0,
		 /*Displayed*/ 1,
		 /*state*/ 0,
		 /*x*/ 310,
		 /*y*/ 105,
		 /*w*/ button_width,
		 /*h*/ 30},

	{// button 32 CQ SOTA
		 /*text0*/ "SOTA",
		 /*text1*/ "SOTA",
		 /*blank*/ "    ",
		 /*Active*/ 0,
		 /*Displayed*/ 1,
		 /*state*/ 0,
		 /*x*/ 365,
		 /*y*/ 105,
		 /*w*/ button_width,
		 /*h*/ 30},

	{// button 33 CQ QRPP
		 /*text0*/ "QRPP",
		 /*text1*/ "QRPP",
		 /*blank*/ "    ",
		 /*Active*/ 0,
		 /*Displayed*/ 1,
		 /*state*/ 0,
		 /*x*/ 420,
		 /*y*/ 105,
		 /*w*/ button_width,
		 /*h*/ 30},

	{// button 34 Free Text 1
		 /*text0*/ "Free1",
		 /*text1*/ "Free1",
		 /*blank*/ "    ",
		 /*Active*/ 0,
		 /*Displayed*/ 1,
		 /*state*/ 0,
		 /*x*/ 256,
		 /*y*/ 130,
		 /*w*/ 160,
		 /*h*/ 30},
	{// button 35 Free Text 2
		 /*text0*/ "Free2",
		 /*text1*/ "Free2",
		 /*blank*/ "    ",
		 /*Active*/ 0,
		 /*Displayed*/ 1,
		 /*state*/ 0,
		 /*x*/ 256,
		 /*y*/ 155,
		 /*w*/ 160,
		 /*h*/ 30},

	{// button 36 Free Text 3
		 /*text0*/ "Free3",
		 /*text1*/ "Free3",
		 /*blank*/ "    ",
		 /*Active*/ 0,
		 /*Displayed*/ 1,
		 /*state*/ 0,
		 /*x*/ 256,
		 /*y*/ 180,
		 /*w*/ 160,
		 /*h*/ 30},

	{// button 37 Free Text 4
		 /*text0*/ "Free4",
		 /*text1*/ "Free4",
		 /*blank*/ "    ",
		 /*Active*/ 0,
		 /*Displayed*/ 1,
		 /*state*/ 0,
		 /*x*/ 256,
		 /*y*/ 205,
		 /*w*/ 160,
		 /*h*/ 30},

	{// button 38 Skip Grid
		 /*text0*/ "SkipGrid",
		 /*text1*/ "SkipGrid",
		 /*blank*/ "        ",
		 /*Active*/ 0,
		 /*Displayed*/ 1,
		 /*state*/ 1,
		 /*x*/ 240,
		 /*y*/ 70,
		 /*w*/ 90,
		 /*h*/ 30},

		 //text0, text1, blank, Active, Displayed, state,   x,   y,   w,   h

		/*39*/ {   "E",  "E",  "  ",  0,      1,         0,     240,  205,  16,  30},    //Enable Edit

		/*40*/ {   "A",  "A",  " ",  0,      1,         0,     240,  80,  25,  30},
		/*41*/ {   "B",  "B",  " ",  0,      1,         0,     265,  80,  25,  30},
		/*42*/ {   "C",  "C",  " ",  0,      1,         0,     290,  80,  25,  30},
		/*43*/ {   "D",  "D",  " ",  0,      1,         0,     315,  80,  25,  30},
		/*44*/ {   "E",  "E",  " ",  0,      1,         0,     340,  80,  25,  30},
		/*45*/ {   "F",  "F",  " ",  0,      1,         0,     365,  80,  25,  30},
		/*46*/ {   "G",  "G",  " ",  0,      1,         0,     390,  80,  25,  30},
		/*47*/ {   "H",  "H",  " ",  0,      1,         0,     415,  80,  25,  30},
		/*48*/ {   "I",  "I",  " ",  0,      1,         0,     440,  80,  25,  30},

		/*49*/ {   "J",  "J",  " ",  0,      1,         0,     240,  105,  25,  30},
		/*50*/ {   "K",  "K",  " ",  0,      1,         0,     265,  105,  25,  30},
		/*51*/ {   "L",  "L",  " ",  0,      1,         0,     290,  105,  25,  30},
		/*52*/ {   "M",  "M",  " ",  0,      1,         0,     315,  105,  25,  30},
		/*53*/ {   "N",  "N",  " ",  0,      1,         0,     340,  105,  25,  30},
		/*54*/ {   "O",  "O",  " ",  0,      1,         0,     365,  105,  25,  30},
		/*55*/ {   "P",  "P",  " ",  0,      1,         0,     390,  105,  25,  30},
		/*56*/ {   "Q",  "Q",  " ",  0,      1,         0,     415,  105,  25,  30},
		/*57*/ {   "R",  "R",  " ",  0,      1,         0,     440,  105,  25,  30},
                  
		/*58*/ {   "S",  "S",  " ",  0,      1,         0,     240,  130,  25,  30},
		/*59*/ {   "T",  "T",  " ",  0,      1,         0,     265,  130,  25,  30},
		/*60*/ {   "U",  "U",  " ",  0,      1,         0,     290,  130,  25,  30},
		/*61*/ {   "V",  "V",  " ",  0,      1,         0,     315,  130,  25,  30},
		/*62*/ {   "W",  "W",  " ",  0,      1,         0,     340,  130,  25,  30},
		/*63*/ {   "X",  "X",  " ",  0,      1,         0,     365,  130,  25,  30},
		/*64*/ {   "Y",  "Y",  " ",  0,      1,         0,     390,  130,  25,  30},
		/*65*/ {   "Z",  "Z",  " ",  0,      1,         0,     415,  130,  25,  30},
		/*66*/ {   "0",  "0",  " ",  0,      1,         0,     440,  130,  25,  30},

		/*67*/ {   "1",  "1",  " ",  0,      1,         0,     240,  155,  25,  30},
		/*68*/ {   "2",  "2",  " ",  0,      1,         0,     265,  155,  25,  30},
		/*69*/ {   "3",  "3",  " ",  0,      1,         0,     290,  155,  25,  30},
		/*70*/ {   "4",  "4",  " ",  0,      1,         0,     315,  155,  25,  30},
		/*71*/ {   "5",  "5",  " ",  0,      1,         0,     340,  155,  25,  30},
		/*72*/ {   "6",  "6",  " ",  0,      1,         0,     365,  155,  25,  30},
		/*73*/ {   "7",  "7",  " ",  0,      1,         0,     390,  155,  25,  30},
		/*74*/ {   "8",  "8",  " ",  0,      1,         0,     415,  155,  25,  30},
		/*75*/ {   "9",  "9",  " ",  0,      1,         0,     440,  155,  25,  30},

		/*76*/ {   "+",  "+",  " ",  0,      1,         0,     240,  180,  25,  30},
		/*77*/ {   "-",  "-",  " ",  0,      1,         0,     265,  180,  25,  30},
		/*78*/ {   ".",  ".",  " ",  0,      1,         0,     290,  180,  25,  30},
		/*79*/ {   "/",  "/",  " ",  0,      1,         0,     315,  180,  25,  30},
		/*80*/ {   "?",  "?",  " ",  0,      1,         0,     340,  180,  25,  30},
		/*81*/ {   "SPC",  "SPC",  "   ",  0,      1,      0,     365,  180,  25,  30},
		/*82*/ {   "<--",  "<--",  "  ",  0,        1,         0,     415,  180,  25,  30},
	
}; // end of button definition

void drawButton(uint16_t button)
{
	BSP_LCD_SetFont(&Font16);
	if (sButtonData[button].Active > 0)
	{
		if (sButtonData[button].state == 1)
			BSP_LCD_SetBackColor(LCD_COLOR_RED);
		else
			BSP_LCD_SetBackColor(LCD_COLOR_BLUE);

		BSP_LCD_SetTextColor(LCD_COLOR_WHITE);

		if (sButtonData[button].state == 1)
			BSP_LCD_DisplayStringAt(sButtonData[button].x, sButtonData[button].y + 15,
									(const uint8_t *)sButtonData[button].text1, LEFT_MODE);
		else
			BSP_LCD_DisplayStringAt(sButtonData[button].x, sButtonData[button].y + 15,
									(const uint8_t *)sButtonData[button].text0, LEFT_MODE);

		BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
	}
}

void checkButton(void)
{
	for (uint16_t button = 0; button < numButtons; button++)
	{
		if (testButton(sButtonData[button].x, sButtonData[button].y, sButtonData[button].w,
					   sButtonData[button].h) == 1)
		{

			switch (sButtonData[button].Active)
			{
			case 0:
				break;

			case 1:
				sButtonData[button].state = !sButtonData[button].state;
				drawButton(button);
				executeButton(button);
				break;

			case 2:
				executeButton(button);
				break;

			case 3:
				executeCalibrationButton(button);
				break;
			}
		}
	}
}

void SelectFilterBlock()
{
	if (Band_Minimum == _40M)
	{
		if (BandIndex < _17M) // i.e. 40M, 30M or 20M
			RLY_Select_20to40();
		else
			RLY_Select_10to17();
	}
}

static void toggle_button_state(int button)
{
	sButtonData[button].state = 1;
	drawButton(button);
	HAL_Delay(10);
	sButtonData[button].state = 0;
	drawButton(button);
}

void executeButton(uint16_t index)
{
	switch (index)
	{
	case 0: // Reset
		clear_xmit_messages();
		terminate_QSO();
		Auto_QSO_State = 0;
		QSO_xmit = 0;
		clear_reply_message_box();
		clear_log_stored_data();
		clear_log_messages();

		toggle_button_state(0);
		break;

	case 1: // Toggle beacon mode
		if (!sButtonData[1].state)
		{
			Beacon_On = 0;
			Beacon_State = 0;
			clear_reply_message_box();
			clear_log_messages();
			clear_log_stored_data();
		}
		else
		{
			Beacon_On = 1;
			clear_reply_message_box();
			clear_log_stored_data();
			clear_Beacon_log_messages();
			Beacon_State = 1;
		}
		break;

	case 2: // Toggle tune (setup) mode
		if (!sButtonData[2].state)
		{
			tune_Off_sequence();
			Tune_On = 0;
			Arm_Tune = 0;
			xmit_flag = 0;
			receive_sequence();
			erase_Cal_Display();
		}
		else
		{
			Tune_On = 1; // Turns off display of FT8 traffic
			setup_Cal_Display();
			Arm_Tune = 0;
		}
		break;

	case 3:
		if (sButtonData[3].state == 1)
			if(Free_Index != 0) Send_Free = 1;
			else {sButtonData[3].state = 0; drawButton(3);Send_Free = 0;}
		else
			Send_Free = 0;

		break;

	case 4: // Toggle QSO TX fix mode
		if (sButtonData[4].state == 1)
			QSO_Fix = 1;
		else
			QSO_Fix = 0;
		break;

	case 5: // Toggle synchonisation mode
		if (!sButtonData[5].state)
			Auto_Sync = 0;
		else
			Auto_Sync = 1;
		break;

	case 6: // Lower Gain
		if (AGC_Gain >= 3)
			AGC_Gain--;
		show_short(405, 255, AGC_Gain);
		Set_PGA_Gain(AGC_Gain);
		break;

	case 7: // Raise Gain
		if (AGC_Gain < 31)
			AGC_Gain++;
		show_short(405, 255, AGC_Gain);
		Set_PGA_Gain(AGC_Gain);
		break;

	case 8: // Lower Freq
		if (cursor > 0)
		{
			cursor--;
			NCO_Frequency = (double)(cursor + ft8_min_bin) * FFT_Resolution;
		}
		show_variable(400, 25, (int)NCO_Frequency);
		break;

	case 9: // Raise Freq
		if (cursor < (ft8_buffer - ft8_min_bin - 8))
		{ 
			// limits highest NCO frequency to (3875 - 50)hz
			cursor++;
			NCO_Frequency = (double)(cursor + ft8_min_bin) * FFT_Resolution;
		}
		show_variable(400, 25, (int)NCO_Frequency);
		break;

	case 12: // Transmit for Calibration
		if (!sButtonData[12].state)
		{
			tune_Off_sequence();
			Arm_Tune = 0;
			xmit_flag = 0;
			receive_sequence();
		}
		else
		{
			xmit_sequence();
			HAL_Delay(10);
			xmit_flag = 1;
			tune_On_sequence();
			Arm_Tune = 1;
		}
		break;

	case 13: // Save Band Changes
		Options_SetValue(0, BandIndex);
		Options_StoreValue(0);
		start_freq = sBand_Data[BandIndex].Frequency;
		show_wide(380, 0, start_freq);

		sprintf(display_frequency, "%s", sBand_Data[BandIndex].display);

		set_Rcvr_Freq();
		HAL_Delay(10);

		SelectFilterBlock();

		toggle_button_state(13);
		break;

	case 14: // Edit RTC time
		set_RTC_to_TimeEdit();
		toggle_button_state(14);
		break;

	case 27: // Edit RTC date
		set_RTC_to_DateEdit();
		toggle_button_state(27);
		break;

	case 29: // Standard CQ
		if (sButtonData[29].state == 1){
			CQ_Mode_Index = 0;
			sButtonData[30].state = 0;drawButton(30);
			sButtonData[31].state = 0;drawButton(31);
			sButtonData[32].state = 0;drawButton(32);
			sButtonData[33].state = 0;drawButton(33);
			sButtonData[3].text0 = " CQ ";drawButton(3);
		}
		break;

	case 30: // CQ DX
		if (sButtonData[30].state == 1){
			CQ_Mode_Index = 1;
			sButtonData[29].state = 0;drawButton(29);
			sButtonData[31].state = 0;drawButton(31);
			sButtonData[32].state = 0;drawButton(32);
			sButtonData[33].state = 0;drawButton(33);
			sButtonData[3].text0 = " DX ";drawButton(3);
		}
		break;
	case 31: // CQ POTA
		if (sButtonData[31].state == 1){
			CQ_Mode_Index = 2;
			sButtonData[30].state = 0;drawButton(30);
			sButtonData[29].state = 0;drawButton(29);
			sButtonData[32].state = 0;drawButton(32);
			sButtonData[33].state = 0;drawButton(33);
			sButtonData[3].text0 = "POTA";drawButton(3);
		}
		break;

	case 32: // CQ SOTA
		if (sButtonData[32].state == 1){
			CQ_Mode_Index = 3;
			sButtonData[30].state = 0;drawButton(30);
			sButtonData[31].state = 0;drawButton(31);
			sButtonData[29].state = 0;drawButton(29);
			sButtonData[33].state = 0;drawButton(33);
			sButtonData[3].text0 = "SOTA";drawButton(3);
		}
		break;

	case 33: // CQ QRPP
		if (sButtonData[33].state == 1){
			CQ_Mode_Index = 4;
			sButtonData[30].state = 0;drawButton(30);
			sButtonData[31].state = 0;drawButton(31);
			sButtonData[29].state = 0;drawButton(29);
			sButtonData[32].state = 0;drawButton(32);
			sButtonData[3].text0 = "QRPP";drawButton(3);
		}
		break;
	case 34: // Free Text 1
		if (sButtonData[34].state == 1){
			if(sButtonData[37].state == 1) {sButtonData[37].state = 0;drawButton(37);}
			if(sButtonData[35].state == 1) {sButtonData[35].state = 0;drawButton(35);}
			if(sButtonData[36].state == 1) {sButtonData[36].state = 0;drawButton(36);}
            Free_Index = 1;
		}
		else
			Free_Index = 0;
		break;

	case 35: // Free Text 2

		if (sButtonData[35].state == 1){
			if(sButtonData[34].state == 1) {sButtonData[34].state = 0;drawButton(34);}
			if(sButtonData[37].state == 1) {sButtonData[37].state = 0;drawButton(37);}
			if(sButtonData[36].state == 1) {sButtonData[36].state = 0;drawButton(36);}
			Free_Index = 2;
		}
		else
			Free_Index = 0;

		break;
	case 36: // Free Text 3

		if (sButtonData[36].state == 1){
			if(sButtonData[37].state == 1) {sButtonData[37].state = 0;drawButton(37);}
			if(sButtonData[34].state == 1) {sButtonData[34].state = 0;drawButton(34);}
			if(sButtonData[35].state == 1) {sButtonData[35].state = 0;drawButton(35);}
			Free_Index = 3;
		}
		else
			Free_Index = 0;

		break;
	case 37: // Free Text 4

		if (sButtonData[37].state == 1){
			if(sButtonData[34].state == 1) {sButtonData[34].state = 0;drawButton(34);}
			if(sButtonData[35].state == 1) {sButtonData[35].state = 0;drawButton(35);}
			if(sButtonData[36].state == 1) {sButtonData[36].state = 0;drawButton(36);}
			Free_Index = 4;
		}
		else
			Free_Index = 0;

		break;

	case 38: // Skip Grid
		if (sButtonData[38].state == 1)
			SkipGrid = 1;
		else
			SkipGrid = 0;
		break;

	case 39: // Enable Edit
		if (sButtonData[39].state == 1)
			EnableKeyboard();
		else
			DisableKeyboard();
		break;

	case 40: AppendChar(Free_Text4, 'A'); UpdateFreeText4(); toggle_button_state(40); break;
	case 41: AppendChar(Free_Text4, 'B'); UpdateFreeText4(); toggle_button_state(41); break;
	case 42: AppendChar(Free_Text4, 'C'); UpdateFreeText4(); toggle_button_state(42); break;
	case 43: AppendChar(Free_Text4, 'D'); UpdateFreeText4(); toggle_button_state(43); break;
	case 44: AppendChar(Free_Text4, 'E'); UpdateFreeText4(); toggle_button_state(44); break;
	case 45: AppendChar(Free_Text4, 'F'); UpdateFreeText4(); toggle_button_state(45); break;
	case 46: AppendChar(Free_Text4, 'G'); UpdateFreeText4(); toggle_button_state(46); break;
	case 47: AppendChar(Free_Text4, 'H'); UpdateFreeText4(); toggle_button_state(47); break;
	case 48: AppendChar(Free_Text4, 'I'); UpdateFreeText4(); toggle_button_state(48); break;
                                                                                       
	case 49: AppendChar(Free_Text4, 'J'); UpdateFreeText4(); toggle_button_state(49); break;
	case 50: AppendChar(Free_Text4, 'K'); UpdateFreeText4(); toggle_button_state(50); break;
	case 51: AppendChar(Free_Text4, 'L'); UpdateFreeText4(); toggle_button_state(51); break;
	case 52: AppendChar(Free_Text4, 'M'); UpdateFreeText4(); toggle_button_state(52); break;
	case 53: AppendChar(Free_Text4, 'N'); UpdateFreeText4(); toggle_button_state(53); break;
	case 54: AppendChar(Free_Text4, 'O'); UpdateFreeText4(); toggle_button_state(54); break;
	case 55: AppendChar(Free_Text4, 'P'); UpdateFreeText4(); toggle_button_state(55); break;
	case 56: AppendChar(Free_Text4, 'Q'); UpdateFreeText4(); toggle_button_state(56); break;
	case 57: AppendChar(Free_Text4, 'R'); UpdateFreeText4(); toggle_button_state(57); break;
                                                                                       
	case 58: AppendChar(Free_Text4, 'S'); UpdateFreeText4(); toggle_button_state(58); break;
	case 59: AppendChar(Free_Text4, 'T'); UpdateFreeText4(); toggle_button_state(59); break;
	case 60: AppendChar(Free_Text4, 'U'); UpdateFreeText4(); toggle_button_state(60); break;
	case 61: AppendChar(Free_Text4, 'V'); UpdateFreeText4(); toggle_button_state(61); break;
	case 62: AppendChar(Free_Text4, 'W'); UpdateFreeText4(); toggle_button_state(62); break;
	case 63: AppendChar(Free_Text4, 'X'); UpdateFreeText4(); toggle_button_state(63); break;
	case 64: AppendChar(Free_Text4, 'Y'); UpdateFreeText4(); toggle_button_state(64); break;
	case 65: AppendChar(Free_Text4, 'Z'); UpdateFreeText4(); toggle_button_state(65); break;
	case 66: AppendChar(Free_Text4, '0'); UpdateFreeText4(); toggle_button_state(66); break;
	                                                                               
	case 67: AppendChar(Free_Text4, '1'); UpdateFreeText4(); toggle_button_state(67); break;
	case 68: AppendChar(Free_Text4, '2'); UpdateFreeText4(); toggle_button_state(68); break;
	case 69: AppendChar(Free_Text4, '3'); UpdateFreeText4(); toggle_button_state(69); break;
	case 70: AppendChar(Free_Text4, '4'); UpdateFreeText4(); toggle_button_state(70); break;
	case 71: AppendChar(Free_Text4, '5'); UpdateFreeText4(); toggle_button_state(71); break;
	case 72: AppendChar(Free_Text4, '6'); UpdateFreeText4(); toggle_button_state(72); break;
	case 73: AppendChar(Free_Text4, '7'); UpdateFreeText4(); toggle_button_state(73); break;
	case 74: AppendChar(Free_Text4, '8'); UpdateFreeText4(); toggle_button_state(74); break;
	case 75: AppendChar(Free_Text4, '9'); UpdateFreeText4(); toggle_button_state(75); break;
                                                                                       
	case 76: AppendChar(Free_Text4, '+'); UpdateFreeText4(); toggle_button_state(76); break;
	case 77: AppendChar(Free_Text4, '-'); UpdateFreeText4(); toggle_button_state(77); break;
	case 78: AppendChar(Free_Text4, '.'); UpdateFreeText4(); toggle_button_state(78); break;
	case 79: AppendChar(Free_Text4, '/'); UpdateFreeText4(); toggle_button_state(79); break;
	case 80: AppendChar(Free_Text4, '?'); UpdateFreeText4(); toggle_button_state(80); break;
	case 81: AppendChar(Free_Text4, ' '); UpdateFreeText4(); toggle_button_state(81); break;
	case 82: DeleteLastChar(Free_Text4);  UpdateFreeText4(); toggle_button_state(82); break;

	}
}

static void processButton(int id, int isIncrement, int isDate)
{
	RTCStruct *data = &s_RTC_Data[id];
	if (isIncrement ? data->data < data->Maximum : data->data > data->Minimum)
	{
		data->data = isIncrement ? data->data + 1 : data->data - 1;
	}
	else
	{
		data->data = isIncrement ? data->Minimum : data->Maximum;
	}
	isDate ? display_RTC_DateEdit(RTC_Button - 20, RTC_line3 + 15) : display_RTC_TimeEdit(RTC_Button - 20, RTC_line0 + 15);
}

void executeCalibrationButton(uint16_t index)
{
	switch (index)
	{
	case 10: // Lower Band
		if (BandIndex >= Band_Minimum)
		{
			if(sBand_Data[BandIndex].Frequency != sBand_Data_default[BandIndex].Frequency){
				sBand_Data[BandIndex].Frequency = sBand_Data_default[BandIndex].Frequency;
				sBand_Data[BandIndex].display = sBand_Data_default[BandIndex].display;
			}
			else if(BandIndex > Band_Minimum) BandIndex--;

			show_wide(340, 55, sBand_Data[BandIndex].Frequency);
			sprintf(display_frequency, "%s", sBand_Data[BandIndex].display);
		}
		break;

	case 11: // Raise Band
		if (BandIndex <= _10M)
		{
			if(sBand_Data[BandIndex].Frequency != sBand_Data_external[BandIndex].Frequency &&
					sBand_Data_external[BandIndex].Frequency != 0){
				sBand_Data[BandIndex].Frequency = sBand_Data_external[BandIndex].Frequency;
				sBand_Data[BandIndex].display = sBand_Data_external[BandIndex].display;
			}
			else if(BandIndex < _10M) BandIndex++;

			show_wide(340, 55, sBand_Data[BandIndex].Frequency);
			sprintf(display_frequency, "%s", sBand_Data[BandIndex].display);
		}
		break;

	case 15: // Lower Hour
		processButton(3, 0, 0);
		break;

	case 16: // Raise Hour
		processButton(3, 1, 0);
		break;

	case 17: // Lower Minute
		processButton(4, 0, 0);
		break;

	case 18: // Raise Minute
		processButton(4, 1, 0);
		break;

	case 19: // Lower Second
		processButton(5, 0, 0);
		break;

	case 20: // Raise Second
		processButton(5, 1, 0);
		break;

	case 21: // Lower Day
		processButton(0, 0, 1);
		break;

	case 22: // Raise Day
		processButton(0, 1, 1);
		break;

	case 23: // Lower Month
		processButton(1, 0, 1);
		break;

	case 24: // Raise Month
		processButton(1, 1, 1);
		break;

	case 25: // Lower Year
		processButton(2, 0, 1);
		break;

	case 26: // Raise Year
		processButton(2, 1, 1);
		break;

	}
}

void setup_Cal_Display(void)
{
	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	BSP_LCD_FillRect(0, FFT_H, 480, 201);

	sButtonData[10].Active = 3;
	sButtonData[11].Active = 3;

	for (int button = 12; button <= 14; ++button)
		sButtonData[button].Active = 1;

	for (int button = 15; button < 27; button++)
	{
		sButtonData[button].Active = 3;
		drawButton(button);
	}

	sButtonData[27].Active = 1;

	for (int button = 10; button <= 14; ++button)
		drawButton(button);
	drawButton(27);

	for (int button = 29; button <= 39; ++button){
		sButtonData[button].Active = 1;
		drawButton(button);
	}

	show_wide(340, 55, start_freq);

	load_RealTime();
	display_RTC_TimeEdit(RTC_Button - 20, RTC_line0 + 15);

	load_RealDate();
	display_RTC_DateEdit(RTC_Button - 20, RTC_line3 + 15);
}

void erase_Cal_Display(void)
{
	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	BSP_LCD_FillRect(0, FFT_H, 480, 201);

	for (int button = 10; button < numButtons; button++)
	{
		sButtonData[button].Active = 0;
	}

	for (int button = 29; button <= numButtons - 1; ++button){
		sButtonData[button].Active = 0;
		drawButton(button);
	}

	for (int button = 12; button <= 14; ++button)
		sButtonData[button].state = 0;

	sButtonData[27].state = 0;
	sButtonData[28].Active = 1;
}

void PTT_Out_Init(void)
{
	GPIO_InitTypeDef gpio_init_structure;

	__HAL_RCC_GPIOI_CLK_ENABLE();

	gpio_init_structure.Pin = GPIO_PIN_2; // D8  RXSW
	gpio_init_structure.Mode = GPIO_MODE_OUTPUT_OD;
	gpio_init_structure.Pull = GPIO_PULLUP;
	gpio_init_structure.Speed = GPIO_SPEED_HIGH;

	HAL_GPIO_Init(GPIOI, &gpio_init_structure);

	HAL_GPIO_WritePin(GPIOI, GPIO_PIN_2, GPIO_PIN_SET); // Set = Receive connect

	__HAL_RCC_GPIOA_CLK_ENABLE();

	gpio_init_structure.Pin = GPIO_PIN_15; // D9 TXSW
	gpio_init_structure.Mode = GPIO_MODE_OUTPUT_OD;
	gpio_init_structure.Pull = GPIO_PULLUP;
	gpio_init_structure.Speed = GPIO_SPEED_HIGH;

	HAL_GPIO_Init(GPIOA, &gpio_init_structure);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET); // Set = Receive short
}

void Init_BoardVersionInput(void)
{
	GPIO_InitTypeDef gpio_init_structure;

	__HAL_RCC_GPIOH_CLK_ENABLE();

	gpio_init_structure.Pin = GPIO_PIN_6; // D6  BTS
	gpio_init_structure.Mode = GPIO_MODE_INPUT;
	gpio_init_structure.Pull = GPIO_PULLUP;
	gpio_init_structure.Speed = GPIO_SPEED_HIGH;

	HAL_GPIO_Init(GPIOH, &gpio_init_structure);

	HAL_GPIO_WritePin(GPIOH, GPIO_PIN_6, GPIO_PIN_RESET); // Set = Receive connect
}

void DeInit_BoardVersionInput(void)
{
	HAL_GPIO_DeInit(GPIOH, GPIO_PIN_6);
}

void PTT_Out_Set(void)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOI, GPIO_PIN_2, GPIO_PIN_SET);
}

void PTT_Out_RST_Clr(void)
{
	HAL_GPIO_WritePin(GPIOI, GPIO_PIN_2, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
}

void RLY_Select_20to40(void)
{
	HAL_GPIO_WritePin(GPIOI, GPIO_PIN_3, GPIO_PIN_SET);
}

void RLY_Select_10to17(void)
{
	HAL_GPIO_WritePin(GPIOI, GPIO_PIN_3, GPIO_PIN_RESET);
}

static void Init_BandSwitchOutput(void)
{
	GPIO_InitTypeDef gpio_init_structure;

	gpio_init_structure.Pin = GPIO_PIN_3; // D7  RLY
	gpio_init_structure.Mode = GPIO_MODE_OUTPUT_OD;
	gpio_init_structure.Pull = GPIO_PULLUP;
	gpio_init_structure.Speed = GPIO_SPEED_HIGH;

	HAL_GPIO_Init(GPIOI, &gpio_init_structure);

	HAL_GPIO_WritePin(GPIOI, GPIO_PIN_3, GPIO_PIN_RESET);
}

void Check_Board_Version(void)
{
	Band_Minimum = _20M;

	// GPIO Pin 6 is grounded for new model
	if (HAL_GPIO_ReadPin(GPIOH, GPIO_PIN_6) == 0)
	{
		Init_BandSwitchOutput();

		Band_Minimum = _40M;
	}

	Options_SetMinimum(Band_Minimum);
}

void set_codec_input_gain(void)
{
	Set_PGA_Gain(AGC_Gain);
	HAL_Delay(10);
	Set_ADC_DVC(190);
}

void receive_sequence(void)
{
	PTT_Out_Set(); // set output high to connect receiver to antenna
	HAL_Delay(10);
	sButtonData[28].state = 0;
	drawButton(28);
}

void xmit_sequence(void)
{
	PTT_Out_RST_Clr(); // set output low to disconnect receiver from antenna
	HAL_Delay(10);
	sButtonData[28].state = 1;
	drawButton(28);
}

const uint64_t F_boot = 11229600000ULL;

void start_Si5351(void)
{
	init(SI5351_CRYSTAL_LOAD_0PF, 0);
	drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA);
	drive_strength(SI5351_CLK1, SI5351_DRIVE_2MA);
	drive_strength(SI5351_CLK2, SI5351_DRIVE_2MA);
	set_freq(F_boot, SI5351_CLK1);
	HAL_Delay(10);
	output_enable(SI5351_CLK1, 1);
	HAL_Delay(20);
	set_Rcvr_Freq();
}

void FT8_Sync(void)
{
	start_time = HAL_GetTick();
	ft8_flag = 1;
	FT_8_counter = 0;
	ft8_marker = 1;
}

void EnableKeyboard(void)
{
	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	BSP_LCD_FillRect(240, 70, 239, 150);

	//disable buttons
	sButtonData[12].Active = 0;
	sButtonData[13].Active = 0;
	sButtonData[38].Active = 0;
	for(int i=29; i<37; i++) sButtonData[i].Active = 0;

	//Enable Keyboard
	for(int i=40; i<83; i++) {
		sButtonData[i].Active = 2;
		drawButton(i);
	}
}

void DisableKeyboard(void)
{
	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	BSP_LCD_FillRect(240, 70, 239, 150);

	//disable keyboard
	for(int i=40; i<83; i++) {
		sButtonData[i].Active = 0;

	}

	//Enable buttons
	sButtonData[12].Active = 1;
	sButtonData[13].Active = 1;
	sButtonData[38].Active = 1;
	drawButton(12);
	drawButton(13);
	drawButton(38);

	for(int i=29; i<37; i++) {
		sButtonData[i].Active = 1;
		drawButton(i);
	}
}

void AppendChar(char *str, char c){
	int len = strlen(str);
	if(len < 19) {
		str[len] = c;
		str[len+1] = '\0';
	}
}

void DeleteLastChar(char *str){
	int len = strlen(str);
	if(len > 0) str[len-1] = '\0';
}

void UpdateFreeText4(void) {
	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	BSP_LCD_FillRect(256, 220, 223, 30);
	sButtonData[37].text0 = Free_Text4;
	sButtonData[37].text1 = Free_Text4;
	drawButton(37);
}
