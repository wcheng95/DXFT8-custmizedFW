/*
 * decode_ft8.c
 *
 *  Created on: Sep 16, 2019
 *      Author: user
 */

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <ctype.h>
#include <Display.h>
#include <gen_ft8.h>

#include "unpack.h"
#include "ldpc.h"
#include "decode.h"
#include "constants.h"
#include "encode.h"
#include "button.h"
#include "main.h"
#include "text.h"

#include "Process_DSP.h"
#include "stm32746g_discovery_lcd.h"
#include "log_file.h"
#include "decode_ft8.h"
#include "traffic_manager.h"
#include "ADIF.h"
#include "DS3231.h"

#include "ff.h"		/* Declarations of FatFs API */
#include "diskio.h" /* Declarations of device I/O functions */
#include "stdio.h"
#include "stm32746g_discovery_sd.h"
#include "stm32746g_discovery.h"

const int kLDPC_iterations = 20;
const int kMax_candidates = 20;
const int kMax_decoded_messages = 20; // chhh 27 feb
const unsigned int kMax_message_length = 20;
const int kMin_score = 40; // Minimum sync score threshold for candidates

static int validate_locator(const char locator[]);
static int strindex(const char s[], const char t[]);

static Decode new_decoded[20]; // chh 27 Feb
extern char current_QSO_receive_message[];
extern char current_Beacon_receive_message[];

static display_message display[10];

static Calling_Station Answer_CQ[50]; //
static int log_size = 50;

static int num_calls; // number of unique calling stations
static int message_limit = 10;

int Auto_QSO_State; // chh
int Target_RSL;

static FATFS FS;
static FIL AllLog;
extern char rtc_date_string[9];
extern char rtc_time_string[9];

int ft8_decode(void)
{
	// Find top candidates by Costas sync score and localize them in time and frequency
	Candidate candidate_list[kMax_candidates];

	int num_candidates = find_sync(export_fft_power, ft8_msg_samples, ft8_buffer, kCostas_map, kMax_candidates, candidate_list, kMin_score);
	char decoded[kMax_decoded_messages][kMax_message_length];

	const float fsk_dev = 6.25f; // tone deviation in Hz and symbol rate

	// Go over candidates and attempt to decode messages
	int num_decoded = 0;

	for (int idx = 0; idx < num_candidates; ++idx)
	{
		Candidate cand = candidate_list[idx];
		float freq_hz = ((float)cand.freq_offset + (float)cand.freq_sub / 2.0f) * fsk_dev;

		float log174[N];
		extract_likelihood(export_fft_power, ft8_buffer, cand, kGray_map, log174);

		// bp_decode() produces better decodes, uses way less memory
		uint8_t plain[N];
		int n_errors = 0;
		bp_decode(log174, kLDPC_iterations, plain, &n_errors);

		if (n_errors > 0)
			continue;

		// Extract payload + CRC (first K bits)
		uint8_t a91[K_BYTES];
		pack_bits(plain, K, a91);

		// Extract CRC and check it
		uint16_t chksum = ((a91[9] & 0x07) << 11) | (a91[10] << 3) | (a91[11] >> 5);
		a91[9] &= 0xF8;
		a91[10] = 0;
		a91[11] = 0;
		uint16_t chksum2 = crc(a91, 96 - 14);
		if (chksum != chksum2)
			continue;

		char message[14 + 14 + 7 + 1];

		char call_to[14];
		char call_from[14];
		char locator[7];
		int rc = unpack77_fields(a91, call_to, call_from, locator);
		if (rc < 0)
			continue;

		sprintf(message, "%s %s %s ", call_to, call_from, locator);

		_Bool found = false;
		for (int i = 0; i < num_decoded; ++i)
		{
			if (0 == strcmp(decoded[i], message))
			{
				found = true;
				break;
			}
		}

		float raw_RSL;
		int display_RSL;
		int received_RSL;

		if (!found && num_decoded < kMax_decoded_messages)
		{
			if (strlen(message) < kMax_message_length)
			{
				strcpy(decoded[num_decoded], message);

				new_decoded[num_decoded].sync_score = cand.score;
				new_decoded[num_decoded].freq_hz = (int)freq_hz;

				strcpy(new_decoded[num_decoded].call_to, call_to);
				strcpy(new_decoded[num_decoded].call_from, call_from);
				strcpy(new_decoded[num_decoded].locator, locator);

				new_decoded[num_decoded].slot = slot_state;

				raw_RSL = (float)cand.score;
				display_RSL = (int)((raw_RSL - 160)) / 6;
				new_decoded[num_decoded].snr = display_RSL;

				if (validate_locator(locator) == 1)
				{
					strcpy(new_decoded[num_decoded].target_locator, locator);
					new_decoded[num_decoded].sequence = 1;
				}

				else if (strindex(locator, "73") >= 0 || strindex(locator, "RR73") >= 0 || strindex(locator, "RRR") >= 0)
				{
					new_decoded[num_decoded].RR73 = 1;
					new_decoded[num_decoded].sequence = 4;
				}
				else
				{
					if (locator[0] == 82)
					{
						locator[0] = 32;
						new_decoded[num_decoded].RR73 = 1;
						new_decoded[num_decoded].sequence = 3;
					}

					received_RSL = atoi(locator);
					if (received_RSL < 30) // Prevents an 73 being decoded as a received RSL
					{
						new_decoded[num_decoded].received_snr = received_RSL;
						new_decoded[num_decoded].sequence = 2;
					}
				}

				++num_decoded;
			}
		}
	} // End of big decode loop

	return num_decoded;
}

void display_messages(int decoded_messages)
{
	const char CQ[] = "CQ";

	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	BSP_LCD_FillRect(0, FFT_H, 264, 200);
	BSP_LCD_SetFont(&Font16);

	for (int i = 0; i < decoded_messages && i < message_limit; i++)
	{
		const char *call_to = new_decoded[i].call_to;
		const char *call_from = new_decoded[i].call_from;
		const char *locator = new_decoded[i].locator;

		//if (strcmp(CQ, call_to) == 0)
		if(starts_with(call_to, CQ))
		{
			if (strcmp(Station_Call, call_from) != 0)
			{
				sprintf(display[i].message, "%s %s %s %2i %d", call_to, call_from, locator, new_decoded[i].snr, new_decoded[i].freq_hz);
			}
			else
			{
				sprintf(display[i].message, "%s %s %s %d", call_to, call_from, locator, new_decoded[i].freq_hz);
			}
			display[i].text_color = 1;
		}
		else
		{
			sprintf(display[i].message, "%s %s %s %4d", call_to, call_from, locator, new_decoded[i].freq_hz);
			display[i].text_color = 0;
		}

	}


	//f_mount(&FS, "SD:", 1);
	if(f_open(&AllLog, "AllLog.txt", FA_OPEN_ALWAYS | FA_WRITE | FA_OPEN_APPEND) == FR_OK){
		char myLog[64];
		char *myBand[] = {"40m", "30m", "20m", "17m", "15m", "12m", "10m"};
		f_lseek(&AllLog, f_size(&AllLog));
		for (int i = 0; i < decoded_messages && i < message_limit; i++){
			sprintf(myLog, "[%s %s][%s] %s %s %s %2i %d\n", rtc_date_string, rtc_time_string, myBand[BandIndex], new_decoded[i].call_to, new_decoded[i].call_from, new_decoded[i].locator, new_decoded[i].snr, new_decoded[i].freq_hz);
			f_puts(myLog, &AllLog);
		}
		f_close(&AllLog);
	}

	for (int j = 0; j < decoded_messages && j < message_limit; j++)
	{
		if (display[j].text_color == 0)
			BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
		else
			BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
		BSP_LCD_DisplayStringAt(0, 40 + j * 20, (const uint8_t *)display[j].message, LEFT_MODE);
	}
}

void clear_messages(void)
{
	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	BSP_LCD_FillRect(0, FFT_H, 264, 201);
}

static int validate_locator(const char locator[])
{
	uint8_t A1, A2, N1, N2;
	uint8_t test = 0;

	A1 = locator[0] - 65;
	A2 = locator[1] - 65;
	N1 = locator[2] - 48;
	N2 = locator[3] - 48;

	if (A1 <= 17)
		test++;
	if (A2 > 0 && A2 < 17)
		test++; // block RR73 Arctic and Antarctica
	if (N1 <= 9)
		test++;
	if (N2 <= 9)
		test++;

	if (test == 4)
		return 1;
	else
		return 0;
}

void clear_log_stored_data(void)
{
	const char call_blank[] = "       ";
	const char locator_blank[] = "    ";

	for (int i = 0; i < log_size; i++)
	{
		Answer_CQ[i].number_times_called = 0;
		strcpy(Answer_CQ[i].call, call_blank);
		strcpy(Answer_CQ[i].locator, locator_blank);
		Answer_CQ[i].RSL = 0;
		Answer_CQ[i].RR73 = 0;
		Answer_CQ[i].received_RSL = 99;
	}
}

void clear_decoded_messages(void)
{
	const char call_blank[] = "       ";
	const char locator_blank[] = "    ";

	for (int i = 0; i < kMax_decoded_messages; i++)
	{
		strcpy(new_decoded[i].call_to, call_blank);
		strcpy(new_decoded[i].call_from, call_blank);
		strcpy(new_decoded[i].locator, locator_blank);
		strcpy(new_decoded[i].target_locator, locator_blank);
		new_decoded[i].freq_hz = 0;
		new_decoded[i].sync_score = 0;
		new_decoded[i].received_snr = 99;
		new_decoded[i].slot = 0;
		new_decoded[i].RR73 = 0;
	}
}

int Check_Calling_Stations(int num_decoded)
{
	int Beacon_Reply_Status = 0;
	for (int i = 0; i < num_decoded; i++)
	{ 
		// check to see if being called
		int old_call;
		int old_call_address;

		//if (strindex(new_decoded[i].call_to, Station_Call) >= 0 && isValidCallsign(new_decoded[i].call_from))
		if (strindex(new_decoded[i].call_to, Station_Call) >= 0 )

		{
			old_call = 0;
			for (int j = 0; j < num_calls; j++)
			{
				if (strcmp(Answer_CQ[j].call, new_decoded[i].call_from) == 0)
				{
					old_call = Answer_CQ[j].number_times_called;
					old_call++;
					Answer_CQ[j].number_times_called = old_call;
					old_call_address = j;
				}
			}

			const char *call_to = new_decoded[i].call_to;
			const char *call_from = new_decoded[i].call_from;
			const char *locator = new_decoded[i].locator;

			if (old_call == 0)
			{
				sprintf(current_Beacon_receive_message, "%s %s %s", call_to, call_from, locator);
				strcpy(current_QSO_receive_message, current_Beacon_receive_message);

				if (Beacon_On == 1)
					update_Beacon_log_display(0);
				else
				if (Beacon_On == 0)
					update_log_display(0);

				strcpy(Target_Call, call_from);

				if (Beacon_On == 1)
					Target_RSL = new_decoded[i].snr;

				if (new_decoded[i].received_snr != 99)
					Station_RSL = new_decoded[i].received_snr;

				if (Beacon_On == 1){
					if(new_decoded[i].sequence == 2) set_reply(2);
					else set_reply(0);
				}

				Beacon_Reply_Status = 1;

				strcpy(Answer_CQ[num_calls].call, call_from);
				strcpy(Answer_CQ[num_calls].locator, new_decoded[i].target_locator);
				Answer_CQ[num_calls].RSL = Target_RSL;
				Answer_CQ[num_calls].received_RSL = Station_RSL;

				num_calls++;
				break;
			}
			if (old_call >= 1 && old_call < 5)
			{
				sprintf(current_Beacon_receive_message, "%s %s %s", call_to, call_from, locator);
				strcpy(current_QSO_receive_message, current_Beacon_receive_message);

				if (Beacon_On == 1)
					update_Beacon_log_display(0);
				else
				if (Beacon_On == 0)
					update_log_display(0);

				if (new_decoded[i].RR73 == 1)
					RR73_sent = 1;

				strcpy(Target_Call, Answer_CQ[old_call_address].call);
				strcpy(Target_Locator, Answer_CQ[old_call_address].locator);

				Target_RSL = Answer_CQ[old_call_address].RSL;

				if (new_decoded[i].received_snr != 99)
					Station_RSL = new_decoded[i].received_snr;
				else
					Station_RSL = Answer_CQ[old_call_address].received_RSL;

				if (Answer_CQ[old_call_address].RR73 == 0)
				{
					if (Beacon_On == 1)
					{
						if (new_decoded[i].RR73 == 1)
						{
							if(new_decoded[i].sequence == 4) set_reply(3);
							else set_reply(1);

							Answer_CQ[old_call_address].RR73 = 1;
						}
						else {
							if(new_decoded[i].sequence == 2) set_reply(2);
							else set_reply(0);
						}
					}

					Beacon_Reply_Status = 1;
				} // Check for RR73 =1

			} // check for old call
		} // check for station call

	} // check to see if being called

	return Beacon_Reply_Status;
}

void process_selected_Station(int stations_decoded, int TouchIndex)
{
	if (stations_decoded > 0 && TouchIndex <= stations_decoded)
	{
		strcpy(Target_Call, new_decoded[TouchIndex].call_from);
		strcpy(Target_Locator, new_decoded[TouchIndex].target_locator);
		Target_RSL = new_decoded[TouchIndex].snr;
		target_slot = new_decoded[TouchIndex].slot;
		target_freq = new_decoded[TouchIndex].freq_hz;

		if (QSO_Fix == 1)
			set_QSO_Xmit_Freq(target_freq);

		compose_messages();
		if((new_decoded[TouchIndex].RR73 == 1) && (0 == strcmp(new_decoded[TouchIndex].call_to, Station_Call))) {
			new_decoded[TouchIndex].RR73 = 2;
			que_message(2);
			QSO_xmit = 1;
		}
		else {
			//if(SkipGrid) que_message(1);
			//else que_message(0);
			//QSO_xmit = 1;
			//QSO_xmit_count++;

			Auto_QSO_State = 1;
			RSL_sent = 0;
			RR73_sent = 0;
		}
	}

	FT8_Touch_Flag = 0;
}

void set_QSO_Xmit_Freq(int freq)
{
	freq = freq - ft8_min_freq;
	cursor = (uint16_t)((float)freq / FFT_Resolution);

	Set_Cursor_Frequency();
	show_variable(400, 25, (int)NCO_Frequency);
}

static int strindex(const char s[], const char t[])
{
	int i, j, k, result;

	result = -1;

	for (i = 0; s[i] != '\0'; i++)
	{
		for (j = i, k = 0; t[k] != '\0' && s[j] == t[k]; j++, k++)
			;
		if (k > 0 && t[k] == '\0')
			result = i;
	}
	return result;
}
