/*
 * decode_ft8.h
 *
 *  Created on: Nov 2, 2019
 *      Author: user
 */

#ifndef DECODE_FT8_H_
#define DECODE_FT8_H_

extern int Auto_QSO_State;
extern char Locator[5];          // four character locator  + /0
extern char Station_Call[7];     // six character call sign + /0
extern char Target_Call[7];      // six character call sign + /0
extern char Target_Locator[5];   // four character locator  + /0
extern char RSL[5];
extern int Station_RSL;
extern int Target_RSL;

typedef struct
{
    char call_to[14];   // call also be 'CQ'
    char call_from[14];
    char locator[7];    // can also be a response 'RR73' etc.
    int freq_hz;
    int sync_score;
    int snr;
    int received_snr;
    char target_locator[7];
    int slot;
    int RR73;
    int sequence;
} Decode;

typedef struct
{
    char message[40];
    int text_color;
} display_message;

typedef struct
{
    int number_times_called;
    char call[14];
    char locator[7];
    int RSL;
    int received_RSL;
    int RR73;

} Calling_Station;

int Check_Calling_Stations(int num_decoded);
void display_messages(int decoded_messages);
void process_selected_Station(int stations_decoded, int TouchIndex);
void clear_log_stored_data(void);
void set_QSO_Xmit_Freq(int freq);
void clear_decoded_messages(void);

#endif /* DECODE_FT8_H_ */
