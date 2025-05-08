
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stdio.h"

#include "stm32746g_discovery.h"
#include "stm32f7xx_hal.h"

#define NoOp  __NOP()

extern uint32_t current_time, start_time, ft8_time;

extern int QSO_xmit;
extern int Touch_xmit;
extern int Xmit_DSP_counter;

extern int slot_state;
extern int target_slot;
extern int target_freq;

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
