/**
 ******************************************************************************
 * @file    stm32746g_discovery_ts.c
 * @author  MCD Application Team
 * @version V1.0.0
 * @date    25-June-2015
 * @brief   This file provides a set of functions needed to manage the Touch
 *          Screen on STM32746G-Discovery board.
 @verbatim
 1. How To use this driver:
 --------------------------
 - This driver is used to drive the touch screen module of the STM32746G-Discovery
 board on the RK043FN48H-CT672B 480x272 LCD screen with capacitive touch screen.
 - The FT5336 component driver must be included in project files according to
 the touch screen driver present on this board.

 2. Driver description:
 ---------------------
 + Initialization steps:
 o Initialize the TS module using the BSP_TS_Init() function. This
 function includes the MSP layer hardware resources initialization and the
 communication layer configuration to start the TS use. The LCD size properties
 (x and y) are passed as parameters.
 o If TS interrupt mode is desired, you must configure the TS interrupt mode
 by calling the function BSP_TS_ITConfig(). The TS interrupt mode is generated
 as an external interrupt whenever a touch is detected.
 The interrupt mode internally uses the IO functionalities driver driven by
 the IO expander, to configure the IT line.

 + Touch screen use
 o The touch screen state is captured whenever the function BSP_TS_GetState() is
 used. This function returns information about the last LCD touch occurred
 in the TS_StateTypeDef structure.
 o If TS interrupt mode is used, the function BSP_TS_ITGetStatus() is needed to get
 the interrupt status. To clear the IT pending bits, you should call the
 function BSP_TS_ITClear().
 o The IT is handled using the corresponding external interrupt IRQ handler,
 the user IT callback treatment is implemented on the same external interrupt
 callback.
 @endverbatim
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
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
#include "stm32746g_discovery_ts.h"

/** @addtogroup BSP
 * @{
 */

/** @addtogroup STM32746G_DISCOVERY
 * @{
 */

/** @defgroup STM32746G_DISCOVERY_TS STM32746G_DISCOVERY_TS
 * @{
 */

/** @defgroup STM32746G_DISCOVERY_TS_Private_Types_Definitions STM32746G_DISCOVERY_TS Types Definitions
 * @{
 */
/**
 * @}
 */

/** @defgroup STM32746G_DISCOVERY_TS_Private_Defines STM32746G_DISCOVERY_TS Types Defines
 * @{
 */
/**
 * @}
 */

/** @defgroup STM32746G_DISCOVERY_TS_Private_Macros STM32746G_DISCOVERY_TS Private Macros
 * @{
 */
/**
 * @}
 */

/** @defgroup STM32746G_DISCOVERY_TS_Imported_Variables STM32746G_DISCOVERY_TS Imported Variables
 * @{
 */
/**
 * @}
 */

/** @defgroup STM32746G_DISCOVERY_TS_Private_Variables STM32746G_DISCOVERY_TS Private Variables
 * @{
 */
static TS_DrvTypeDef *tsDriver;
static uint16_t tsXBoundary, tsYBoundary;
static uint8_t tsOrientation;
static uint8_t I2cAddress;
/**
 * @}
 */

/** @defgroup STM32746G_DISCOVERY_TS_Private_Function_Prototypes STM32746G_DISCOVERY_TS Private Function Prototypes
 * @{
 */
/**
 * @}
 */

/** @defgroup STM32746G_DISCOVERY_TS_Exported_Functions STM32746G_DISCOVERY_TS Exported Functions
 * @{
 */

/**
 * @brief  Initializes and configures the touch screen functionalities and
 *         configures all necessary hardware resources (GPIOs, I2C, clocks..).
 * @param  ts_SizeX: Maximum X size of the TS area on LCD
 * @param  ts_SizeY: Maximum Y size of the TS area on LCD
 * @retval TS_OK if all initializations are OK. Other value if error.
 */
uint8_t BSP_TS_Init(uint16_t ts_SizeX, uint16_t ts_SizeY) {
	uint8_t status = TS_OK;
	tsXBoundary = ts_SizeX;
	tsYBoundary = ts_SizeY;

	/* Read ID and verify if the touch screen driver is ready */
	ft5336_ts_drv.Init(TS_I2C_ADDRESS);
	if (ft5336_ts_drv.ReadID(TS_I2C_ADDRESS) == FT5336_ID_VALUE) {
		/* Initialize the TS driver structure */
		tsDriver = &ft5336_ts_drv;
		I2cAddress = TS_I2C_ADDRESS;
		tsOrientation = TS_SWAP_XY;

		/* Initialize the TS driver */
		tsDriver->Start(I2cAddress);
	} else {
		status = TS_DEVICE_NOT_FOUND;
	}

	return status;
}

/**
 * @brief  DeInitializes the TouchScreen.
 * @retval TS state
 */
uint8_t BSP_TS_DeInit(void) {
	/* Actually ts_driver does not provide a DeInit function */
	return TS_OK;
}

/**
 * @brief  Configures and enables the touch screen interrupts.
 * @retval TS_OK if all initializations are OK. Other value if error.
 */
uint8_t BSP_TS_ITConfig(void) {
	GPIO_InitTypeDef gpio_init_structure;

	/* Configure Interrupt mode for SD detection pin */
	gpio_init_structure.Pin = TS_INT_PIN;
	gpio_init_structure.Pull = GPIO_NOPULL;
	gpio_init_structure.Speed = GPIO_SPEED_FAST;
	gpio_init_structure.Mode = GPIO_MODE_IT_RISING;
	HAL_GPIO_Init(TS_INT_GPIO_PORT, &gpio_init_structure);

	/* Enable and set Touch screen EXTI Interrupt to the lowest priority */
	HAL_NVIC_SetPriority((IRQn_Type) (TS_INT_EXTI_IRQn), 0x0F, 0x00);
	HAL_NVIC_EnableIRQ((IRQn_Type) (TS_INT_EXTI_IRQn));

	/* Enable the TS ITs */
	tsDriver->EnableIT(I2cAddress);

	return TS_OK;
}

/**
 * @brief  Gets the touch screen interrupt status.
 * @retval TS_OK if all initializations are OK. Other value if error.
 */
uint8_t BSP_TS_ITGetStatus(void) {
	/* Return the TS IT status */
	return (tsDriver->GetITStatus(I2cAddress));
}

/**
 * @brief  Returns status and positions of the touch screen.
 * @param  TS_State: Pointer to touch screen current state structure
 * @retval TS_OK if all initializations are OK. Other value if error.
 */
uint8_t BSP_TS_GetState(TS_StateTypeDef *TS_State) {
	static uint32_t _x[TS_MAX_NB_TOUCH] = { 0, 0 };
	static uint32_t _y[TS_MAX_NB_TOUCH] = { 0, 0 };
	uint8_t ts_status = TS_OK;
	uint16_t x[TS_MAX_NB_TOUCH];
	uint16_t y[TS_MAX_NB_TOUCH];
	uint16_t brute_x[TS_MAX_NB_TOUCH];
	uint16_t brute_y[TS_MAX_NB_TOUCH];
	uint16_t x_diff;
	uint16_t y_diff;
	uint32_t index;
#if (TS_MULTI_TOUCH_SUPPORTED == 1)
  uint32_t weight = 0;
  uint32_t area = 0;
  uint32_t event = 0;
#endif /* TS_MULTI_TOUCH_SUPPORTED == 1 */

	/* Check and update the number of touches active detected */
	TS_State->touchDetected = tsDriver->DetectTouch(I2cAddress);

	if (TS_State->touchDetected) {
		for (index = 0; index < TS_State->touchDetected; index++) {
			/* Get each touch coordinates */
			tsDriver->GetXY(I2cAddress, &(brute_x[index]), &(brute_y[index]));

			if (tsOrientation == TS_SWAP_NONE) {
				x[index] = brute_x[index];
				y[index] = brute_y[index];
			}

			if (tsOrientation & TS_SWAP_X) {
				x[index] = 4096 - brute_x[index];
			}

			if (tsOrientation & TS_SWAP_Y) {
				y[index] = 4096 - brute_y[index];
			}

			if (tsOrientation & TS_SWAP_XY) {
				y[index] = brute_x[index];
				x[index] = brute_y[index];
			}

			x_diff =
					x[index] > _x[index] ?
							(x[index] - _x[index]) : (_x[index] - x[index]);
			y_diff =
					y[index] > _y[index] ?
							(y[index] - _y[index]) : (_y[index] - y[index]);

			if ((x_diff + y_diff) > 5) {
				_x[index] = x[index];
				_y[index] = y[index];
			}

			if (I2cAddress == FT5336_I2C_SLAVE_ADDRESS) {
				TS_State->touchX[index] = x[index];
				TS_State->touchY[index] = y[index];
			} else {
				/* 2^12 = 4096 : indexes are expressed on a dynamic of 4096 */
				TS_State->touchX[index] = (tsXBoundary * _x[index]) >> 12;
				TS_State->touchY[index] = (tsYBoundary * _y[index]) >> 12;
			}

#if (TS_MULTI_TOUCH_SUPPORTED == 1)

      /* Get touch info related to the current touch */
      ft5336_TS_GetTouchInfo(I2cAddress, index, &weight, &area, &event);

      /* Update TS_State structure */
      TS_State->touchWeight[index] = weight;
      TS_State->touchArea[index]   = area;

      /* Remap touch event */
      switch(event)
      {
        case FT5336_TOUCH_EVT_FLAG_PRESS_DOWN	:
          TS_State->touchEventId[index] = TOUCH_EVENT_PRESS_DOWN;
          break;
        case FT5336_TOUCH_EVT_FLAG_LIFT_UP :
          TS_State->touchEventId[index] = TOUCH_EVENT_LIFT_UP;
          break;
        case FT5336_TOUCH_EVT_FLAG_CONTACT :
          TS_State->touchEventId[index] = TOUCH_EVENT_CONTACT;
          break;
        case FT5336_TOUCH_EVT_FLAG_NO_EVENT :
          TS_State->touchEventId[index] = TOUCH_EVENT_NO_EVT;
          break;
        default :
          ts_status = TS_ERROR;
          break;
      } /* of switch(event) */

#endif /* TS_MULTI_TOUCH_SUPPORTED == 1 */

		} /* of for(index=0; index < TS_State->touchDetected; index++) */

#if (TS_MULTI_TOUCH_SUPPORTED == 1)
    /* Get gesture Id */
    ts_status = BSP_TS_Get_GestureId(TS_State);
#endif /* TS_MULTI_TOUCH_SUPPORTED == 1 */

	} /* end of if(TS_State->touchDetected != 0) */

	return (ts_status);
}

#if (TS_MULTI_TOUCH_SUPPORTED == 1)
/**
  * @brief  Update gesture Id following a touch detected.
  * @param  TS_State: Pointer to touch screen current state structure
  * @retval TS_OK if all initializations are OK. Other value if error.
  */
uint8_t BSP_TS_Get_GestureId(TS_StateTypeDef *TS_State)
{
  uint32_t gestureId = 0;
  uint8_t  ts_status = TS_OK;

  /* Get gesture Id */
  ft5336_TS_GetGestureID(I2cAddress, &gestureId);

  /* Remap gesture Id to a TS_GestureIdTypeDef value */
  switch(gestureId)
  {
    case FT5336_GEST_ID_NO_GESTURE :
      TS_State->gestureId = GEST_ID_NO_GESTURE;
      break;
    case FT5336_GEST_ID_MOVE_UP :
      TS_State->gestureId = GEST_ID_MOVE_UP;
      break;
    case FT5336_GEST_ID_MOVE_RIGHT :
      TS_State->gestureId = GEST_ID_MOVE_RIGHT;
      break;
    case FT5336_GEST_ID_MOVE_DOWN :
      TS_State->gestureId = GEST_ID_MOVE_DOWN;
      break;
    case FT5336_GEST_ID_MOVE_LEFT :
      TS_State->gestureId = GEST_ID_MOVE_LEFT;
      break;
    case FT5336_GEST_ID_ZOOM_IN :
      TS_State->gestureId = GEST_ID_ZOOM_IN;
      break;
    case FT5336_GEST_ID_ZOOM_OUT :
      TS_State->gestureId = GEST_ID_ZOOM_OUT;
      break;
    default :
      ts_status = TS_ERROR;
      break;
  } /* of switch(gestureId) */

  return(ts_status);
}
#endif /* TS_MULTI_TOUCH_SUPPORTED == 1 */

/**
 * @brief  Clears all touch screen interrupts.
 */
void BSP_TS_ITClear(void) {
	/* Clear TS IT pending bits */
	tsDriver->ClearIT(I2cAddress);
}

/** @defgroup STM32756G_DISCOVERY_TS_Private_Functions TS Private Functions
 * @{
 */

/**
 * @brief  Function used to reset all touch data before a new acquisition
 *         of touch information.
 * @param  TS_State: Pointer to touch screen current state structure
 * @retval TS_OK if OK, TE_ERROR if problem found.
 */
uint8_t BSP_TS_ResetTouchData(TS_StateTypeDef *TS_State) {
	uint8_t ts_status = TS_ERROR;
	uint32_t index;

	if (TS_State != (TS_StateTypeDef*) NULL) {
		// Gesture and some touch support removed for performance
		TS_State->touchDetected = 0;

		for (index = 0; index < TS_MAX_NB_TOUCH; index++) {
			TS_State->touchX[index] = 0;
			TS_State->touchY[index] = 0;
		}

		ts_status = TS_OK;

	} /* of if (TS_State != (TS_StateTypeDef *)NULL) */

	return (ts_status);
}

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
