// Copyright (c) 2021 Ziga Miklosic
// All Rights Reserved
// This software is under MIT licence (https://opensource.org/licenses/MIT)
////////////////////////////////////////////////////////////////////////////////
/**
*@file      xpt2046.h
*@brief     Application layer function for XPT2046 chip
*@author    Ziga Miklosic
*@date      29.06.2021
*@version	V1.0.1
*/
////////////////////////////////////////////////////////////////////////////////
/**
*@addtogroup XPT2046_API
* @{ <!-- BEGIN GROUP -->
*
* 	Application layer interface with XPT2046 chip.
*/
////////////////////////////////////////////////////////////////////////////////

#ifndef _XPT2046_H_
#define _XPT2046_H_

#ifdef __cplusplus
 extern "C" {
#endif

////////////////////////////////////////////////////////////////////////////////
// Includes
////////////////////////////////////////////////////////////////////////////////
#include <stdint.h>
#include <stdbool.h>
#include "xpt2046_cfg.h"

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////

/**
 * 	Module version
 */
#define XPT2046_VER_MAJOR		( 1 )
#define XPT2046_VER_MINOR		( 0 )
#define XPT2046_VER_DEVELOP		( 1 )

// General status
typedef enum
{
	eXPT2046_OK = 0,
	eXPT2046_ERROR,
	eXPT2046_CAL_IN_PROGRESS,
} xpt2046_status_t;

typedef struct
{
	bool pthres;
	bool skip;
	bool debounce;
	bool median;
	bool average;
	bool iir;
	bool jitter;
} filter_config_td; 


////////////////////////////////////////////////////////////////////////////////
// Display drivers for draw calibration points
////////////////////////////////////////////////////////////////////////////////
typedef void (drv_print_str_t)   (char* str, int32_t x, int32_t y);
typedef void (drv_draw_point_t)  (int32_t x, int32_t y);
typedef void (drv_clear_point_t) (int32_t x, int32_t y);

////////////////////////////////////////////////////////////////////////////////
// Function Prototypes
////////////////////////////////////////////////////////////////////////////////
xpt2046_status_t 	xpt2046_init					(void);
void				xpt2046_register_driver 		(drv_print_str_t * ps, drv_draw_point_t * dp, drv_clear_point_t * cp);
bool				xpt2046_is_init					(void);
void 				xpt2046_hndl					(void);

void				xpt2046_set_filters				(const filter_config_td * const fc);
void				xpt2046_get_filters				(filter_config_td * const fc);

xpt2046_status_t 	xpt2046_get_touch				(uint16_t * const p_page, uint16_t * const p_col, uint16_t * const p_force, bool * const p_pressed);
xpt2046_status_t 	xpt2046_start_calibration		(void);
bool				xpt2046_is_calibrated			(void);
xpt2046_status_t	xpt2046_set_cal_factors			(const int32_t * const p_factors);
xpt2046_status_t	xpt2046_get_cal_factors			(const int32_t * p_factors);

#ifdef __cplusplus
}
#endif

#endif // _XPT2046_H_
