// Copyright (c) 2021 Ziga Miklosic
// All Rights Reserved
// This software is under MIT licence (https://opensource.org/licenses/MIT)
////////////////////////////////////////////////////////////////////////////////
/**
*@file      xpt2046.c
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

////////////////////////////////////////////////////////////////////////////////
// Includes
////////////////////////////////////////////////////////////////////////////////
#include <stdint.h>
#include <string.h>

#include "xpt2046.h"
#include "xpt2046_low_if.h"
#include "../../xpt2046_cfg.h"
#include "../../xpt2046_if.h"

// Display
//#include "drivers/devices/ili9488/ili9488/src/ili9488.h"
#include "../../gui_port.h"

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////

// Max. FSM state
#define XPT2046_LIMIT_FMS_MS					( 1000000UL ) // [ms]
#define XPT2046_LIMIT_FMS_DURATION(time)		(( time > XPT2046_LIMIT_FMS_MS ) ? ( XPT2046_LIMIT_FMS_MS ) : ( time ))

// Touch
typedef struct
{
	uint16_t 	page;
	uint16_t 	col;
	uint16_t 	force;
	bool		pressed;
} xpt2046_touch_t;

// Point
typedef struct
{
	int64_t x;
	int64_t y;
} xpt2046_point_t;

// Points
typedef enum
{
	eXPT2046_CAL_P1 = 0,
	eXPT2046_CAL_P2,
	eXPT2046_CAL_P3,

	eXPT2046_CAL_P_NUM_OF,
} xpt2046_points_t;

// Calibration data
typedef struct
{
	xpt2046_point_t 	Dp[ eXPT2046_CAL_P_NUM_OF ];	// Display points (predefined)
	xpt2046_point_t 	Tp[ eXPT2046_CAL_P_NUM_OF ];	// Touch points
	int32_t				factors[ 7 ];					// Calibration factors
	bool				start;
	bool				busy;
	bool 				done;
} xpt2046_cal_data_t;

// FSM states
typedef enum
{
	eXPT2046_FSM_NORMAL = 0,
	eXPT2046_FSM_P1_ACQ,
	eXPT2046_FSM_P2_ACQ,
	eXPT2046_FSM_P3_ACQ,
	eXPT2046_FSM_CALC_FACTORS,
} xpt2046_cal_state_t;

// Calibration FSM
typedef struct
{
	struct
	{
		uint32_t 	duration;
		bool 		first_entry;
	} time;

	struct
	{
		xpt2046_cal_state_t cur;
		xpt2046_cal_state_t next;
	} state;
} xpt2046_fsm_t;

#if ( XPT2046_FILTER_AVG_EN )

	// Filter data
	typedef struct
	{
		uint16_t samp_buf[ XPT2046_FILTER_AVG_SAMP ];
		uint32_t sum;
	} xpt2046_filt_data_t;

	// Filter objects
	typedef struct
	{
		xpt2046_filt_data_t x;
		xpt2046_filt_data_t y;
		xpt2046_filt_data_t force;
	} xpt2046_filter_t;

#endif

////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////

// Touch data
static xpt2046_touch_t g_touch;

// Calibration data
static xpt2046_cal_data_t g_cal_data =
{
	// Predefined display points
	.Dp =
	{
		XPT2046_POINT_1_XY,
		XPT2046_POINT_2_XY,
		XPT2046_POINT_3_XY,
	},

	.start = false,
	.busy = false,
	.done = false
};

// FSM handler
static xpt2046_fsm_t g_cal_fsm;

// Calibration point
ili9488_circ_attr_t g_cal_circ_attr =
{
	.position.radius	= XPT2046_POINT_SIZE,

	.border.enable		= false,
	.border.width		= 0,
	.border.color		= eILI9488_COLOR_BLACK,

	.fill.enable		= true,
};

// Initialization done flag
static bool gb_is_init = false;

////////////////////////////////////////////////////////////////////////////////
// Function prototypes
////////////////////////////////////////////////////////////////////////////////
static void 	xpt2046_read_data_from_controler	(uint16_t * const p_X, uint16_t * const p_Y, uint16_t * const p_force, bool * const p_is_pressed);
static void 	xpt2046_calibrate_data				(uint16_t * const p_X, uint16_t * const p_Y, const int32_t * const p_factors);
static void 	xpt2046_cal_hndl					(void);
static void 	xpt2046_calculate_factors			(int32_t * p_factors, const xpt2046_point_t * const p_Dp, const xpt2046_point_t * const p_Tp);
static int32_t 	xpt2046_limit_cal_Y_data			(const int32_t unlimited_data);
static int32_t 	xpt2046_limit_cal_X_data			(const int32_t unlimited_data);

static void xpt2046_fms_manager			(void);
static void xpt2046_fsm_normal			(void);
static void xpt2046_fsm_p1_acq			(void);
static void xpt2046_fsm_p2_acq			(void);
static void xpt2046_fsm_p3_acq			(void);
static void xpt2046_fsm_calc_factors	(void);

static void xpt2046_set_cal_point		(const xpt2046_points_t px);
static void xpt2046_clear_cal_point		(const xpt2046_points_t px);

static void xpt2046_filter_pthres(uint16_t * const p_X, uint16_t * const p_Y, uint16_t * const p_force, bool * const p_is_pressed);
static void xpt2046_filter_skip(uint16_t * const p_X, uint16_t * const p_Y, uint16_t * const p_force, bool * const p_is_pressed);
static void xpt2046_filter_debounce(uint16_t * const p_X, uint16_t * const p_Y, uint16_t * const p_force, bool * const p_is_pressed);
static void xpt2046_filter_median_fast(uint16_t * const p_X, uint16_t * const p_Y, uint16_t * const p_force, bool * const p_is_pressed);
static void xpt2046_filter_median(uint16_t * const p_X, uint16_t * const p_Y, uint16_t * const p_force, bool * const p_is_pressed);
static void xpt2046_filter_iir(uint16_t * const p_X, uint16_t * const p_Y, uint16_t * const p_force, bool * const p_touch);
static void xpt2046_filter_average(uint16_t * const p_X, uint16_t * const p_Y, uint16_t * const p_force, bool * const p_touch);
static void xpt2046_filter_jitter(uint16_t * const p_X, uint16_t * const p_Y, uint16_t * const p_force, bool * const p_touch);

////////////////////////////////////////////////////////////////////////////////
// Functions
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
/**
*		Touch controller initialization
*
* @warning  After initialization the "IRQ touch line" pin stays low for 6...10 microseconds.<br>
*           Before calling the "Main touch controller handler" it may be required to use a delay.
*           In order to avoid the first false triggering of the touch.
*
* @return 	status - Status of operation
*/
////////////////////////////////////////////////////////////////////////////////
xpt2046_status_t xpt2046_init(void)
{
	xpt2046_status_t status = eXPT2046_OK;

	XPT2046_ASSERT( false == gb_is_init );

	// Initialize low level drivers
	status = xpt2046_if_init();

	// Initialize FSM
	g_cal_fsm.state.cur = eXPT2046_FSM_NORMAL;
	g_cal_fsm.state.next = eXPT2046_FSM_NORMAL;
	g_cal_fsm.time.duration = 0;
	g_cal_fsm.time.first_entry = false;

	// Init done
	gb_is_init = true;

	XPT2046_ASSERT( status == eXPT2046_OK );

	return status;
}

////////////////////////////////////////////////////////////////////////////////
/**
*		Get module init flag
*
* @return 	gb_is_init - Initialization status
*/
////////////////////////////////////////////////////////////////////////////////
bool xpt2046_is_init(void)
{
	return gb_is_init;
}

////////////////////////////////////////////////////////////////////////////////
/**
*			Get touch data
*
* @param[out]	p_page 		- Pointer to page (x) coordinate
* @param[out]	p_col 		- Pointer to column (y) coordinate
* @param[out]	p_force 	- Pointer to pressure (force) of touch
* @param[out]	p_pressed 	- Pointer to pressed flag
* @return		status 		- Status of initialization
*/
////////////////////////////////////////////////////////////////////////////////
xpt2046_status_t xpt2046_get_touch(uint16_t * const p_page, uint16_t * const p_col, uint16_t * const p_force, bool * const p_pressed)
{
	xpt2046_status_t status = eXPT2046_OK;

	XPT2046_ASSERT( true == gb_is_init );

	if ( NULL != p_page )
	{
		*p_page 	= g_touch.page;
	}

	if ( NULL != p_col )
	{
		*p_col 		= g_touch.col;
	}

	if ( NULL != p_force )
	{
		*p_force 	= g_touch.force;
	}

	if ( NULL != p_pressed )
	{
		*p_pressed 	= g_touch.pressed;
	}

	return status;
}

////////////////////////////////////////////////////////////////////////////////
/**
*		Main touch controller handler
*
* @note 	Shall be called periodically every 10ms.
*
* @return 		void
*/
////////////////////////////////////////////////////////////////////////////////
void xpt2046_hndl(void)
{
	uint16_t X;
	uint16_t Y;
	uint16_t force;
	bool is_pressed;

	XPT2046_ASSERT( true == gb_is_init );

	// Get data
	xpt2046_read_data_from_controler( &X, &Y, &force, &is_pressed );

	// Apply filters
	#if XPT2046_FILTER_PTHRES_EN
		xpt2046_filter_pthres( &X, &Y, &force, &is_pressed );
	#endif

	#if XPT2046_FILTER_SKIP_EN
		xpt2046_filter_skip( &X, &Y, &force, &is_pressed );
	#endif

	#if XPT2046_FILTER_DEBOUNCE_EN
		xpt2046_filter_debounce( &X, &Y, &force, &is_pressed );
	#endif

	#if XPT2046_FILTER_MEDIAN_EN
	#if XPT2046_FILTER_USE_FAST_MEDIAN
		xpt2046_filter_median_fast( &X, &Y, &force, &is_pressed );
	#else
		xpt2046_filter_median( &X, &Y, &force, &is_pressed );
	#endif
	#endif

	#if XPT2046_FILTER_IIR_EN
		xpt2046_filter_iir( &X, &Y, &force, &is_pressed );
	#endif

	#if XPT2046_FILTER_AVG_EN
		xpt2046_filter_average( &X, &Y, &force, &is_pressed );
	#endif

	#if XPT2046_FILTER_JITTER_EN
		xpt2046_filter_jitter( &X, &Y, &force, &is_pressed );
	#endif

	// Apply calibration
	if ( g_cal_data.done )
	{
		xpt2046_calibrate_data( &X, &Y, (const int32_t*)&g_cal_data.factors );
	}

	// Store
	g_touch.page = X;
	g_touch.col = Y;
	g_touch.force = force;
	g_touch.pressed = is_pressed;

	// Calibration handler
	xpt2046_cal_hndl();
}

////////////////////////////////////////////////////////////////////////////////
/**
*		Read data from controller
*
* @note p_is_pressed - input state of IRQ touch line<br>
*       p_force - pressing force in range:
*       - 0        - no pressure
*       - near 60  - pressure with finger
*       - near 100 - pressing with stylus
*       - 200..300 - very strong pressure
*
* @param[out]	p_X				- Pointer to x coordinate
* @param[out]	p_Y				- Pointer to y coordinate
* @param[out]	p_force			- Pointer to pressure (force) of touch
* @param[out]	p_is_pressed	- Pointer to pressed state
* @return 		status			- Status of operation
*/
////////////////////////////////////////////////////////////////////////////////
static void xpt2046_read_data_from_controler(uint16_t * const p_X, uint16_t * const p_Y, uint16_t * const p_force, bool * const p_is_pressed)
{
	xpt2046_status_t status = eXPT2046_OK;
	uint16_t Z1;
	uint16_t Z2;
	static uint16_t X_prev;
	static uint16_t Y_prev;

	// Is pressed
	if ( eXPT2046_INT_ON == xpt2046_low_if_get_int() )
	{
		*p_is_pressed = true;

		// Get pressure data
		status |= xpt2046_low_if_exchange( eXPT2046_ADDR_Z1_POS, eXPT2046_PD_DEVICE_FULLY_ON, eXPT2046_START_ON, &Z1 );
		status |= xpt2046_low_if_exchange( eXPT2046_ADDR_YN, eXPT2046_PD_DEVICE_FULLY_ON, eXPT2046_START_ON, &Z2 );

		// Dummy reading X to reduce noise
		status |= xpt2046_low_if_exchange( eXPT2046_ADDR_X_POS, eXPT2046_PD_DEVICE_FULLY_ON, eXPT2046_START_ON, NULL );

		// Get X position
		status |= xpt2046_low_if_exchange( eXPT2046_ADDR_X_POS, eXPT2046_PD_DEVICE_FULLY_ON, eXPT2046_START_ON, p_X );

		// Dummy reading Y to reduce noise
		status |= xpt2046_low_if_exchange( eXPT2046_ADDR_Y_POS, eXPT2046_PD_DEVICE_FULLY_ON, eXPT2046_START_ON, NULL );

		// Get X position
		status |= xpt2046_low_if_exchange( eXPT2046_ADDR_Y_POS, eXPT2046_PD_VREF_ON, eXPT2046_START_ON, p_Y );

		if ( eXPT2046_OK == status )
		{
			if ( Z2 != Z1 )
            {
				// The following algorithm for calculating the pressing force
				// is slightly dependent on the location on the screen.
				// Linearly depends on the pressing force and is in the range:
				// 0         - no pressure
				// About 50  - pressure with finger
				// About 100 - pressing with stylus
				// 200..300  - very strong pressure
				// 
				// 8000 : coefficient
				// 100  : 100%
                *p_force = 8000U * 100U * Z1 / (*p_X * (Z2 - Z1));
			}
			X_prev = *p_X;
			Y_prev = *p_Y;
		}
	}
	else
	{
		*p_is_pressed = false;

		// Return old value
		*p_X = X_prev;
		*p_Y = Y_prev;
		*p_force = 0;
	}
}

////////////////////////////////////////////////////////////////////////////////
/**
*		Start calibration routine
*
* @return 	status - Status of operation
*/
////////////////////////////////////////////////////////////////////////////////
xpt2046_status_t xpt2046_start_calibration(void)
{
	xpt2046_status_t status = eXPT2046_OK;

	if ( true == gb_is_init )
	{
		if ( false == g_cal_data.busy )
		{
			g_cal_data.start = true;
			g_cal_data.done = false;
		}
		else
		{
			status = eXPT2046_CAL_IN_PROGRESS;
		}
	}
	else
	{
		status = eXPT2046_ERROR;

		XPT2046_DBG_PRINT( "Module not initialized!" );
		XPT2046_ASSERT( 0 );
	}

	return status;
}

////////////////////////////////////////////////////////////////////////////////
/**
*		Calibration FSM handler
*
* @note 	This handler must be called within ili9488 task, as it
* 			is used display functionalities!
*
* @return 		void
*/
////////////////////////////////////////////////////////////////////////////////
static void xpt2046_cal_hndl(void)
{
	xpt2046_fms_manager();

	switch( g_cal_fsm.state.cur )
	{
		case eXPT2046_FSM_NORMAL:
			xpt2046_fsm_normal();
			break;

		case eXPT2046_FSM_P1_ACQ:
			xpt2046_fsm_p1_acq();
			break;

		case eXPT2046_FSM_P2_ACQ:
			xpt2046_fsm_p2_acq();
			break;

		case eXPT2046_FSM_P3_ACQ:
			xpt2046_fsm_p3_acq();
			break;

		case eXPT2046_FSM_CALC_FACTORS:
			xpt2046_fsm_calc_factors();
			break;

		default:
			xpt2046_fsm_normal();

			XPT2046_DBG_PRINT( "Invalid FSM state..." );
			XPT2046_ASSERT( 0 );
			break;
	}
}

////////////////////////////////////////////////////////////////////////////////
/**
*		Calibration FSM manager
*
* @return 		void
*/
////////////////////////////////////////////////////////////////////////////////
static void xpt2046_fms_manager(void)
{
	static uint32_t tick = 0;

	//if ( state_prev != g_cal_fsm.state.cur )
	if ( g_cal_fsm.state.cur != g_cal_fsm.state.next )
	{
		g_cal_fsm.state.cur = g_cal_fsm.state.next;
		g_cal_fsm.time.duration = 0;
		g_cal_fsm.time.first_entry = true;
	}
	else
	{
		g_cal_fsm.time.duration += (uint32_t) ( HAL_GetTick() - tick );
		g_cal_fsm.time.duration = XPT2046_LIMIT_FMS_DURATION( g_cal_fsm.time.duration );
		g_cal_fsm.time.first_entry = false;
	}

	tick = HAL_GetTick();
}

////////////////////////////////////////////////////////////////////////////////
/**
*		Normal (idle) FSM state
*
* @return 		void
*/
////////////////////////////////////////////////////////////////////////////////
static void xpt2046_fsm_normal(void)
{
	if ( true == g_cal_data.start )
	{
		g_cal_data.start = false;
		g_cal_data.busy = true;

		g_cal_fsm.state.next = eXPT2046_FSM_P1_ACQ;
	}
}

////////////////////////////////////////////////////////////////////////////////
/**
*		Acquire P1 point FSM state
*
* @return 		void
*/
////////////////////////////////////////////////////////////////////////////////
static void xpt2046_fsm_p1_acq(void)
{
	static bool point_touched = false;

	if ( true == g_cal_fsm.time.first_entry )
	{
		// Calibration info
		ili9488_set_string_pen( eILI9488_COLOR_BLACK, eILI9488_COLOR_YELLOW, eILI9488_FONT_20 );
		ili9488_set_string( "Calibration in progress...", 10, 100 );

		// Set up P1
		xpt2046_set_cal_point( eXPT2046_CAL_P1 );

		point_touched = false;
	}
	else
	{
		// Wait for first touch
		if ( false == point_touched )
		{
			if ( true == g_touch.pressed )
			{
				point_touched = true;
			}
		}

		// Point touch
		else
		{
			// Acquire data
			g_cal_data.Tp[0].x = g_touch.page;
			g_cal_data.Tp[0].y = g_touch.col;

			// Wait for release
			if ( false == g_touch.pressed )
			{
				g_cal_fsm.state.next = eXPT2046_FSM_P2_ACQ;

				// Clear P1
				xpt2046_clear_cal_point( eXPT2046_CAL_P1 );
			}
		}
	}
}

////////////////////////////////////////////////////////////////////////////////
/**
*		Acquire P2 point FSM state
*
* @return 		void
*/
////////////////////////////////////////////////////////////////////////////////
static void xpt2046_fsm_p2_acq(void)
{
	static bool point_touched = false;

	if ( true == g_cal_fsm.time.first_entry )
	{
		// Set up P2
		xpt2046_set_cal_point( eXPT2046_CAL_P2 );

		point_touched = false;
	}
	else
	{
		// Wait for first touch
		if ( false == point_touched )
		{
			if ( true == g_touch.pressed )
			{
				point_touched = true;
			}
		}

		// Point touch
		else
		{
			// Acquire data
			g_cal_data.Tp[1].x = g_touch.page;
			g_cal_data.Tp[1].y = g_touch.col;

			// Wait for release
			if ( false == g_touch.pressed )
			{
				g_cal_fsm.state.next = eXPT2046_FSM_P3_ACQ;

				// Clear point
				xpt2046_clear_cal_point( eXPT2046_CAL_P2 );
			}
		}
	}
}

////////////////////////////////////////////////////////////////////////////////
/**
*		Acquire P3 point FSM state
*
* @return 		void
*/
////////////////////////////////////////////////////////////////////////////////
static void xpt2046_fsm_p3_acq(void)
{
	static bool point_touched = false;

	if ( true == g_cal_fsm.time.first_entry )
	{
		// Set up P3
		xpt2046_set_cal_point( eXPT2046_CAL_P3 );

		point_touched = false;
	}
	else
	{
		// Wait for first touch
		if ( false == point_touched )
		{
			if ( true == g_touch.pressed )
			{
				point_touched = true;
			}
		}

		// Point touch
		else
		{
			// Acquire data
			g_cal_data.Tp[2].x = g_touch.page;
			g_cal_data.Tp[2].y = g_touch.col;

			// Wait for release
			if ( false == g_touch.pressed )
			{
				g_cal_fsm.state.next = eXPT2046_FSM_CALC_FACTORS;

				// Clear point
				xpt2046_clear_cal_point( eXPT2046_CAL_P3 );

				// User info
				ili9488_set_string( "Calibration finished ...   ", 10, 100 );
			}
		}
	}
}

////////////////////////////////////////////////////////////////////////////////
/**
*		Calculate calibraton factor FSM state
*
* @return 		void
*/
////////////////////////////////////////////////////////////////////////////////
static void xpt2046_fsm_calc_factors(void)
{
	int32_t cal_factors[7];

	// Calculate calibration data
	xpt2046_calculate_factors( (int32_t*) &cal_factors, (const xpt2046_point_t*) &g_cal_data.Dp, (const xpt2046_point_t*) &g_cal_data.Tp );

	// Store
	memcpy( g_cal_data.factors, cal_factors, sizeof( cal_factors ));

	// Go to normal
	g_cal_fsm.state.next = eXPT2046_FSM_NORMAL;

	// Manage flags
	g_cal_data.busy = false;
	g_cal_data.done = true;
}

////////////////////////////////////////////////////////////////////////////////
/**
*		Set (draw) calibration point
*
* @param[in]	px	- Calibration point number
* @return 		void
*/
////////////////////////////////////////////////////////////////////////////////
static void xpt2046_set_cal_point(const xpt2046_points_t px)
{
	if ( px < eXPT2046_CAL_P_NUM_OF )
	{
		g_cal_circ_attr.position.start_page = g_cal_data.Dp[ px ].x;
		g_cal_circ_attr.position.start_col 	= g_cal_data.Dp[ px ].y;
		g_cal_circ_attr.fill.color			= XPT2046_POINT_COLOR_FG;
		ili9488_draw_circle( &g_cal_circ_attr );
	}
}

////////////////////////////////////////////////////////////////////////////////
/**
*		Clear calibration point
*
* @param[in]	px	- Calibration point number
* @return 		void
*/
////////////////////////////////////////////////////////////////////////////////
static void xpt2046_clear_cal_point(const xpt2046_points_t px)
{
	if ( px < eXPT2046_CAL_P_NUM_OF )
	{
		//ili9488_fill_rectangle( g_cal_data.Dp[ px ].x, g_cal_data.Dp[ px ].y, XPT2046_POINT_SIZE, XPT2046_POINT_SIZE, XPT2046_POINT_COLOR_BG );

		g_cal_circ_attr.position.start_page = g_cal_data.Dp[ px ].x;
		g_cal_circ_attr.position.start_col 	= g_cal_data.Dp[ px ].y;
		g_cal_circ_attr.fill.color			= XPT2046_POINT_COLOR_BG;
		ili9488_draw_circle( &g_cal_circ_attr );
	}
}

////////////////////////////////////////////////////////////////////////////////
/**
*		Calculate calibration data
*
* @param[out]	p_factors 	- Pointer to cal data
* @param[in]	p_Dp	 	- Pointer to display points
* @param[in]	p_Tp	 	- Pointer to touch points
* @return 		void
*/
////////////////////////////////////////////////////////////////////////////////
static void xpt2046_calculate_factors(int32_t * p_factors, const xpt2046_point_t * const p_Dp, const xpt2046_point_t * const p_Tp)
{
	// Calculate C vectors
	p_factors[0] = 	(	(int32_t) (( p_Tp[0].x - p_Tp[2].x ) * ( p_Tp[1].y - p_Tp[2].y ))
					- 	(int32_t) (( p_Tp[1].x - p_Tp[2].x ) * ( p_Tp[0].y - p_Tp[2].y )));

	p_factors[1] = 	(	(int32_t) (( p_Dp[0].x - p_Dp[2].x ) * ( p_Tp[1].y - p_Tp[2].y ))
					- 	(int32_t) (( p_Dp[1].x - p_Dp[2].x ) * ( p_Tp[0].y - p_Tp[2].y )));

	p_factors[2] = 	(	(int32_t) (( p_Tp[0].x - p_Tp[2].x ) * ( p_Dp[1].x - p_Dp[2].x ))
					- 	(int32_t) (( p_Dp[0].x - p_Dp[2].x ) * ( p_Tp[1].x - p_Tp[2].x )));

	p_factors[3] = 	(	(int32_t) ( p_Tp[0].y * (((int32_t) p_Tp[2].x * p_Dp[1].x ) - ((int32_t) p_Tp[1].x * p_Dp[2].x )))
					+	(int32_t) ( p_Tp[1].y * (((int32_t) p_Tp[0].x * p_Dp[2].x ) - ((int32_t) p_Tp[2].x * p_Dp[0].x )))
					+	(int32_t) ( p_Tp[2].y * (((int32_t) p_Tp[1].x * p_Dp[0].x ) - ((int32_t) p_Tp[0].x * p_Dp[1].x ))));

	p_factors[4] = 	(	(int32_t) (( p_Dp[0].y - p_Dp[2].y ) * ( p_Tp[1].y - p_Tp[2].y ))
					- 	(int32_t) (( p_Dp[1].y - p_Dp[2].y ) * ( p_Tp[0].y - p_Tp[2].y )));

	p_factors[5] = 	(	(int32_t) (( p_Tp[0].x - p_Tp[2].x ) * ( p_Dp[1].y - p_Dp[2].y ))
					- 	(int32_t) (( p_Dp[0].y - p_Dp[2].y ) * ( p_Tp[1].x - p_Tp[2].x )));


	p_factors[6] = 	(	(int32_t) ( p_Tp[0].y * (((int32_t) p_Tp[2].x * p_Dp[1].y ) - ((int32_t) p_Tp[1].x * p_Dp[2].y )))
					+	(int32_t) ( p_Tp[1].y * (((int32_t) p_Tp[0].x * p_Dp[2].y ) - ((int32_t) p_Tp[2].x * p_Dp[0].y )))
					+	(int32_t) ( p_Tp[2].y * (((int32_t) p_Tp[1].x * p_Dp[0].y ) - ((int32_t) p_Tp[0].x * p_Dp[1].y ))));
}

////////////////////////////////////////////////////////////////////////////////
/**
*		Calibrate raw touch data
*
* @param[in] 	p_X			- Pointer to x coordinate
* @param[in] 	p_Y			- Pointer to y coordinate
* @param[in] 	p_factors	- Pointer to cal factors
* @return 		void
*/
////////////////////////////////////////////////////////////////////////////////
static void xpt2046_calibrate_data(uint16_t * const p_X, uint16_t * const p_Y, const int32_t * const p_factors)
{
	xpt2046_point_t Dp;
	xpt2046_point_t Tp;

	// Touch coordinate
	Tp.x = (int64_t) *p_X;
	Tp.y = (int64_t) *p_Y;

	// Apply factors
	Dp.x = ((( p_factors[1] * Tp.x ) + ( p_factors[2] * Tp.y ) + p_factors[3] ) / p_factors[0] );
	Dp.y = ((( p_factors[4] * Tp.x ) + ( p_factors[5] * Tp.y ) + p_factors[6] ) / p_factors[0] );

	// Limit
	Dp.x = xpt2046_limit_cal_X_data( Dp.x );
	Dp.y = xpt2046_limit_cal_Y_data( Dp.y );

	// Return calibrated values
	*p_X = (uint16_t) Dp.x;
	*p_Y = (uint16_t) Dp.y;
}

////////////////////////////////////////////////////////////////////////////////
/**
*		Limit X calibration coordinate
*
* @param[in] 	unlimited_data 	- Calibrated unlimited data
* @return 		lim_data		- Limited data due to limitation of display
*/
////////////////////////////////////////////////////////////////////////////////
static int32_t xpt2046_limit_cal_X_data(const int32_t unlimited_data)
{
	int32_t lim_data;

	if ( unlimited_data < 0 )
	{
		lim_data = 0;
	}
	else if ( unlimited_data > XPT2046_DISPLAY_MAX_X )
	{
		lim_data = XPT2046_DISPLAY_MAX_X;
	}
	else
	{
		lim_data = unlimited_data;
	}

	return lim_data;
}

////////////////////////////////////////////////////////////////////////////////
/**
*		Limit Y calibration coordinate
*
* @param[in] 	unlimited_data 	- Calibrated unlimited data
* @return 		lim_data		- Limited data due to limitation of display
*/
////////////////////////////////////////////////////////////////////////////////
static int32_t xpt2046_limit_cal_Y_data(const int32_t unlimited_data)
{
	int32_t lim_data;

	if ( unlimited_data < 0 )
	{
		lim_data = 0;
	}
	else if ( unlimited_data > XPT2046_DISPLAY_MAX_Y )
	{
		lim_data = XPT2046_DISPLAY_MAX_Y;
	}
	else
	{
		lim_data = unlimited_data;
	}

	return lim_data;
}

////////////////////////////////////////////////////////////////////////////////
/**
*		Get calibration done flag
*
* @return 	calibration done
*/
////////////////////////////////////////////////////////////////////////////////
bool xpt2046_is_calibrated(void)
{
	return g_cal_data.done;
}

////////////////////////////////////////////////////////////////////////////////
/**
*		Set calibration factors
*
* @param[in] 	p_factors	- Pointer to factors
* @return 		status 		- Status of operation
*/
////////////////////////////////////////////////////////////////////////////////
xpt2046_status_t xpt2046_set_cal_factors(const int32_t * const p_factors)
{
	xpt2046_status_t status = eXPT2046_OK;

	if ( NULL != p_factors )
	{
		// Calibration already done some time in past
		g_cal_data.done = true;

		// Copy factors
		memcpy( &g_cal_data.factors, p_factors, sizeof( g_cal_data.factors ));
	}
	else
	{
		status = eXPT2046_ERROR;
	}

	return status;
}

////////////////////////////////////////////////////////////////////////////////
/**
*		Get calibration factors
*
* @param[out] 	p_factors	- Pointer to factors
* @return 		status 		- Status of operation
*/
////////////////////////////////////////////////////////////////////////////////
xpt2046_status_t xpt2046_get_cal_factors(const int32_t * p_factors)
{
	xpt2046_status_t status = eXPT2046_OK;

	if ( NULL != p_factors )
	{
		// Copy factors
		memcpy( (void*) p_factors, (void*) &g_cal_data.factors, sizeof( g_cal_data.factors ));
	}
	else
	{
		status = eXPT2046_ERROR;
	}

	return status;
}


#if ( XPT2046_FILTER_PTHRES_EN )
////////////////////////////////////////////////////////////////////////////////
/**
*		Pressure threshold filter.
*
* @note	Given that a release is always pressure 0 and a press is always >= 100,
*		this discards samples below / above the specified pressure threshold.
*
* @param[out]	p_X				- Pointer to x coordinate
* @param[out]	p_Y				- Pointer to y coordinate
* @param[out]	p_force			- Pointer to pressure (force) of touch
* @param[out]	p_is_pressed	- Pointer to pressed state
* @return 		void
*/
////////////////////////////////////////////////////////////////////////////////
static void xpt2046_filter_pthres(uint16_t *const p_X, uint16_t *const p_Y,
								  uint16_t *const p_force, bool *const p_is_pressed)
{
	static bool mem_pressed = false;

	if ( *p_force > XPT2046_FILTER_PTHRES_ON )
	{
		mem_pressed = true;
	}
	else if ( *p_force < XPT2046_FILTER_PTHRES_OFF )
	{
		mem_pressed = false;
	}

	*p_is_pressed = mem_pressed;
}
#endif


#if ( XPT2046_FILTER_SKIP_EN )
////////////////////////////////////////////////////////////////////////////////
/**
*		Skip samples after press.
*
* @note	This should help if the first samples are unreliable for the device.
*
* @param[out]	p_X				- Pointer to x coordinate
* @param[out]	p_Y				- Pointer to y coordinate
* @param[out]	p_force			- Pointer to pressure (force) of touch
* @param[out]	p_is_pressed	- Pointer to pressed state
* @return 		void
*/
////////////////////////////////////////////////////////////////////////////////
static void xpt2046_filter_skip(uint16_t *const p_X, uint16_t *const p_Y,
								uint16_t *const p_force, bool *const p_is_pressed)
{
	static int counter = 0;
	bool mem_pressed;

	mem_pressed = false;

	if ( *p_is_pressed == true )
	{
		if ( counter >= XPT2046_FILTER_SKIP_SAMP )
		{
			mem_pressed = true;
		}
		else
		{
			counter++;
		}
	}
	else
	{
		counter = 0;
	}

	*p_is_pressed = mem_pressed;
}
#endif


#if ( XPT2046_FILTER_DEBOUNCE_EN )
////////////////////////////////////////////////////////////////////////////////
/**
*		Simple debounce mechanism that drops input events for
*		the specified time after a touch gesture stopped.
*
* @param[out]	p_X				- Pointer to x coordinate
* @param[out]	p_Y				- Pointer to y coordinate
* @param[out]	p_force			- Pointer to pressure (force) of touch
* @param[out]	p_is_pressed	- Pointer to pressed state
* @return 		void
*/
////////////////////////////////////////////////////////////////////////////////
static void xpt2046_filter_debounce(uint16_t *const p_X, uint16_t *const p_Y,
									uint16_t *const p_force, bool *const p_is_pressed)
{
	static uint32_t release_milis;
	static bool mem_pressed;

	uint32_t milis = HAL_GetTick();

	if ( *p_is_pressed == false )
	{
		if ( mem_pressed == true )
		{
			release_milis = milis;
		}
	}
	else
	{
		if ( milis < (uint32_t)( release_milis + XPT2046_FILTER_DEBOUNCE_TIME ) )
		{
			*p_is_pressed = false;
		}
		//release_milis = milis;
	}

	mem_pressed = *p_is_pressed;
}
#endif


#if ( XPT2046_FILTER_MEDIAN_EN )
#if ( XPT2046_FILTER_USE_FAST_MEDIAN )

static void fast_median_cmp_swap(uint16_t *p_a, uint16_t *p_b)
{
	uint16_t tmp;

	if ( *p_a > *p_b )
	{
		tmp = *p_a;
		*p_a = *p_b;
		*p_b = tmp;
	}
}


static uint16_t fast_median(uint16_t *p_array)
{
	uint16_t sort_array[5];

	memcpy( sort_array, p_array, sizeof(sort_array) );

	// Sorting the work array in increasing order
	fast_median_cmp_swap( &sort_array[0], &sort_array[1] );
	fast_median_cmp_swap( &sort_array[2], &sort_array[3] );
	fast_median_cmp_swap( &sort_array[0], &sort_array[2] );
	fast_median_cmp_swap( &sort_array[1], &sort_array[4] );
	fast_median_cmp_swap( &sort_array[0], &sort_array[1] );
	fast_median_cmp_swap( &sort_array[2], &sort_array[3] );
	fast_median_cmp_swap( &sort_array[1], &sort_array[2] );
	fast_median_cmp_swap( &sort_array[3], &sort_array[4] );
	fast_median_cmp_swap( &sort_array[2], &sort_array[3] );

	return sort_array[2];
}

////////////////////////////////////////////////////////////////////////////////
/**
*		The median filter reduces noise in the samples' coordinate values.
*		It is able to filter undesired single large jumps in the signal.
*
* @param[out]	p_X				- Pointer to x coordinate
* @param[out]	p_Y				- Pointer to y coordinate
* @param[out]	p_force			- Pointer to pressure (force) of touch
* @param[out]	p_is_pressed	- Pointer to pressed state
* @return   	void
*/
////////////////////////////////////////////////////////////////////////////////
static void xpt2046_filter_median_fast(uint16_t *const p_X, uint16_t *const p_Y,
									   uint16_t *const p_force, bool *const p_is_pressed)
{
	static uint16_t x_mediane[5];
	static uint16_t y_mediane[5];
	static uint8_t samp_cnt = 0;
	static uint8_t samp_head = 0;

	// collect data
	if ( *p_is_pressed == true )
	{
		x_mediane[samp_head] = *p_X;
		y_mediane[samp_head] = *p_Y;

		if ( samp_cnt < 5 )
		{
			samp_cnt++;
		}

		// Ring buffer positiion
		samp_head = (samp_head + 1) % 5;
	}
	else
	{
		samp_cnt = 0;
	}

	if ( samp_cnt == 5 )
	{
		*p_X = fast_median( x_mediane );
		*p_Y = fast_median( y_mediane );
	}
	else
	{
		*p_is_pressed = false;
	}
}

#else // ( XPT2046_FILTER_USE_FAST_MEDIAN )

////////////////////////////////////////////////////////////////////////////////
/**
*		The median filter reduces noise in the samples' coordinate values.
*		It is able to filter undesired single large jumps in the signal.
*
* @param[out]	p_X				- Pointer to x coordinate
* @param[out]	p_Y				- Pointer to y coordinate
* @param[out]	p_force			- Pointer to pressure (force) of touch
* @param[out]	p_is_pressed	- Pointer to pressed state
* @return   	void
*/
////////////////////////////////////////////////////////////////////////////////
static void xpt2046_filter_median(uint16_t *const p_X, uint16_t *const p_Y,
								  uint16_t *const p_force, bool *const p_is_pressed)
{
	static uint16_t x_mediane[XPT2046_FILTER_MEDIAN_WIN];
	static uint16_t y_mediane[XPT2046_FILTER_MEDIAN_WIN];
	static uint16_t sort_array[XPT2046_FILTER_MEDIAN_WIN];
	static uint8_t samp_cnt = 0;
	static uint8_t samp_head = 0;

	// collect data
	if ( *p_is_pressed == true )
	{
		x_mediane[samp_head] = *p_X;
		y_mediane[samp_head] = *p_Y;

		if ( samp_cnt < XPT2046_FILTER_MEDIAN_WIN )
		{
			samp_cnt++;
		}

		// Ring buffer positiion
		samp_head = (samp_head + 1) % XPT2046_FILTER_MEDIAN_WIN;

	}
	else
	{
		samp_cnt = 0;
	}

	if ( samp_cnt == XPT2046_FILTER_MEDIAN_WIN )
	{
		memcpy( sort_array, x_mediane, sizeof(sort_array) );
		// Sorting the work array in descending order
		for ( int ab = 0; ab < (XPT2046_FILTER_MEDIAN_WIN - 1); ab++ )
		{
			for ( int aa = (ab + 1); aa < XPT2046_FILTER_MEDIAN_WIN; aa++ )
			{
				if ( sort_array[ab] < sort_array[aa] )
				{ // SWAP
					int swap_temp = sort_array[ab];
					sort_array[ab] = sort_array[aa];
					sort_array[aa] = swap_temp;
				}
			}
		}
		// Select the middle element
		*p_X = sort_array[XPT2046_FILTER_MEDIAN_WIN / 2];

		memcpy( sort_array, y_mediane, sizeof(sort_array) );
		// Sorting the work array in descending order
		for ( int ab = 0; ab < (XPT2046_FILTER_MEDIAN_WIN - 1); ab++ )
		{
			for ( int aa = (ab + 1); aa < XPT2046_FILTER_MEDIAN_WIN; aa++ )
			{
				if ( sort_array[ab] < sort_array[aa] )
				{ // SWAP
					int swap_temp = sort_array[ab];
					sort_array[ab] = sort_array[aa];
					sort_array[aa] = swap_temp;
				}
			}
		}
		// Select the middle element
		*p_Y = sort_array[XPT2046_FILTER_MEDIAN_WIN / 2];
	}
	else
	{
		*p_is_pressed = false;
	}
}


#endif // ( XPT2046_FILTER_USE_FAST_MEDIAN )
#endif // ( XPT2046_FILTER_MEDIAN_EN )

#if ( XPT2046_FILTER_IIR_EN )
////////////////////////////////////////////////////////////////////////////////
/**
*		Infinite impulse response filter.
*		This is a smoothing filter to remove low-level noise.
*
* @note	There is a trade-off between noise removal (smoothing) and responsiveness.<br>
*		The parameters N and D specify the level of smoothing in the form of a fraction (N/D).
*
* @param[out]	p_X				- Pointer to x coordinate
* @param[out]	p_Y				- Pointer to y coordinate
* @param[out]	p_force			- Pointer to pressure (force) of touch
* @param[out]	p_is_pressed	- Pointer to pressed state
* @return   	void
*/
////////////////////////////////////////////////////////////////////////////////
static void xpt2046_filter_iir(uint16_t *const p_X, uint16_t *const p_Y,
							   uint16_t *const p_force, bool *const p_is_pressed)
{
	static bool mem_pressed = false;
	static int32_t x_s = 0;
	static int32_t y_s = 0;
	const int32_t N = XPT2046_FILTER_IIR_N;
	const int32_t D = XPT2046_FILTER_IIR_D;

	if ( *p_is_pressed == true )
	{
		if ( mem_pressed == true)
		{
			x_s = (N * x_s + (D - N) * *p_X + D / 2) / D;
			y_s = (N * y_s + (D - N) * *p_Y + D / 2) / D;
			*p_X = x_s;
			*p_Y = y_s;
		}
		else
		{
			x_s = *p_X;
			y_s = *p_Y;
		}
	}

	mem_pressed = *p_is_pressed;
}
#endif

#if ( XPT2046_FILTER_AVG_EN )
////////////////////////////////////////////////////////////////////////////////
/**
*		Averaging filter.
*		Save and averages the last [XPT2046_FILTER_AVG_SAMP] values of X, Y, force.
*
* @param[out]	p_X		- Pointer to x coordinate
* @param[out]	p_Y		- Pointer to y coordinate
* @param[out]	p_force	- Pointer to pressure (force) of touch
* @param[out]	p_touch	- Pointer to touch detected state
* @return 		status	- Status of operation
*/
////////////////////////////////////////////////////////////////////////////////
static void xpt2046_filter_average(uint16_t * const p_X, uint16_t * const p_Y, uint16_t * const p_force, bool * const p_touch)
{
	static xpt2046_filter_t filter;
	static uint8_t samp_cnt = 0;
	static uint8_t samp_head = 0;
	uint32_t i;

	if 	( true == *p_touch )
	{
		if ( samp_cnt >= XPT2046_FILTER_AVG_SAMP )
		{
			// Subtracting the outgoing value
			filter.x.sum -= filter.x.samp_buf[ samp_head ];
			filter.y.sum -= filter.y.samp_buf[ samp_head ];
			filter.force.sum -= filter.force.samp_buf[ samp_head ];
		}

		// Addition the input value
		filter.x.sum += *p_X;
		filter.y.sum += *p_Y;
		filter.force.sum += *p_force;

		// Fill buffer
		filter.x.samp_buf[ samp_head ] = *p_X;
		filter.y.samp_buf[ samp_head ] = *p_Y;
		filter.force.samp_buf[ samp_head ] = *p_force;

		// Increment sample counter
		if ( samp_cnt < XPT2046_FILTER_AVG_SAMP )
		{
			samp_cnt++;
		}

		// Ring buffer positiion
		samp_head = (samp_head + 1) % XPT2046_FILTER_AVG_SAMP;
	}
	else
	{
		// No touch detected -> reset filter
		samp_cnt = 0;
		filter.x.sum = 0;
		filter.y.sum = 0;
		filter.force.sum = 0;
	}

	if ( samp_cnt >= XPT2046_FILTER_AVG_SAMP )
	{
		// Average
		*p_X = (uint16_t) (filter.x.sum / XPT2046_FILTER_AVG_SAMP );
		*p_Y = (uint16_t) (filter.y.sum / XPT2046_FILTER_AVG_SAMP );
		*p_force = (uint16_t) (filter.force.sum / XPT2046_FILTER_AVG_SAMP );
	}
	else
	{
		*p_touch = false;
	}

}
#endif

#if ( XPT2046_FILTER_JITTER_EN )
////////////////////////////////////////////////////////////////////////////////
/**
*		Jitter filter for raw XY data.
*
* @note	Removes jitter on the X and Y co-ordinates.
*
* @param[out]	p_X				- Pointer to x coordinate
* @param[out]	p_Y				- Pointer to y coordinate
* @param[out]	p_force			- Pointer to pressure (force) of touch
* @param[out]	p_is_pressed	- Pointer to pressed state
* @return   	void
*/
////////////////////////////////////////////////////////////////////////////////
static void xpt2046_filter_jitter(uint16_t * const p_X, uint16_t * const p_Y,
								  uint16_t * const p_force, bool * const p_is_pressed)
{
	static uint16_t x_prev;
	static uint16_t y_prev;
	static bool mem_pressed = false;

	if ( *p_is_pressed == true && mem_pressed == true )
	{
		if ( *p_X > x_prev + XPT2046_FILTER_JITTER_THRES )
		{
			*p_X = *p_X - XPT2046_FILTER_JITTER_THRES;
		}
		else if ( *p_X < x_prev - XPT2046_FILTER_JITTER_THRES )
		{
			*p_X = *p_X + XPT2046_FILTER_JITTER_THRES;
		}
		else
		{
			*p_X = x_prev;
		}

		if ( *p_Y > y_prev + XPT2046_FILTER_JITTER_THRES )
		{
			*p_Y = *p_Y - XPT2046_FILTER_JITTER_THRES;
		}
		else if ( *p_Y < y_prev - XPT2046_FILTER_JITTER_THRES )
		{
			*p_Y = *p_Y + XPT2046_FILTER_JITTER_THRES;
		}
		else
		{
			*p_Y = y_prev;
		}
	}

	mem_pressed = *p_is_pressed;
	x_prev = *p_X;
	y_prev = *p_Y;
}
#endif

////////////////////////////////////////////////////////////////////////////////
/**
* @} <!-- END GROUP -->
*/
////////////////////////////////////////////////////////////////////////////////
