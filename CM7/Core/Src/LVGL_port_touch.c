
/*********************
 *      INCLUDES
 *********************/
#include "main.h"
#include "LVGL_port_touch.h"

/*********************
 *      DEFINES
 *********************/

#define TS_INSTANCE		(0)

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void touchpad_read(lv_indev_drv_t *drv, lv_indev_data_t *data);

/**********************
 *  STATIC VARIABLES
 **********************/
static  TS_State_t 	TS_State;
TS_State_t 		    TS_StateList;   // only debug for defaultTask read touch

/**********************************************************************************/
/**********************************************************************************/
void lvgl_touchscreen_init()    // Initialize your input devices here - default I2C4 for 745i-DISCO
{
	static lv_indev_drv_t 	indev_drv; /*Descriptor of an input device driver*/
	TS_Init_t 				hTS;

	hTS.Width   = LCD_TOUCH_WIDTH;
	hTS.Height  = LCD_TOUCH_HEIGHT;

	#if (DISCO_LCD_4_3_inch == 1)       // MEP
  	  hTS.Orientation = TS_SWAP_XY;     // orig DISCO745 - 4.3"
  	  hTS.Accuracy = 5 ;                // def. 5
    #elif (DISCO_LCD_5_0_inch == 1)
  	  hTS.Orientation = TS_SWAP_NONE;   // 5" LCD
  	  hTS.Accuracy = 2;                 //
    #endif

  	 /* Init TOUCH IC - FT5336/FT5446 */
	BSP_TS_Init(TS_INSTANCE, &hTS);

	/* basic LVGL driver initialization */
	lv_indev_drv_init(&indev_drv);                  /*Basic initialization*/
	indev_drv.type 		= LV_INDEV_TYPE_POINTER;   /*The touchpad is pointer type device*/
	indev_drv.read_cb 	= touchpad_read;

	/* register the driver in LVGL */
	lv_indev_drv_register(&indev_drv);
}

/**********************************************************************************/
static void touchpad_read(lv_indev_drv_t *indev, lv_indev_data_t *data)
{
	// Read your touchpad
	static int16_t last_x = 0;
	static int16_t last_y = 0;
	//BSP_LED_Toggle(LED1);

	BSP_TS_GetState(TS_INSTANCE, &TS_State);
	if (TS_State.TouchDetected)
	{
		TS_StateList = TS_State;        // debug only
		data->point.x = TS_State.TouchX;
		data->point.y = TS_State.TouchY;
		last_x = data->point.x;
		last_y = data->point.y;
		data->state = LV_INDEV_STATE_PRESSED;
	}
	else
	{
		data->point.x = last_x;
		data->point.y = last_y;
		data->state = LV_INDEV_STATE_RELEASED;
	}
}




/*
static void touchpad_read(lv_indev_drv_t *indev, lv_indev_data_t *data)
{
	// Read your touchpad
	static int16_t last_x = 0;
	static int16_t last_y = 0;
	//BSP_LED_Toggle(LED1);

	BSP_TS_GetState(TS_INSTANCE, &TS_State);
	if (TS_State.TouchDetected)
	{
		TS_StateList = TS_State;        // debug only
		data->point.x = TS_State.TouchX;
		data->point.y = TS_State.TouchY;
		last_x = data->point.x;
		last_y = data->point.y;
		data->state = LV_INDEV_STATE_PRESSED;
	}
	else
	{
		data->point.x = last_x;
		data->point.y = last_y;
		data->state = LV_INDEV_STATE_RELEASED;
	}
}
*/
