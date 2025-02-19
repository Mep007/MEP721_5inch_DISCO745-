/*
 * lvgl_port_touchpad.h
 *
 *  Created on: 23 Dec 2021
 *      Author: Ahmet Alperen Bulut / github.com/ahmetalperenbulut
 */

#ifndef LVGL_PORT_TOUCH_H
#define LVGL_PORT_TOUCH_H

/*********************
 *      INCLUDES
 *********************/
#include "LVGL_port_screen.h"
/*********************
 *      DEFINES
 *********************/
#if (DISCO_LCD_4_3_inch == 1)
  #define LCD_TOUCH_WIDTH       480U
  #define LCD_TOUCH_HEIGHT      272U
#elif (DISCO_LCD_5_0_inch == 1)
  #define LCD_TOUCH_WIDTH       800U
  #define LCD_TOUCH_HEIGHT      480U
#endif

/**********************
 *      TYPEDEFS
 **********************/
extern TS_State_t TS_StateList;
/**********************
 * GLOBAL PROTOTYPES
 **********************/
void lvgl_touchscreen_init();


#endif /* INC_LVGL_PORT_TOUCHPAD_H_ */
