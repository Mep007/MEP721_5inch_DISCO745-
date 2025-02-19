#ifndef LVGL_PORT_SCREEN_H
#define LVGL_PORT_SCREEN_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/

#include "lvgl.h"

/****************************************************
 *  LCD TYPE DEFINITION (need to select ONE only !!!)
 ***************************************************/
#define DISCO_LCD_4_3_inch 		0       // 480x272 -- OEM 745i-DISCO - need modify LTDC settings via CubeMx too !!
#define DISCO_LCD_5_0_inch 		1       // 800x480 -- need modify LTDC settings via CubeMx too !! this example is working with LCD D500FPC7009-C

#define  LCD_BL_PWM_TIM 		htim8     // rest must be set via CubeMX

/*********************
 *      DEFINES
 *********************/

#if (DISCO_LCD_4_3_inch == 1)
  #define MY_DISP_HOR_RES   	 480
  #define MY_DISP_VER_RES   	 272
#elif (DISCO_LCD_5_0_inch == 1)
  #define MY_DISP_HOR_RES   	 800
  #define MY_DISP_VER_RES   	 480
#endif

/**********************
 * GLOBAL PROTOTYPES
 **********************/

void lvgl_display_init (void);

/**
  * @brief TouchScreen Slave I2C address 1
  */
/*
#define TS_I2C_ADDRESS              0x70U

#define TS_INT_PIN                   GPIO_PIN_2
#define TS_INT_GPIO_PORT             GPIOG
#define TS_INT_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOG_CLK_ENABLE()
#define TS_INT_GPIO_CLK_DISABLE()    __HAL_RCC_GPIOG_CLK_DISABLE()
#define TS_INT_EXTI_IRQn             EXTI2_IRQn
#define TS_EXTI_LINE                 EXTI_LINE_2
*/
//typedef struct
//{
//  uint32_t   Width;                  /* Screen Width */
//  uint32_t   Height;                 /* Screen Height */
//  uint32_t   Orientation;            /* Touch Screen orientation from the upper left position  */
//  uint32_t   Accuracy;               /* Expressed in pixel and means the x or y difference vs old
//                                        position to consider the new values valid */
//} TS_Init_t;



#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif /* __LVGL_PORT_DISPLAY_H */
