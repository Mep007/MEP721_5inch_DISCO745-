/*********************
 *      INCLUDES
 *********************/
#include "main.h"
#include "LVGL_port_screen.h"  // <<<< ---- SELECT USED DISPLAY HERE !!!!!!!!!!
/**********************
 *  STATIC PROTOTYPES
 **********************/
/*Declare two drawing buffers 9.3 */
static lv_display_t * disp;

//static void disp_flush (lv_disp_drv_t*, const lv_area_t*, lv_color_t*);  // 8.4.11
static void disp_flush (lv_display_t *, const lv_area_t *, uint8_t *);  // 9.3
static void disp_flush_complete (DMA2D_HandleTypeDef*);

/**********************
 *  STATIC VARIABLES
 **********************/
/*  8.4.11
static lv_disp_drv_t		disp_drv;
static lv_disp_draw_buf_t 	disp_buf;
*/
/**********************
 *  DEFINES
 **********************/
//#define LVGL_BUFFER_1_ADDR_AT_SDRAM	SDRAM_DEVICE_ADDR + 2 * (MY_DISP_HOR_RES * MY_DISP_VER_RES)      // RGB565 ( ex. 800x480 x2bytes/per pixel = 768kB/per buffer)
//#define LVGL_BUFFER_2_ADDR_AT_SDRAM	SDRAM_DEVICE_ADDR + 4 * (MY_DISP_HOR_RES * MY_DISP_VER_RES)      //
static volatile uint32_t buf1[(MY_DISP_HOR_RES*MY_DISP_VER_RES)/2]__attribute__ (( section(".SDRAM_data"), used)) = {0};
static volatile uint32_t buf2[(MY_DISP_HOR_RES*MY_DISP_VER_RES)/2]__attribute__ (( section(".SDRAM_data"), used)) = {0};


/**********************
 *   GLOBAL FUNCTIONS
 **********************/


void lvgl_display_init(void)
{

	/*Create the display*/
	disp = lv_display_create(MY_DISP_HOR_RES, MY_DISP_VER_RES);

	/*Set the buffers*/
	lv_display_set_buffers(disp, (void*) buf1, (void*) buf2, sizeof(buf1), LV_DISPLAY_RENDER_MODE_PARTIAL);


	/* interrupt callback for DMA2D transfer */
	hdma2d.XferCpltCallback = disp_flush_complete;

	lv_display_set_flush_cb(disp, disp_flush);

}


/* LVGL 8.4.111
void lvgl_display_init (void)   // display initialization /* display is already initialized by cubemx-generated code
{
  lv_disp_draw_buf_init (&disp_buf,
     // (void*) LVGL_BUFFER_1_ADDR_AT_SDRAM,
     // (void*) LVGL_BUFFER_2_ADDR_AT_SDRAM,
	 (void*)buf1,
	 (void*)buf2,
     MY_DISP_HOR_RES * MY_DISP_VER_RES);

  // register the display in LVGL
  lv_disp_drv_init(&disp_drv);

  // set the resolution of the display
  disp_drv.hor_res = MY_DISP_HOR_RES;
  disp_drv.ver_res = MY_DISP_VER_RES;

  // set callback for display driver
  disp_drv.flush_cb = disp_flush;
  disp_drv.full_refresh = 0;
  disp_drv.direct_mode = 0;

  // interrupt callback for DMA2D transfer
  hdma2d.XferCpltCallback = disp_flush_complete;

  // set a display buffer
  disp_drv.draw_buf = &disp_buf;

  // finally register the driver
  lv_disp_drv_register(&disp_drv);
}
*/
/**********************
 *   STATIC FUNCTIONS
 **********************/

//static void disp_flush (lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_p) // 8.4.1
static void disp_flush (lv_display_t * display,
            const lv_area_t * area,
			uint8_t * px_map)
{
  lv_coord_t width = lv_area_get_width(area);
  lv_coord_t height = lv_area_get_height(area);

  SCB_CleanDCache_by_Addr((uint32_t*)px_map, width * height * 2);
  //SCB_CleanInvalidateDCache();         // cisti a inicializuje celou cache, muze byt pomale
  // setup pro ARGB888 - ale pomale
  /*
  DMA2D->FGPFCCR = DMA2D_INPUT_ARGB8888;
  DMA2D->CR = 0x0U << DMA2D_CR_MODE_Pos;    // no conversion ORIG
  DMA2D->FGMAR = (uint32_t)color_p;
  DMA2D->FGOR = 0;
  DMA2D->OPFCCR = DMA2D_OUTPUT_ARGB8888;
  DMA2D->OMAR = hltdc.LayerCfg[0].FBStartAdress + 4 * \
                (area->y1 * MY_DISP_HOR_RES + area->x1);
  */
  DMA2D->FGPFCCR = DMA2D_INPUT_RGB565;
  DMA2D->CR = 0x0U << DMA2D_CR_MODE_Pos;    // no conversion ORIG
  //DMA2D->FGMAR = (uint32_t)color_p;     // 8.4.1
  DMA2D->FGMAR = (uint32_t)px_map;       // 9.3.1
  DMA2D->FGOR = 0;
  DMA2D->OPFCCR = DMA2D_OUTPUT_RGB565;
  DMA2D->OMAR = hltdc.LayerCfg[0].FBStartAdress + 2 * \
                (area->y1 * MY_DISP_HOR_RES + area->x1);       // *2 (RGB565) nebo *4 ARGB8888
  DMA2D->OOR = MY_DISP_HOR_RES - width;
  DMA2D->NLR = (width << DMA2D_NLR_PL_Pos) | (height << DMA2D_NLR_NL_Pos);
  DMA2D->IFCR = 0x3FU;
  DMA2D->CR |= DMA2D_CR_TCIE;
  DMA2D->CR |= DMA2D_CR_START;

  SCB_InvalidateDCache_by_Addr((uint32_t*)hltdc.LayerCfg[0].FBStartAdress, width * height * 2);
}

//----------------------------------------------------------------------------------------------------
static void disp_flush_complete (DMA2D_HandleTypeDef *hdma2d)
{
  //lv_disp_flush_ready(&disp_drv);  // 8.4
  lv_disp_flush_ready(disp);    // 9.3
}
