#ifndef BSP_LCD_H
#define BSP_LCD_H

#ifdef __cplusplus
extern "C" {
#endif

///////////////////////// Includes ///////////////////////
#include "ili9341.h"
#include <stdint.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"

//////////////////////// Macros //////////////////////////

#define DISABLE     false
#define ENABLE      true
#define LOW         false
#define HIGH        true

/* LCD default width and height */
#define BSP_LCD_WIDTH           240U
#define BSP_LCD_HEIGHT          320U

/*Select pixel format */
#define	BSP_LCD_PIXEL_FMT_L8 		1
#define	BSP_LCD_PIXEL_FMT_RGB565	2
#define BSP_LCD_PIXEL_FMT_RGB666    3
#define	BSP_LCD_PIXEL_FMT_RGB888	4
#define BSP_LCD_PIXEL_FMT 			BSP_LCD_PIXEL_FMT_RGB565

/* 
* If LCD_BACKLIGHT_CONTROL_PWM is 0 then backlight pin will connect to 3.3v pin.
* If LCD_BACKLIGHT_CONTROL_PWM is 1 then backlight can be adjusted using PWM signal.
*/
#define LCD_BACKLIGHT_CONTROL_PWM       DISABLE

/* LCD backlight control pin selection */
#if (LCD_BACKLIGHT_CONTROL_PWM)
    #define BSP_LCD_LED_PIN        22       /* LED will be controlled by PWM */
#else
    #define BSP_LCD_LED_PIN        3.3      /* LED pin directly connected to the 3.3V pin */
#endif

/* LCD pins */
#define BSP_LCD_CSX_PIN         PICO_DEFAULT_SPI_CSN_PIN    // GP17
#define BSP_LCD_RESX_PIN        20                          // GP20
#define BSP_LCD_DCX_PIN         21                          // GP21
#define BSP_LCD_SDI_PIN         PICO_DEFAULT_SPI_TX_PIN     // GP19
#define BSP_LCD_SCK_PIN         PICO_DEFAULT_SPI_SCK_PIN    // GP18
#define BSP_LCD_SDO_PIN         PICO_DEFAULT_SPI_RX_PIN     // GP16
#define BSP_LCD_SPI_PORT        spi_default                 // SPI0


/*Select orientation*/
#define PORTRAIT  0
#define LANDSCAPE 1
#define BSP_LCD_ORIENTATION   PORTRAIT

#if(BSP_LCD_ORIENTATION == PORTRAIT)
	#define  BSP_LCD_ACTIVE_WIDTH 			BSP_LCD_WIDTH
	#define  BSP_LCD_ACTIVE_HEIGHT  		BSP_LCD_HEIGHT
#elif(BSP_LCD_ORIENTATION == LANDSCAPE)
	#define  BSP_LCD_ACTIVE_WIDTH 			BSP_LCD_HEIGHT
	#define  BSP_LCD_ACTIVE_HEIGHT 			BSP_LCD_WIDTH
#endif

/* Chip select pin management */
#define AUTO				 1
#define MANUAL				 0
/* 
*   If BSP_LCD_CS_MANAGE is AUTO then CS pin will be managed by SPI hardware itself. 
*   If BSP_LCD_CS_MANAGE is MANUAL then CS signal must be managed by yourself.
*/
#define BSP_LCD_CSX_MANAGE    MANUAL

/* DMA management */
#define USE_DMA 0       /* Enable to use DMA transfer */

typedef struct{
    uint16_t x1;
    uint16_t x2;
    uint16_t y1;
    uint16_t y2;
}lcd_area_t;

struct bsp_lcd;

typedef void (*bsp_lcd_dma_cplt_cb_t)(struct bsp_lcd*);
typedef void (*bsp_lcd_dma_err_cb_t)(struct bsp_lcd*);

typedef struct{
    uint8_t orientation;
    uint8_t pixel_format;
    uint8_t * draw_buffer1;
    uint8_t * draw_buffer2;
    uint32_t write_length;
    uint8_t *buff_to_draw;
    uint8_t *buff_to_flush;
    lcd_area_t area;
    bsp_lcd_dma_cplt_cb_t dma_cplt_cb;
    bsp_lcd_dma_err_cb_t dma_err_cb;
}bsp_lcd_t;





void bsp_lcd_init();
void bsp_lcd_set_orientation(int orientation);
void bsp_lcd_write(uint8_t *buffer, uint32_t nbytes);
void bsp_lcd_set_background_color(uint32_t rgb888);
void bsp_lcd_fill_rect(uint32_t rgb888, uint32_t x_start, uint32_t x_width,uint32_t y_start,uint32_t y_height);
void bsp_lcd_write_dma(uint32_t src_addr, uint32_t nbytes);
void bsp_lcd_set_display_area(uint16_t x1, uint16_t x2, uint16_t y1 , uint16_t y2);
void bsp_lcd_send_cmd_mem_write(void);
uint16_t bsp_lcd_convert_rgb888_to_rgb565(uint32_t rgb888);
void *bsp_lcd_get_draw_buffer1_addr(void);
void *bsp_lcd_get_draw_buffer2_addr(void);


#ifdef __cplusplus
}
#endif


#endif  /* BSP_LCD_H */