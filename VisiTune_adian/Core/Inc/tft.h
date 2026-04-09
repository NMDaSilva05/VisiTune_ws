#ifndef TFT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l4xx_hal.h"
#include <stdint.h>
#include <stddef.h>

// --- Pins (override in your project if you want) ---
#ifndef TFT_CS_GPIO_Port
#define TFT_CS_GPIO_Port   GPIOC
#endif
#ifndef TFT_CS_Pin
#define TFT_CS_Pin         GPIO_PIN_8
#endif

#ifndef TFT_DC_GPIO_Port
#define TFT_DC_GPIO_Port   GPIOC
#endif
#ifndef TFT_DC_Pin
#define TFT_DC_Pin         GPIO_PIN_9
#endif

#ifndef TFT_RST_GPIO_Port
#define TFT_RST_GPIO_Port  GPIOC
#endif
#ifndef TFT_RST_Pin
#define TFT_RST_Pin        GPIO_PIN_6
#endif

// Panel logical size for rotation=0
#define TFT_W  480
#define TFT_H  320

// HAL handles provided by main.c (CubeMX)
extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim1;

// -------- small inline helpers OK in header --------
static inline void tft_select(void)   { HAL_GPIO_WritePin(TFT_CS_GPIO_Port, TFT_CS_Pin, GPIO_PIN_RESET); }
static inline void tft_deselect(void) { HAL_GPIO_WritePin(TFT_CS_GPIO_Port, TFT_CS_Pin, GPIO_PIN_SET);   }

static inline void tft_dc_cmd(void)   { HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_RESET); }
static inline void tft_dc_data(void)  { HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_SET);   }

static inline uint16_t color565(uint8_t r, uint8_t g, uint8_t b)
{
    return (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
}

// ------------- public API (declarations) -------------
void     tft_init(void);
void     tft_set_rotation(uint8_t r);
void     tft_draw_pixel(int x, int y, uint16_t rgb565);
void     tft_fill_rect(int x, int y, int w, int h, uint16_t rgb565);
void     tft_blit565(int x, int y, int w, int h, const uint8_t *img);
void     tft_set_addr_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
void	 tft_draw_char(int x, int y, char c, uint16_t color);
void 	 tft_draw_text(int x, int y, const char *str, uint16_t color);
static void tft_draw_char_scaled(int x, int y, char c, uint16_t color, int scale);
void 	 tft_draw_text_wrap(int x, int y, const char *str, uint16_t color, int max_width);


// Optional DMA helpers / callbacks you use
void     tft_stream_dma(const uint8_t *data, size_t nbytes);

// Backlight helpers (TIM1 CH2 in your setup)
void     TFT_SetBacklightPercent(uint8_t pct);
void     TFT_SetBacklight255(uint8_t val);

#ifdef __cplusplus
}
#endif
#endif // TFT_H
