#include "tft.h"

// --- private state ---
static uint16_t tft_width  = TFT_W;
static uint16_t tft_height = TFT_H;

static void tft_hw_reset(void);
static int  in_bounds(int x, int y);

// --- private helpers ---
static void tft_hw_reset(void)
{
    HAL_GPIO_WritePin(TFT_RST_GPIO_Port, TFT_RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(TFT_RST_GPIO_Port, TFT_RST_Pin, GPIO_PIN_SET);
    HAL_Delay(120);
}

static int in_bounds(int x, int y)
{
    return (x >= 0) && (y >= 0) && (x < (int)tft_width) && (y < (int)tft_height);
}

// --- low-level write helpers (internal) ---
static inline void tft_write_cmd(uint8_t cmd)
{
    tft_dc_cmd();
    tft_select();
    HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
    tft_deselect();
}

static inline void tft_write_data(const uint8_t *buf, size_t n)
{
    tft_dc_data();
    tft_select();
    HAL_SPI_Transmit(&hspi1, (uint8_t*)buf, (uint16_t)n, HAL_MAX_DELAY);
    tft_deselect();
}

static inline void tft_write_data8(uint8_t v) { tft_write_data(&v, 1); }

// --- API impl ---
void tft_set_rotation(uint8_t r)
{
    static const uint8_t madctl_tbl[4] = {
        0x48, 0x28, 0x88, 0xE8
    };
    r &= 3;
    tft_write_cmd(0x36);
    tft_write_data8(madctl_tbl[r]);

    if (r & 1) { tft_width = 480; tft_height = 320; }
    else       { tft_width = 320; tft_height = 480; }
}

void tft_set_addr_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    uint8_t b[4];
    tft_write_cmd(0x2A);
    b[0]=x0>>8; b[1]=x0&0xFF; b[2]=x1>>8; b[3]=x1&0xFF;
    tft_write_data(b,4);

    tft_write_cmd(0x2B);
    b[0]=y0>>8; b[1]=y0&0xFF; b[2]=y1>>8; b[3]=y1&0xFF;
    tft_write_data(b,4);
}

void tft_init(void)
{
    tft_hw_reset();
    tft_write_cmd(0x11);        // SLPOUT
    HAL_Delay(120);
    tft_write_cmd(0x3A);        // COLMOD
    tft_write_data8(0x55);      // RGB565
    tft_set_rotation(2);
    tft_write_cmd(0x29);        // DISPON
    HAL_Delay(20);
}

void tft_draw_pixel(int x, int y, uint16_t rgb565)
{
    if (!in_bounds(x,y)) return;

    tft_set_addr_window((uint16_t)x,(uint16_t)y,(uint16_t)x,(uint16_t)y);

    uint8_t cmd = 0x2C;
    tft_dc_cmd(); tft_select();
    HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
    tft_dc_data();
    uint8_t two[2] = { (uint8_t)(rgb565>>8), (uint8_t)(rgb565 & 0xFF) };
    HAL_SPI_Transmit(&hspi1, two, 2, HAL_MAX_DELAY);
    tft_deselect();
}

void tft_fill_rect(int x, int y, int w, int h, uint16_t rgb565)
{
    if (w<=0 || h<=0) return;
    if (x<0){ w+=x; x=0; } if (y<0){ h+=y; y=0; }
    if (x+w>tft_width)  w = tft_width  - x;
    if (y+h>tft_height) h = tft_height - y;
    if (w<=0 || h<=0) return;

    tft_set_addr_window((uint16_t)x,(uint16_t)y,(uint16_t)(x+w-1),(uint16_t)(y+h-1));

    tft_dc_cmd(); tft_select();
    uint8_t cmd = 0x2C;
    HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
    tft_dc_data();

    uint8_t buf[128];
    uint8_t hb = (uint8_t)(rgb565>>8), lb = (uint8_t)(rgb565 & 0xFF);
    for (size_t i=0;i<sizeof(buf);i+=2){ buf[i]=hb; buf[i+1]=lb; }

    uint32_t px = (uint32_t)w*(uint32_t)h;
    while (px){
        size_t chunk_px = px > (sizeof(buf)/2) ? (sizeof(buf)/2) : px;
        HAL_SPI_Transmit(&hspi1, buf, (uint16_t)(chunk_px*2), HAL_MAX_DELAY);
        px -= chunk_px;
    }
    tft_deselect();
}

void tft_blit565(int x, int y, int w, int h, const uint8_t *img)
{
    if (w<=0 || h<=0) return;

    int x0=x, y0=y, x1=x+w-1, y1=y+h-1;
    if (x0>=tft_width || y0>=tft_height || x1<0 || y1<0) return;

    int sx=0, sy=0;
    if (x0<0){ sx=-x0; x0=0; }
    if (y0<0){ sy=-y0; y0=0; }
    if (x1>=tft_width)  x1=tft_width-1;
    if (y1>=tft_height) y1=tft_height-1;
    int cw = x1-x0+1;
    int ch = y1-y0+1;

    tft_set_addr_window((uint16_t)x0,(uint16_t)y0,(uint16_t)x1,(uint16_t)y1);

    tft_dc_cmd(); tft_select();
    uint8_t cmd = 0x2C;
    HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
    tft_dc_data();

    const uint8_t *src = img + ((size_t)sy * w + sx) * 2;
    size_t stride_bytes = (size_t)w * 2;

    for (int row=0; row<ch; ++row) {
        HAL_SPI_Transmit(&hspi1, (uint8_t*)src, (uint16_t)(cw*2), HAL_MAX_DELAY);
        src += stride_bytes;
    }
    tft_deselect();
}

static volatile uint8_t spi_tx_done = 0;

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == &hspi1) {
        tft_deselect();
        spi_tx_done = 1;
    }
}

void tft_stream_dma(const uint8_t *data, size_t nbytes)
{
    tft_dc_data();
    tft_select();
    spi_tx_done = 0;
    HAL_SPI_Transmit_DMA(&hspi1, (uint8_t*)data, (uint16_t)nbytes);
    while (!spi_tx_done) { /* wait */ }
}

void TFT_SetBacklightPercent(uint8_t pct){
    if (pct > 100) pct = 100;
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim1);
    uint32_t ccr = (pct * (arr + 1)) / 100;
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ccr);
}

void TFT_SetBacklight255(uint8_t val){
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim1);
    uint32_t ccr = (val * (arr + 1)) / 255;
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ccr);
}

// ---------------- 5x7 FONT + TEXT RENDERING ----------------

// 5x7 font stored as 7 rows of 5 bits (LSB = leftmost pixel).
// We implement digits '0'-'9', uppercase 'A'-'Z', space, and '?'.
// Lowercase is mapped to uppercase before lookup.

static const uint8_t font5x7_digit[10][7] = {
    // '0'
    {0x0E, 0x11, 0x13, 0x15, 0x19, 0x11, 0x0E},
    // '1'
    {0x04, 0x0C, 0x04, 0x04, 0x04, 0x04, 0x0E},
    // '2'
    {0x0E, 0x11, 0x01, 0x02, 0x04, 0x08, 0x1F},
    // '3'
    {0x0E, 0x11, 0x01, 0x06, 0x01, 0x11, 0x0E},
    // '4'
    {0x02, 0x06, 0x0A, 0x12, 0x1F, 0x02, 0x02},
    // '5'
    {0x1F, 0x10, 0x1E, 0x01, 0x01, 0x11, 0x0E},
    // '6'
    {0x06, 0x08, 0x10, 0x1E, 0x11, 0x11, 0x0E},
    // '7'
    {0x1F, 0x01, 0x02, 0x04, 0x08, 0x08, 0x08},
    // '8'
    {0x0E, 0x11, 0x11, 0x0E, 0x11, 0x11, 0x0E},
    // '9'
    {0x0E, 0x11, 0x11, 0x0F, 0x01, 0x02, 0x0C},
};

static const uint8_t font5x7_upper[26][7] = {
    // 'A'
    {0x0E, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11},
    // 'B'
    {0x1E, 0x11, 0x11, 0x1E, 0x11, 0x11, 0x1E},
    // 'C'
    {0x0E, 0x11, 0x10, 0x10, 0x10, 0x11, 0x0E},
    // 'D'
    {0x1C, 0x12, 0x11, 0x11, 0x11, 0x12, 0x1C},
    // 'E'
    {0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x1F},
    // 'F'
    {0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x10},
    // 'G'
    {0x0E, 0x11, 0x10, 0x17, 0x11, 0x11, 0x0F},
    // 'H'
    {0x11, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11},
    // 'I'
    {0x0E, 0x04, 0x04, 0x04, 0x04, 0x04, 0x0E},
    // 'J'
    {0x01, 0x01, 0x01, 0x01, 0x11, 0x11, 0x0E},
    // 'K'
    {0x11, 0x12, 0x14, 0x18, 0x14, 0x12, 0x11},
    // 'L'
    {0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x1F},
    // 'M'
    {0x11, 0x1B, 0x15, 0x15, 0x11, 0x11, 0x11},
    // 'N'
    {0x11, 0x19, 0x15, 0x13, 0x11, 0x11, 0x11},
    // 'O'
    {0x0E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E},
    // 'P'
    {0x1E, 0x11, 0x11, 0x1E, 0x10, 0x10, 0x10},
    // 'Q'
    {0x0E, 0x11, 0x11, 0x11, 0x15, 0x12, 0x0D},
    // 'R'
    {0x1E, 0x11, 0x11, 0x1E, 0x14, 0x12, 0x11},
    // 'S'
    {0x0F, 0x10, 0x10, 0x0E, 0x01, 0x01, 0x1E},
    // 'T'
    {0x1F, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04},
    // 'U'
    {0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E},
    // 'V'
    {0x11, 0x11, 0x11, 0x11, 0x11, 0x0A, 0x04},
    // 'W'
    {0x11, 0x11, 0x11, 0x15, 0x15, 0x1B, 0x11},
    // 'X'
    {0x11, 0x11, 0x0A, 0x04, 0x0A, 0x11, 0x11},
    // 'Y'
    {0x11, 0x11, 0x0A, 0x04, 0x04, 0x04, 0x04},
    // 'Z'
    {0x1F, 0x01, 0x02, 0x04, 0x08, 0x10, 0x1F},
};

// Fallback '?' glyph
static const uint8_t font5x7_qmark[7] = {
    0x0E,
    0x11,
    0x01,
    0x02,
    0x04,
    0x00,
    0x04,
};

static const uint8_t *tft_lookup_glyph(char c)
{
    // Space
    if (c == ' ') {
        static const uint8_t blank[7] = {0,0,0,0,0,0,0};
        return blank;
    }

    // Normalize to uppercase
    if (c >= 'a' && c <= 'z') {
        c = (char)(c - 'a' + 'A');
    }

    if (c >= '0' && c <= '9') {
        return font5x7_digit[(int)(c - '0')];
    }

    if (c >= 'A' && c <= 'Z') {
        return font5x7_upper[(int)(c - 'A')];
    }

    // Anything else → '?'
    return font5x7_qmark;
}

// Draw a single 5x7 character at (x,y) in the given color.
void tft_draw_char(int x, int y, char c, uint16_t color)
{
    const uint8_t *glyph = tft_lookup_glyph(c);

    for (int row = 0; row < 7; ++row) {
        uint8_t rowbits = glyph[row];

        // Treat bit 4 as leftmost pixel, bit 0 as rightmost.
        for (int col = 0; col < 5; ++col) {
            int bit = 4 - col;                  // reverse the bit index
            if (rowbits & (1u << bit)) {
                tft_draw_pixel(x + col, y + row, color);
            }
        }
    }
}

// Draw a single 5x7 character scaled by `scale`.
static void tft_draw_char_scaled(int x, int y, char c, uint16_t color, int scale)
{
    const uint8_t *glyph = tft_lookup_glyph(c);

    for (int row = 0; row < 7; ++row) {
        uint8_t rowbits = glyph[row];

        for (int col = 0; col < 5; ++col) {
            int bit = 4 - col;  // MSB = leftmost pixel
            if (rowbits & (1u << bit)) {
                int px = x + col * scale;
                int py = y + row * scale;
                // Fill an scale×scale block for each "on" pixel
                tft_fill_rect(px, py, scale, scale, color);
            }
        }
    }
}


// Draw a null-terminated string. Each char is 5x7 with 1px spacing.
// Draw a null-terminated string. Scaled 5x7 font.
void tft_draw_text(int x, int y, const char *str, uint16_t color)
{
    const int scale        = 3;                 // was 3; bump to 4 for bigger text
    const int char_advance = 5 * scale + scale; // 5 pixels wide + 1px spacing, scaled
    const int line_height  = 7 * scale + scale; // 7 pixels tall + 1px spacing, scaled

    int cursor_x = x;

    while (*str) {
        char c = *str++;

        if (c == '\n') {
            y += line_height;
            cursor_x = x;
            continue;
        }

        tft_draw_char_scaled(cursor_x, y, c, color, scale);
        cursor_x += char_advance;
    }
}


// Draw a null-terminated string with word wrapping within max_width pixels.
void tft_draw_text_wrap(int x, int y, const char *str, uint16_t color, int max_width)
{
    const int scale        = 3;                 // must match tft_draw_text
    const int char_advance = 5 * scale + scale; // 5 pixels + spacing
    const int line_height  = 7 * scale + scale; // 7 pixels + spacing

    if (max_width <= 0) return;

    int max_chars = max_width / char_advance;
    if (max_chars <= 0) max_chars = 1;

    while (*str) {
        // Skip leading spaces at the start of a line
        while (*str == ' ') str++;

        const char *line_start = str;
        int count = 0;
        int last_space = -1;

        // Figure out how many chars fit on this line
        while (str[count] && str[count] != '\n' && count < max_chars) {
            if (str[count] == ' ')
                last_space = count;
            count++;
        }

        int line_len = count;

        // If we hit the max chars and we're in the middle of a word,
        // back up to the last space so we wrap nicely.
        if (str[count] && str[count] != '\n' && last_space != -1) {
            line_len = last_space;
        }

        // Draw [line_start, line_start + line_len)
        int cursor_x = x;
        for (int i = 0; i < line_len; ++i) {
            char c = line_start[i];
            if (c == ' ') {
                cursor_x += char_advance;
                continue;
            }
            tft_draw_char_scaled(cursor_x, y, c, color, scale);
            cursor_x += char_advance;
        }

        // Advance str by the part we just rendered
        str = line_start + line_len;

        // Skip any spaces right after that (so we don't start a new line with space)
        while (*str == ' ') str++;

        // Handle explicit newline
        if (*str == '\n') {
            str++;
        }

        // Move to next line in the footer band
        y += line_height;
    }
}


