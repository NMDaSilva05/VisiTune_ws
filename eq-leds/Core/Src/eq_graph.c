/**
 * VisiTune EQ Graph
 * Designed for 8x32 LED matrix with horizontal serpentine layout
 *
 * Authored by Tony Valencia
 */

#include "main.h"
#include "ws2812.h"
#include "eq_graph.h"
#include <string.h>

uint8_t active_demo_eq = 0;

// Color gradient from green (low) to red (high)
const uint8_t led_line_colors_eq[][3] = {
    { 0, 100, 0 },		// Green  (G=100, R=0, B=0)
    { 100, 50, 0 },	    // Yellow (G=100, R=50, B=0)
    { 100, 0, 0}		// Red    (G=0, R=100, B=0)
};

#define NUM_COLORS (sizeof(led_line_colors_eq) / sizeof(led_line_colors_eq[0]))

// Matrix configuration for 8-wide × 32-tall strip
#define MATRIX_WIDTH 8        // Number of frequency bands (X-axis)
#define MATRIX_HEIGHT 32      // LEDs per column (Y-axis)
#define TOTAL_LEDS (MATRIX_WIDTH * MATRIX_HEIGHT)  // 256 LEDs

// Visual configuration
#define DECAY_RATE 0.85f      // How fast bars fall (0.0-1.0, higher = slower)
#define PEAK_HOLD_TIME 500    // Peak dot hold time in ms

// FFT data storage
static float band_values[MATRIX_WIDTH] = {0};      // Current smoothed values

/**
 * Convert X,Y coordinates to LED index for horizontal serpentine layout
 *
 * Layout (8 wide, 32 tall, starting bottom-left):
 * Row 0 (bottom): LED 0 → 1 → 2 → 3 → 4 → 5 → 6 → 7
 * Row 1:          LED 15 ← 14 ← 13 ← 12 ← 11 ← 10 ← 9 ← 8
 * Row 2:          LED 16 → 17 → 18 → 19 → 20 → 21 → 22 → 23
 * ...and so on (alternating direction each row)
 *
 * @param x Column (0-7)
 * @param y Row from bottom (0-31, where 0 is bottom)
 * @return LED index in strip
 */
static uint16_t xy_to_led_index(uint8_t x, uint8_t y) {
    if (x >= MATRIX_WIDTH || y >= MATRIX_HEIGHT) {
        return 0xFFFF;  // Out of bounds
    }

    uint16_t index;

    if (y % 2 == 0) {
        // Even rows: left to right
        index = y * MATRIX_WIDTH + x;
    } else {
        // Odd rows: right to left (serpentine)
        index = y * MATRIX_WIDTH + (MATRIX_WIDTH - 1 - x);
    }

    return index;
}

void eq_graph_set(ws2812_handleTypeDef *ws2812, uint8_t demo) {
    active_demo_eq = demo;

    // Clear the display when switching modes
    if (demo == EQ_GRAPH_FFT) {
        memset(band_values, 0, sizeof(band_values));
        zeroLedValues(ws2812);
    }
}

/**
 * Update EQ graph with FFT data
 *
 * @param ws2812 LED strip handle
 * @param fft_magnitudes Array of FFT magnitude values (normalized 0.0-1.0)
 * @param num_bins Number of FFT bins provided
 */
void eq_graph_update_fft(ws2812_handleTypeDef *ws2812, float *fft_magnitudes, uint16_t num_bins) {

    // Calculate how many FFT bins to average per column
    uint16_t bins_per_column = num_bins / MATRIX_WIDTH;
    if (bins_per_column < 1) bins_per_column = 1;

    // Process each frequency band (column)
    for (uint8_t col = 0; col < MATRIX_WIDTH; col++) {
        // Average FFT bins for this column
        float band_sum = 0.0f;
        uint16_t start_bin = col * bins_per_column;
        uint16_t end_bin = start_bin + bins_per_column;
        if (end_bin > num_bins) end_bin = num_bins;

        for (uint16_t bin = start_bin; bin < end_bin; bin++) {
            band_sum += fft_magnitudes[bin];
        }
        float band_avg = band_sum / bins_per_column;

        // Smooth the value (low-pass filter)
        band_values[col] = band_values[col] * DECAY_RATE + band_avg * (1.0f - DECAY_RATE);

        // Clamp values to 0.0-1.0 range
        if (band_values[col] > 1.0f) band_values[col] = 1.0f;
        if (band_values[col] < 0.0f) band_values[col] = 0.0f;
    }
}

/**
 * Render the EQ graph to LED strip (8x32 matrix)
 */
void eq_graph_render(ws2812_handleTypeDef *ws2812) {
    // Clear all LEDs
    zeroLedValues(ws2812);

    // Draw each column (frequency band)
    for (uint8_t col = 0; col < MATRIX_WIDTH; col++) {
        // Calculate bar height (0 to MATRIX_HEIGHT)
        uint16_t bar_height = (uint16_t)(band_values[col] * MATRIX_HEIGHT);

        // Draw vertical bar from bottom up
        for (uint16_t row = 0; row < bar_height; row++) {
			float height_ratio = (float)row / (float)(MATRIX_HEIGHT - 1);

			// Map to color index
			float color_position = height_ratio * (NUM_COLORS - 1);
			uint8_t color_index = (uint8_t)(color_position + 0.5f);  // Round to nearest
			if (color_index >= NUM_COLORS) color_index = NUM_COLORS - 1;

			// Convert X,Y to LED index
			uint16_t led_index = xy_to_led_index(col, row);

			if (led_index < ws2812->leds) {
				setLedValues(ws2812, led_index,
							led_line_colors_eq[color_index][0],
							led_line_colors_eq[color_index][1],
							led_line_colors_eq[color_index][2]);
			}
        }
    }
}

void eq_graph_test_columns(ws2812_handleTypeDef *ws2812) {
    static uint8_t test_col = 0;
    static uint32_t next_test = 0;
    uint32_t now = uwTick;

    if (now >= next_test) {
        zeroLedValues(ws2812);

        // Light up entire column in white
        for (uint8_t row = 0; row < MATRIX_HEIGHT; row++) {
            uint16_t led_index = xy_to_led_index(test_col, row);
            if (led_index < ws2812->leds) {
                setLedValues(ws2812, led_index, 255, 255, 255);
            }
        }

        test_col = (test_col + 1) % MATRIX_WIDTH;
        next_test = now + 500;  // Switch column every 500ms
    }
}

void eq_graph_tick(ws2812_handleTypeDef *ws2812) {
    switch (active_demo_eq) {
    case EQ_GRAPH_FFT:
        // FFT mode - render the EQ graph
        eq_graph_render(ws2812);
        break;
    default:
        // Do nothing
        break;
    }
}
