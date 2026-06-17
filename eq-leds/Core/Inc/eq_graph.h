/**
 * VisiTune EQ graph
 *
 * Authored by Tony Valencia
 */

#ifndef EQ_GRAPH_H
#define EQ_GRAPH_H

#include "ws2812.h"

// Demo modes
#define EQ_GRAPH_LINE 0
#define EQ_GRAPH_FFT  1

void eq_graph_set(ws2812_handleTypeDef *ws2812, uint8_t demo);
void eq_graph_tick(ws2812_handleTypeDef *ws2812);
void eq_graph_update_fft(ws2812_handleTypeDef *ws2812, float *fft_magnitudes, uint16_t num_bins);
void eq_graph_render(ws2812_handleTypeDef *ws2812);

#endif // EQ_GRAPH_H
