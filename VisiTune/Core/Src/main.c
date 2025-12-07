/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>

#include "MIDI.h"
#include "arm_math.h"
#include "eq_graph.h"
#include "packets.h"
#include "tft.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PKT_SYNC_SIZE 2                                    // 0xA5, 0x5A
#define PKT_HEADER_SIZE (sizeof(pkt_header_t))             // 7 bytes (packed)
#define PKT_PREFIX_SIZE (PKT_SYNC_SIZE + PKT_HEADER_SIZE)  // 9 bytes before payload
#define PKT_CRC_SIZE 2

#define UART_ACK_HEADER 'H'
#define UART_ACK_OK 'S'
#define UART_ACK_ERR 'E'
#define UART_ACK_NEXT_AUDIO_CHUNK 'A'

// Direction: STM32 -> PC
typedef enum { BTN_EVENT_PLAY_PAUSE = 'P', BTN_EVENT_NEXT = 'N', BTN_EVENT_PREV = 'B' } btn_event_t;

#define BUTTON_FRAME_PREFIX 0xF0u

// Packet size/overhead (must match Python script)
#define PKT_OVERHEAD (PKT_SYNC_SIZE + PKT_HEADER_SIZE + PKT_CRC_SIZE)  // 11
#define PKT_MAX_SIZE 0xFFFFu  // total packet: sync+header+payload+CRC

#define IMG_PKT_MAX_SIZE PKT_MAX_SIZE
#define AUD_PKT_MAX_SIZE PKT_MAX_SIZE

#define IMG_MAX_PAYLOAD_BYTES (IMG_PKT_MAX_SIZE - PKT_OVERHEAD)
#define AUD_MAX_PAYLOAD_BYTES 4096  // (AUD_PKT_MAX_SIZE - PKT_OVERHEAD)

// DAC circular buffer: 2 halves, each half holds one max audio payload worth of samples
// 2 bytes per 16-bit PCM sample => samples = bytes/2
#define DAC_HALF_SAMPLES (AUD_MAX_PAYLOAD_BYTES / 2u)
#define DAC_BUF_SAMPLES (2u * DAC_HALF_SAMPLES)

// LED PD
#define LED_GPIO_Port GPIOD
#define LED_Pin 14
#define Number_LEDs 256
#define NUM_BINS 256

// Text packets
#define TEXT_MAX_PAYLOAD 128  // 1 byte field_id + up to 127 chars
volatile uint8_t text_pkt_ready = 0;
volatile uint16_t text_pkt_len = 0;
uint8_t text_buf[TEXT_MAX_PAYLOAD + PKT_CRC_SIZE];

char current_title[64] = "";
char current_artist[64] = "";
char current_album[64] = "";
char current_other[64] = "";

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac1_ch1;

UART_HandleTypeDef hlpuart1;
DMA_HandleTypeDef hdma_lpuart1_rx;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi2_rx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
DMA_HandleTypeDef hdma_tim4_ch3;

/* USER CODE BEGIN PV */

// ADC declarations
uint16_t adc_buf[4];
volatile uint8_t rewind_flag = 0;
volatile uint8_t forward_flag = 0;
volatile uint8_t pause_button_flag = 0;
volatile uint8_t paused = 0;

// MIDI declarations
char keyPress;
uint32_t MIDI_index = 0;
uint8_t MIDI_playing = 0;
uint8_t sound_effect_index = 0;
const uint16_t* MIDI_effects[] = {crowd_cheer, bass_kick, eh,     hi_hat,
                                  medium_kick, snare1,    snare2, vine_boom};
const uint32_t MIDI_lengths[] = {crowd_cheer_len, bass_kick_len, eh_len,     hi_hat_len,
                                 medium_kick_len, snare1_len,    snare2_len, vine_boom_len};

// UART RX context
typedef enum {
    RX_STATE_WAIT_PREFIX,   // waiting for 2 sync + 7 header bytes
    RX_STATE_WAIT_PAYLOAD,  // waiting for payload (image or audio)
    RX_STATE_WAIT_CRC       // waiting for audio CRC (2 bytes)
} uart_rx_state_t;

typedef struct {
    uart_rx_state_t state;
    uint8_t prefix_buf[PKT_PREFIX_SIZE];  // 9 bytes: sync + header
    pkt_header_t header;
    uint16_t payload_len;
    uint8_t* payload_dst;  // destination buffer for payload+CRC
} uart_rx_ctx_t;

uart_rx_ctx_t g_uart_rx;

// Buffers
uint16_t dac_buf[DAC_BUF_SAMPLES];  // DAC circular buffer

uint8_t image_pkt_buf[IMG_MAX_PAYLOAD_BYTES + PKT_CRC_SIZE];

// Audio RX views onto dac_buf halves
uint8_t* audio_pkt_buf0 = (uint8_t*)dac_buf;                     // first half
uint8_t* audio_pkt_buf1 = (uint8_t*)&dac_buf[DAC_HALF_SAMPLES];  // second half

// NEW: CRC storage for audio packets
uint8_t audio_crc_buf[PKT_CRC_SIZE];

// LED Driver
ws2812_handleTypeDef ws2812;
uint32_t now = 0;         // timing variable
uint32_t next_demo = 10;  // timing variable
uint32_t next_fft = 0;
int idle_leds = 0;
float fft_magnitudes[DAC_HALF_SAMPLES];  // FFT data buffer

// Flags
volatile uint8_t image_pkt_ready = 0;
volatile uint16_t image_pkt_len = 0;

volatile uint8_t audio_pkt_ready = 0;
volatile uint8_t audio_write_idx = 0;  // 0 or 1
volatile uint16_t audio_pkt_len = 0;

// Which halves of dac_buf are free to write into
volatile uint8_t half0_free = 1;  // dac_buf[0 .. DAC_HALF_SAMPLES-1]
volatile uint8_t half1_free = 1;  // dac_buf[DAC_HALF_SAMPLES .. end]

volatile uint8_t audio_stream_active = 0;

// Stream state for DAC priming
volatile uint8_t dac_running = 0;        // 0 = DAC DMA stopped, 1 = running
volatile uint8_t audio_prime_count = 0;  // how many good packets we've accepted this stream

volatile uint16_t ctrl_pkt_len = 0;
volatile uint8_t ctrl_pkt_ready = 0;
uint8_t ctrl_buf[16];

q15_t filter_buf[DAC_HALF_SAMPLES];
/* FFT size auto-derives from audio payload: payload_bytes / 2 samples = FFT block size */
#define FFT_SIZE (AUD_MAX_PAYLOAD_BYTES / 2u)
#define FFT_INCREMENT (FFT_SIZE / 2)

float32_t fft_in[FFT_SIZE];
float32_t fft_out[FFT_SIZE];
float32_t overlap[FFT_INCREMENT];
static float32_t window[FFT_SIZE];
q15_t debug_out_buffer[DAC_HALF_SAMPLES];

arm_rfft_fast_instance_f32 rfft;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
static void uart_start_header_rx(void);
static void processFFT(const q15_t* in, q15_t* out, uint16_t num_samples);
static void ADC_ReadAll_Polling(void);
static void protocol_soft_reset(void);
static void MIDI_Check(void);
static char read_keypad(void);
static void handle_ctrl_packet(void);
static void handle_image_packet(void);
static void handle_audio_packet(void);
static void handle_text_packet(void);
static void handle_button_presses(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

arm_biquad_casd_df1_inst_f32 EQ;
#define NUM_EQ_STAGES 4
float eqCoeffs[5 * NUM_EQ_STAGES];
float eqState[4 * NUM_EQ_STAGES];

// EQ Stuff
typedef struct {
    float a0, a1, a2;
    float b1, b2;
    // float z1, z2;
} biquad_t;

// static inline float biquad_run(biquad_t* bq, float in)
//{
//     float out = in * bq->a0 + bq->z1;
//     bq->z1 = in * bq->a1 + bq->z2 - bq->b1 * out;
//     bq->z2 = in * bq->a2 - bq->b2 * out;
//     return out;
// }

// EQ LED GRAPH functions

void HAL_TIM_PWM_PulseFinishedHalfCpltCallback(TIM_HandleTypeDef* htim) {
    if (htim->Instance == TIM4) {
        ws2812_update_buffer(&ws2812, &ws2812.dma_buffer[0]);
    }
}

// Done sending the second half of the DMA buffer - this can now be safely updated
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef* htim) {
    if (htim->Instance == TIM4) {
        ws2812_update_buffer(&ws2812, &ws2812.dma_buffer[BUFFER_SIZE]);
    }
}

void fft_to_magnitude(const float32_t* fft_raw, float32_t* mag_bins, uint32_t num_bins) {
    // Bin 0 (DC)
    mag_bins[0] = fabsf(fft_raw[0]);

    // Bin 1 (Nyquist, real only)
    if (num_bins > 1) {
        mag_bins[1] = fabsf(fft_raw[1]);
    }

    // Bins 2 to num_bins-1: complex values, packed as interleaved real/imag starting at fft_raw[2]
    for (uint32_t k = 2; k < num_bins; k++) {
        float32_t re = fft_raw[2 * (k - 1)];      // real part
        float32_t im = fft_raw[2 * (k - 1) + 1];  // imag part
        mag_bins[k] = sqrtf(re * re + im * im);
    }
}

void magnitude_to_dB(float32_t* mag_bins, uint32_t num_bins) {
    const float32_t floor_val = 1e-12f;
    const float32_t scale = 20.0f / logf(10.0f);

    for (uint32_t k = 0; k < num_bins; k++) {
        float32_t x = mag_bins[k];
        if (x < floor_val) x = floor_val;

        mag_bins[k] = scale * logf(x);  // => dB value
                                        //        mag_bins[k] /= DAC_HALF_SAMPLES;
        //        mag_bins[k] = mag_bins[k] - (80.0f);   // shift -80..0 dB → 0..80
    }
}

float32_t mag_bins[NUM_BINS];  // magnitude per bin

void EQ_LED_Tick(arm_rfft_fast_instance_f32* rfft) {
    now = uwTick;

    float32_t filter_buf_f32[DAC_HALF_SAMPLES];
    float32_t fft_raw[NUM_BINS];  // raw FFT data

    memset(filter_buf_f32, 0, sizeof(filter_buf_f32));
    memset(fft_raw, 0, sizeof(fft_raw));
    memset(mag_bins, 0, sizeof(mag_bins));

    if (idle_leds) {
        zeroLedValues(&ws2812);
    }

    if (now >= next_fft && !idle_leds) {
        arm_q15_to_float(filter_buf, filter_buf_f32, DAC_HALF_SAMPLES);
        arm_rfft_fast_f32(rfft, filter_buf_f32, fft_raw, 0);
        fft_to_magnitude(fft_raw, mag_bins, NUM_BINS);
        magnitude_to_dB(mag_bins, NUM_BINS);
        eq_graph_update_fft(&ws2812, mag_bins, NUM_BINS);
        next_fft = now + 50;  // Update every 50ms (20 FPS)
    }

    // Render EQ graph every 10ms
    if (now >= next_demo) {
        eq_graph_tick(&ws2812);
        next_demo = now + 50;
    }
}

#include <math.h>

static biquad_t EQ_low, EQ_mid, EQ_high, EQ_window;

static void biquad_low_shelf(biquad_t* bq, float gainDB, float freq, float fs);
static void biquad_peak(biquad_t* bq, float gainDB, float freq, float Q, float fs);
static void biquad_high_shelf(biquad_t* bq, float gainDB, float freq, float fs);
void bpf_constant0dB(biquad_t* bq, float fc, float Q, float fs, uint8_t windowPower);

void EQ_setGains(float lowDB, float midDB, float highDB, uint8_t windowFilterStatus,
                 float window_freq) {
    float fs = 24000.0f;  // your DAC sample rate
    float midFreq = 1200.0f;
    float Q = 0.7f;

    biquad_low_shelf(&EQ_low, lowDB, 200.0f, fs);
    biquad_peak(&EQ_mid, midDB, midFreq, Q, fs);
    biquad_high_shelf(&EQ_high, highDB, 6000.0f, fs);
    bpf_constant0dB(&EQ_window, window_freq, Q, fs, windowFilterStatus);

    // Fill coeff array
    eqCoeffs[0] = EQ_low.a0;
    eqCoeffs[1] = EQ_low.a1;
    eqCoeffs[2] = EQ_low.a2;
    eqCoeffs[3] = -1 * EQ_low.b1;
    eqCoeffs[4] = -1 * EQ_low.b2;

    eqCoeffs[5] = EQ_mid.a0;
    eqCoeffs[6] = EQ_mid.a1;
    eqCoeffs[7] = EQ_mid.a2;
    eqCoeffs[8] = -1 * EQ_mid.b1;
    eqCoeffs[9] = -1 * EQ_mid.b2;

    eqCoeffs[10] = EQ_high.a0;
    eqCoeffs[11] = EQ_high.a1;
    eqCoeffs[12] = EQ_high.a2;
    eqCoeffs[13] = -1 * EQ_high.b1;
    eqCoeffs[14] = -1 * EQ_high.b2;

    eqCoeffs[15] = EQ_window.a0;
    eqCoeffs[16] = EQ_window.a1;
    eqCoeffs[17] = EQ_window.a2;
    eqCoeffs[18] = -1 * EQ_window.b1;
    eqCoeffs[19] = -1 * EQ_window.b2;

    static int first_init = 1;
    if (first_init) {
        arm_biquad_cascade_df1_init_f32(&EQ, NUM_EQ_STAGES, eqCoeffs, eqState);
        first_init = 0;
    }
}

static void biquad_low_shelf(biquad_t* bq, float gainDB, float freq, float fs) {
    float A = powf(10.0f, gainDB / 40.0f);
    float w0 = 2.0f * M_PI * freq / fs;
    float cosw0 = cosf(w0);
    float sinw0 = sinf(w0);

    float alpha = sinw0 / 2.0f * sqrtf((A + 1 / A) * (1.0f / 0.707f - 1) + 2);

    float b0 = A * ((A + 1) - (A - 1) * cosw0 + 2 * sqrtf(A) * alpha);
    float b1 = 2 * A * ((A - 1) - (A + 1) * cosw0);
    float b2 = A * ((A + 1) - (A - 1) * cosw0 - 2 * sqrtf(A) * alpha);
    float a0 = (A + 1) + (A - 1) * cosw0 + 2 * sqrtf(A) * alpha;
    float a1 = -2 * ((A - 1) + (A + 1) * cosw0);
    float a2 = (A + 1) + (A - 1) * cosw0 - 2 * sqrtf(A) * alpha;

    bq->a0 = b0 / a0;
    bq->a1 = b1 / a0;
    bq->a2 = b2 / a0;
    bq->b1 = a1 / a0;
    bq->b2 = a2 / a0;
    // bq->z1 = bq->z2 = 0;
}

static void biquad_peak(biquad_t* bq, float gainDB, float freq, float Q, float fs) {
    float A = powf(10.0f, gainDB / 40.0f);
    float w0 = 2.0f * M_PI * freq / fs;
    float cosw0 = cosf(w0);
    float sinw0 = sinf(w0);
    float alpha = sinw0 / (2.0f * Q);

    float b0 = 1 + alpha * A;
    float b1 = -2 * cosw0;
    float b2 = 1 - alpha * A;
    float a0 = 1 + alpha / A;
    float a1 = -2 * cosw0;
    float a2 = 1 - alpha / A;

    bq->a0 = b0 / a0;
    bq->a1 = b1 / a0;
    bq->a2 = b2 / a0;
    bq->b1 = a1 / a0;
    bq->b2 = a2 / a0;
    // bq->z1 = bq->z2 = 0;
}

static void biquad_high_shelf(biquad_t* bq, float gainDB, float freq, float fs) {
    float A = powf(10.0f, gainDB / 40.0f);
    float w0 = 2.0f * M_PI * freq / fs;
    float cosw0 = cosf(w0);
    float sinw0 = sinf(w0);

    float alpha = sinw0 / 2.0f * sqrtf((A + 1 / A) * (1.0f / 0.707f - 1) + 2);

    float b0 = A * ((A + 1) + (A - 1) * cosw0 + 2 * sqrtf(A) * alpha);
    float b1 = -2 * A * ((A - 1) + (A + 1) * cosw0);
    float b2 = A * ((A + 1) + (A - 1) * cosw0 - 2 * sqrtf(A) * alpha);
    float a0 = (A + 1) - (A - 1) * cosw0 + 2 * sqrtf(A) * alpha;
    float a1 = 2 * ((A - 1) - (A + 1) * cosw0);
    float a2 = (A + 1) - (A - 1) * cosw0 - 2 * sqrtf(A) * alpha;

    bq->a0 = b0 / a0;
    bq->a1 = b1 / a0;
    bq->a2 = b2 / a0;
    bq->b1 = a1 / a0;
    bq->b2 = a2 / a0;
    // bq->z1 = bq->z2 = 0;
}

void bpf_constant0dB(biquad_t* bq, float fc, float Q, float fs, uint8_t windowPower) {
    if (windowPower) {
        float w0 = 2.0f * M_PI * fc / fs;
        float sinw0 = sinf(w0);
        float cosw0 = cosf(w0);
        float alpha = sinw0 / (2.0f * Q);

        // Cookbook formula #2 (0 dB peak gain)
        float b0 = Q * sinw0;
        float b1 = 0.0f;
        float b2 = -Q * sinw0;
        float a0 = 1.0f + alpha;
        float a1 = -2.0f * cosw0;
        float a2 = 1.0f - alpha;

        // Normalize to a0
        bq->a0 = b0 / a0;
        bq->a1 = b1 / a0;
        bq->a2 = b2 / a0;
        bq->b1 = a1 / a0;
        bq->b2 = a2 / a0;
    } else {
        bq->a0 = 1;
        bq->a1 = 0;
        bq->a2 = 0;
        bq->b1 = 0;
        bq->b2 = 0;
    }
}

void processEQ(int16_t* in, int16_t* out, uint16_t n) {
    static float inF[DAC_HALF_SAMPLES];
    static float outF[DAC_HALF_SAMPLES];

    for (int i = 0; i < n; i++) inF[i] = (float)in[i] / 32768.0f;

    arm_biquad_cascade_df1_f32(&EQ, inF, outF, n);

    for (int i = 0; i < n; i++) {
        float x = outF[i];

        if (x > 1.0f) x = 1.0f;
        if (x < -1.0f) x = -1.0f;

        out[i] = (int16_t)(x * 32767.0f);
    }
}

float mapSliderToDB(uint16_t slider) {
    // slider = 0‥4096
    // output = -12 dB to +12 dB
    return (((float)slider / 4096.0f) * 24.0f) - 12.0f;
}

float mapKnobtoFreq(uint16_t knob) {
    float Fc_min = 20.0f;
    float Fc_max = 3000.0f;

    float x = (float)knob / 4095.0f;  // normalize
    if (x < 0.0f) x = 0.0f;
    if (x > 1.0f) x = 1.0f;

    return Fc_min * powf(Fc_max / Fc_min, x);
}

// END EQ Stuff

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_SPI1_Init();
    MX_TIM1_Init();
    MX_LPUART1_UART_Init();
    MX_DAC1_Init();
    MX_TIM2_Init();
    MX_ADC1_Init();
    MX_FATFS_Init();
    MX_SPI2_Init();
    MX_TIM4_Init();
    /* USER CODE BEGIN 2 */

    // EQ STuff
    tft_init();
    tft_fill_rect(0, 0, 320, 480, 0xAAAA);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);  // LED ON

    // Start the ADC reads for the potentiometers and sliders
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);

    // Receieve the header from UART
    uart_start_header_rx();

    /* Start TIM2 (48 kHz trigger) */
    HAL_TIM_Base_Start(&htim2);

    // LEDS
    ws2812_init(&ws2812, &htim4, TIM_CHANNEL_3, Number_LEDs);
    eq_graph_set(&ws2812, EQ_GRAPH_FFT);

    /* Init DAC circular buffer to mid-scale (silence) */
    for (uint32_t i = 0; i < DAC_BUF_SAMPLES; ++i) {
        dac_buf[i] = 2048;  // mid-scale for 12-bit
    }

    /* Do NOT start DAC DMA here.
     * We'll start it after we've "primed" the buffer with at least
     * two good audio packets of a new stream.
     */
    dac_running = 0;
    audio_prime_count = 0;
    audio_stream_active = 0;

    // INIT FIR
    arm_rfft_fast_init_f32(&rfft, FFT_SIZE);
    for (int i = 0; i < FFT_SIZE; i++) {
        window[i] = 0.5f - 0.5f * arm_cos_f32((2 * PI * i) / (FFT_SIZE - 1));
    }
    memset(overlap, 0, sizeof(overlap));

    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */

    while (1) {
        keyPress = read_keypad();
        ADC_ReadAll_Polling();
        MIDI_Check();

        handle_ctrl_packet();
        handle_image_packet();
        handle_audio_packet();
        handle_text_packet();
        handle_button_presses();
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
     */
    if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 1;
    RCC_OscInitStruct.PLL.PLLN = 15;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType =
        RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
        Error_Handler();
    }
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {
    /* USER CODE BEGIN ADC1_Init 0 */

    /* USER CODE END ADC1_Init 0 */

    ADC_ChannelConfTypeDef sConfig = {0};

    /* USER CODE BEGIN ADC1_Init 1 */

    /* USER CODE END ADC1_Init 1 */

    /** Common config
     */
    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    hadc1.Init.LowPowerAutoWait = DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.NbrOfConversion = 4;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
    hadc1.Init.OversamplingMode = DISABLE;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) {
        Error_Handler();
    }

    /** Configure Regular Channel
     */
    sConfig.Channel = ADC_CHANNEL_5;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        Error_Handler();
    }

    /** Configure Regular Channel
     */
    sConfig.Channel = ADC_CHANNEL_6;
    sConfig.Rank = ADC_REGULAR_RANK_2;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        Error_Handler();
    }

    /** Configure Regular Channel
     */
    sConfig.Channel = ADC_CHANNEL_7;
    sConfig.Rank = ADC_REGULAR_RANK_3;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        Error_Handler();
    }

    /** Configure Regular Channel
     */
    sConfig.Channel = ADC_CHANNEL_8;
    sConfig.Rank = ADC_REGULAR_RANK_4;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN ADC1_Init 2 */

    /* USER CODE END ADC1_Init 2 */
}

/**
 * @brief DAC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_DAC1_Init(void) {
    /* USER CODE BEGIN DAC1_Init 0 */

    /* USER CODE END DAC1_Init 0 */

    DAC_ChannelConfTypeDef sConfig = {0};

    /* USER CODE BEGIN DAC1_Init 1 */

    /* USER CODE END DAC1_Init 1 */

    /** DAC Initialization
     */
    hdac1.Instance = DAC1;
    if (HAL_DAC_Init(&hdac1) != HAL_OK) {
        Error_Handler();
    }

    /** DAC channel OUT1 config
     */
    sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
    sConfig.DAC_Trigger = DAC_TRIGGER_T2_TRGO;
    sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_ABOVE_80MHZ;
    sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
    sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
    sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
    if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN DAC1_Init 2 */

    /* USER CODE END DAC1_Init 2 */
}

/**
 * @brief LPUART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_LPUART1_UART_Init(void) {
    /* USER CODE BEGIN LPUART1_Init 0 */

    /* USER CODE END LPUART1_Init 0 */

    /* USER CODE BEGIN LPUART1_Init 1 */

    /* USER CODE END LPUART1_Init 1 */
    hlpuart1.Instance = LPUART1;
    hlpuart1.Init.BaudRate = 1500000;
    hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
    hlpuart1.Init.StopBits = UART_STOPBITS_1;
    hlpuart1.Init.Parity = UART_PARITY_NONE;
    hlpuart1.Init.Mode = UART_MODE_TX_RX;
    hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    hlpuart1.FifoMode = UART_FIFOMODE_DISABLE;
    if (HAL_UART_Init(&hlpuart1) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN LPUART1_Init 2 */

    /* USER CODE END LPUART1_Init 2 */
}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {
    /* USER CODE BEGIN SPI1_Init 0 */

    /* USER CODE END SPI1_Init 0 */

    /* USER CODE BEGIN SPI1_Init 1 */

    /* USER CODE END SPI1_Init 1 */
    /* SPI1 parameter configuration*/
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 7;
    hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
    hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
    if (HAL_SPI_Init(&hspi1) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN SPI1_Init 2 */

    /* USER CODE END SPI1_Init 2 */
}

/**
 * @brief SPI2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI2_Init(void) {
    /* USER CODE BEGIN SPI2_Init 0 */

    /* USER CODE END SPI2_Init 0 */

    /* USER CODE BEGIN SPI2_Init 1 */

    /* USER CODE END SPI2_Init 1 */
    /* SPI2 parameter configuration*/
    hspi2.Instance = SPI2;
    hspi2.Init.Mode = SPI_MODE_MASTER;
    hspi2.Init.Direction = SPI_DIRECTION_2LINES;
    hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi2.Init.NSS = SPI_NSS_SOFT;
    hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
    hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi2.Init.CRCPolynomial = 7;
    hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
    hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
    if (HAL_SPI_Init(&hspi2) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN SPI2_Init 2 */

    /* USER CODE END SPI2_Init 2 */
}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {
    /* USER CODE BEGIN TIM1_Init 0 */

    /* USER CODE END TIM1_Init 0 */

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    /* USER CODE BEGIN TIM1_Init 1 */

    /* USER CODE END TIM1_Init 1 */
    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 119;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = 49;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM1_Init 2 */

    /* USER CODE END TIM1_Init 2 */
}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {
    /* USER CODE BEGIN TIM2_Init 0 */

    /* USER CODE END TIM2_Init 0 */

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    /* USER CODE BEGIN TIM2_Init 1 */

    /* USER CODE END TIM2_Init 1 */
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 0;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 4999;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM2_Init 2 */

    /* USER CODE END TIM2_Init 2 */
}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void) {
    /* USER CODE BEGIN TIM4_Init 0 */

    /* USER CODE END TIM4_Init 0 */

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    /* USER CODE BEGIN TIM4_Init 1 */

    /* USER CODE END TIM4_Init 1 */
    htim4.Instance = TIM4;
    htim4.Init.Prescaler = 0;
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = LED_CNT;
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim4) != HAL_OK) {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_Init(&htim4) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM4_Init 2 */

    /* USER CODE END TIM4_Init 2 */
    HAL_TIM_MspPostInit(&htim4);
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {
    /* DMA controller clock enable */
    __HAL_RCC_DMAMUX1_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* DMA interrupt init */
    /* DMA1_Channel1_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
    /* DMA1_Channel2_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
    /* DMA1_Channel3_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
    /* DMA1_Channel4_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
    /* DMA1_Channel5_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    /* USER CODE BEGIN MX_GPIO_Init_1 */

    /* USER CODE END MX_GPIO_Init_1 */

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    HAL_PWREx_EnableVddIO2();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOF, Col_4_Pin | Col_3_Pin | Col_2_Pin, GPIO_PIN_SET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6 | GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(Col_1_GPIO_Port, Col_1_Pin, GPIO_PIN_SET);

    /*Configure GPIO pins : PE2 PE3 */
    GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF13_SAI1;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /*Configure GPIO pins : Col_4_Pin Col_3_Pin Col_2_Pin */
    GPIO_InitStruct.Pin = Col_4_Pin | Col_3_Pin | Col_2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    /*Configure GPIO pin : PF7 */
    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF13_SAI1;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    /*Configure GPIO pins : Row_1_Pin Row_2_Pin Row_3_Pin */
    GPIO_InitStruct.Pin = Row_1_Pin | Row_2_Pin | Row_3_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /*Configure GPIO pin : PB0 */
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*Configure GPIO pin : Row_4_Pin */
    GPIO_InitStruct.Pin = Row_4_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(Row_4_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : PAUSE_Pin FORWARD_Pin */
    GPIO_InitStruct.Pin = PAUSE_Pin | FORWARD_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

    /*Configure GPIO pin : REWIND_Pin */
    GPIO_InitStruct.Pin = REWIND_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(REWIND_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : SD_CS_Pin Col_1_Pin */
    GPIO_InitStruct.Pin = SD_CS_Pin | Col_1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*Configure GPIO pin : PB14 */
    GPIO_InitStruct.Pin = GPIO_PIN_14;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF14_TIM15;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*Configure GPIO pins : PD8 PD9 */
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /*Configure GPIO pins : PC6 PC8 PC9 */
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /*Configure GPIO pin : PC7 */
    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /*Configure GPIO pins : PA8 PA10 */
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pin : PA9 */
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pins : PC10 PC11 PC12 */
    GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF12_SDMMC1;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /*Configure GPIO pin : PD0 */
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /*Configure GPIO pin : PD2 */
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF12_SDMMC1;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /*Configure GPIO pins : PD3 PD5 PD6 */
    GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_5 | GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /*Configure GPIO pins : PB3 PB4 PB5 */
    GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*Configure GPIO pins : PB8 PB9 */
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* EXTI interrupt init*/
    HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);

    HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI1_IRQn);

    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

    /* USER CODE BEGIN MX_GPIO_Init_2 */

    /*Configure GPIO pins : PD8 PD9 (USART3 TX/RX) */
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

static void handle_ctrl_packet(void) {
    if (!ctrl_pkt_ready) return;
    if (ctrl_pkt_ready) {
        ctrl_pkt_ready = 0;
        uint8_t cmd = ctrl_buf[0];

        if (cmd == CTRL_CMD_AUDIO_STOP) {
            __disable_irq();
            audio_stream_active = 0;

            // Drop any pending audio packet in-flight
            audio_pkt_ready = 0;
            audio_pkt_len = 0;

            // Release both halves so future streams start clean
            half0_free = 1;
            half1_free = 1;

            // Reset priming state
            audio_prime_count = 0;

            paused = 0;

            __enable_irq();

            // Stop DAC DMA if it was running
            if (dac_running) {
                HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
                dac_running = 0;
            }

            // Fill DAC buffer with silence
            for (uint32_t i = 0; i < DAC_BUF_SAMPLES; ++i) {
                dac_buf[i] = 2048;
            }

            uint8_t ack = UART_ACK_OK;  // 'S'
            HAL_UART_Transmit(&hlpuart1, &ack, 1, 10);

            // Re-arm header RX
            uart_start_header_rx();
        }

        // Skip directly to next loop iteration so we don't process audio
        // in the same pass after a STOP.
        return;
    }
}
static void handle_image_packet(void) {
    if (!image_pkt_ready) return;
    if (image_pkt_ready) {
        uint16_t payload_len;
        uint8_t hdr_bytes[CRC_HEADER_LEN];

        __disable_irq();
        payload_len = image_pkt_len;  // bytes of image payload
        image_pkt_ready = 0;

        // Snapshot the header fields for this packet into hdr_bytes[0..6]
        hdr_bytes[0] = g_uart_rx.header.version;
        hdr_bytes[1] = g_uart_rx.header.data_type;
        hdr_bytes[2] = g_uart_rx.header.flags;
        hdr_bytes[3] = (uint8_t)(g_uart_rx.header.seq >> 8);
        hdr_bytes[4] = (uint8_t)(g_uart_rx.header.seq & 0xFF);
        hdr_bytes[5] = (uint8_t)(g_uart_rx.header.len >> 8);
        hdr_bytes[6] = (uint8_t)(g_uart_rx.header.len & 0xFF);
        __enable_irq();

        if (payload_len <= sizeof(image_pkt_buf)) {
            uint16_t recv_crc = ((uint16_t)image_pkt_buf[payload_len] << 8) |
                                (uint16_t)image_pkt_buf[payload_len + 1];

            // Use the *snapshotted* header bytes, not prefix_buf
            uint16_t calc_crc = crc16_ccitt(hdr_bytes, image_pkt_buf, payload_len);

            if (calc_crc != recv_crc) {
                uint8_t nack = UART_ACK_ERR;
                HAL_UART_Transmit(&hlpuart1, &nack, 1, 10);
            } else {
                parse_and_apply_image_packet(image_pkt_buf, payload_len);

                uint8_t ack = UART_ACK_OK;
                HAL_UART_Transmit(&hlpuart1, &ack, 1, 10);
            }
        } else {
            uint8_t nack = UART_ACK_ERR;
            HAL_UART_Transmit(&hlpuart1, &nack, 1, 10);
        }
    }
}
static void handle_audio_packet(void) {
    if (!audio_pkt_ready) return;
    // 2) Handle audio packets (fill halves of dac_buf, DAC runs continuously)
    // 2) Handle audio packets (convert in-place in dac_buf, DAC runs continuously)
    if (audio_pkt_ready) {
        uint8_t idx;
        uint16_t payload_len;
        uint8_t hdr_bytes[CRC_HEADER_LEN];
        uint8_t send_manual_A = 0;

        // Snapshot shared state quickly
        __disable_irq();
        idx = audio_write_idx;        // 0 or 1
        payload_len = audio_pkt_len;  // bytes of audio payload
        audio_pkt_ready = 0;
        __enable_irq();

        // Header snapshot (same as before)
        hdr_bytes[0] = g_uart_rx.header.version;
        hdr_bytes[1] = g_uart_rx.header.data_type;
        hdr_bytes[2] = g_uart_rx.header.flags;
        hdr_bytes[3] = (uint8_t)(g_uart_rx.header.seq >> 8);
        hdr_bytes[4] = (uint8_t)(g_uart_rx.header.seq & 0xFF);
        hdr_bytes[5] = (uint8_t)(g_uart_rx.header.len >> 8);
        hdr_bytes[6] = (uint8_t)(g_uart_rx.header.len & 0xFF);

        // Sanity-check length
        if (payload_len > AUD_MAX_PAYLOAD_BYTES) {
            if (idx == 0)
                half0_free = 1;
            else
                half1_free = 1;
            uint8_t nack = UART_ACK_ERR;
            HAL_UART_Transmit(&hlpuart1, &nack, 1, 10);
            uart_start_header_rx();
            return;
        }

        // CRC check (payload already in the selected half)
        uint8_t* payload_bytes = (uint8_t*)((idx == 0) ? &dac_buf[0] : &dac_buf[DAC_HALF_SAMPLES]);
        uint16_t recv_crc = ((uint16_t)audio_crc_buf[0] << 8) | (uint16_t)audio_crc_buf[1];
        uint16_t calc_crc = crc16_ccitt(hdr_bytes, payload_bytes, payload_len);

        if (calc_crc != recv_crc) {
            if (idx == 0)
                half0_free = 1;
            else
                half1_free = 1;
            uint8_t nack = UART_ACK_ERR;
            HAL_UART_Transmit(&hlpuart1, &nack, 1, 10);
            uart_start_header_rx();
            return;
        }

        // Good packet: process into a temporary output buffer to avoid races.
        uint16_t num_samples = payload_len / 2;
        if (num_samples > DAC_HALF_SAMPLES) num_samples = DAC_HALF_SAMPLES;

        int16_t* src = (int16_t*)payload_bytes;      // raw PCM from UART (signed 16-bit)
        static uint16_t proc_buf[DAC_HALF_SAMPLES];  // processed unsigned-12-bit values (0..4095)
        // Keep a local copy of MIDI state so you only advance MIDI_index while producing real
        // output samples:
        uint32_t local_midi_index = MIDI_index;
        uint8_t local_midi_playing = MIDI_playing;

        // 1) Run FFT / filter to fill filter_buf (q15_t / int16_t out)
        processEQ(src, filter_buf,
                  num_samples);  // filter_buf[] filled with int16_t in -32768..32767
        EQ_LED_Tick(&rfft);

        // 2) Produce mixed samples into proc_buf (12-bit unsigned, right-aligned)
        for (uint16_t i = 0; i < num_samples; ++i) {
            int32_t a = filter_buf[i];  // -32768 .. 32767

            // Convert 16-bit -> 12-bit signed
            int32_t temp = a >> 4;  // -2048 .. 2047
            if (temp < -2048) temp = -2048;
            if (temp > 2047) temp = 2047;

            // MIDI sample (12-bit signed). Only advance local_midi_index when generating real
            // samples.
            int32_t m = 0;
            if (local_midi_playing && (local_midi_index < MIDI_lengths[sound_effect_index])) {
                m = MIDI_effects[sound_effect_index][local_midi_index++];
            } else if (local_midi_playing) {
                // MIDI finished during this half -> stop further MIDI playback
                local_midi_playing = 0;
                local_midi_index = 0;
                m = 0;
            } else {
                m = 0;
            }

            temp = temp * 3 / 4;  // new chat
            m = m * 2 / 3;        // new chat

            int32_t mix = temp + m;  // (m / 2);

            // Clip to signed 12-bit range
            if (mix < -2048) mix = -2048;
            if (mix > 2047) mix = 2047;

            // Convert to unsigned 12-bit for DAC (0..4095)
            proc_buf[i] = (uint16_t)(mix + 2048);
        }

        // 3) Padding: do NOT advance MIDI_index for padding samples.
        //    We'll either silence pad or hold last MIDI sample (prefer silence to keep timing
        //    deterministic).
        for (uint16_t i = num_samples; i < DAC_HALF_SAMPLES; ++i) {
            // choose either silence (0) or last_midi_sample for "hold" behavior:
            // int32_t m = last_midi_sample; // hold last MIDI
            int32_t m = 0;  // silence during padding

            // Clip just in case
            if (m < -2048) m = -2048;
            if (m > 2047) m = 2047;

            proc_buf[i] = (uint16_t)(m + 2048);
        }

        // 4) Atomically copy proc_buf -> chosen half of dac_buf while short IRQ disabled.
        // This makes sure DMA never sees a partially-updated half.
        __disable_irq();
        memcpy(payload_bytes, proc_buf, DAC_HALF_SAMPLES * sizeof(uint16_t));
        // Commit MIDI_index/MIDI_playing back to globals only after copy so their state corresponds
        // to written data.
        MIDI_index = local_midi_index;
        MIDI_playing = local_midi_playing;
        __enable_irq();

        // ---- Priming / DAC-start logic (unchanged, operate after buffer is filled) ----
        if (!dac_running) {
            audio_prime_count++;

            if (audio_prime_count == 1) {
                send_manual_A = 1;  // request next packet immediately
            } else if (audio_prime_count == 2) {
                if (HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*)dac_buf, DAC_BUF_SAMPLES,
                                      DAC_ALIGN_12B_R) != HAL_OK) {
                    Error_Handler();
                }
                dac_running = 1;
                audio_stream_active = 1;
            }
        } else {
            // Already running: keep stream_active unless LAST
            if (!(hdr_bytes[2] & PKT_FLAG_LAST)) {
                audio_stream_active = 1;
            }
        }

        // If this packet is marked LAST, stop requesting future audio.
        if (hdr_bytes[2] & PKT_FLAG_LAST) {
            audio_stream_active = 0;
        }

        // ACK the PC that packet was accepted
        {
            uint8_t ack = UART_ACK_OK;
            HAL_UART_Transmit(&hlpuart1, &ack, 1, 10);
        }

        // If priming and we just finished the first packet, ask the PC for next chunk now
        if (send_manual_A) {
            uint8_t req = UART_ACK_NEXT_AUDIO_CHUNK;  // 'A'
            HAL_UART_Transmit(&hlpuart1, &req, 1, 10);
        }

        // Re-arm UART header receive for next frame
        uart_start_header_rx();
    }
}

static void handle_text_packet(void) {
    if (!text_pkt_ready) return;

    uint16_t payload_len;
    uint8_t hdr_bytes[CRC_HEADER_LEN];

    __disable_irq();
    payload_len = text_pkt_len;  // bytes of payload
    text_pkt_ready = 0;
    __enable_irq();

    // Snapshot header fields
    hdr_bytes[0] = g_uart_rx.header.version;
    hdr_bytes[1] = g_uart_rx.header.data_type;
    hdr_bytes[2] = g_uart_rx.header.flags;
    hdr_bytes[3] = (uint8_t)(g_uart_rx.header.seq >> 8);
    hdr_bytes[4] = (uint8_t)(g_uart_rx.header.seq & 0xFF);
    hdr_bytes[5] = (uint8_t)(g_uart_rx.header.len >> 8);
    hdr_bytes[6] = (uint8_t)(g_uart_rx.header.len & 0xFF);

    if (payload_len < 1 || payload_len + PKT_CRC_SIZE > sizeof(text_buf)) {
        uint8_t nack = UART_ACK_ERR;
        HAL_UART_Transmit(&hlpuart1, &nack, 1, 10);
        return;
    }

    uint16_t recv_crc =
        ((uint16_t)text_buf[payload_len] << 8) | (uint16_t)text_buf[payload_len + 1];

    uint16_t calc_crc = crc16_ccitt(hdr_bytes, text_buf, payload_len);

    if (calc_crc != recv_crc) {
        uint8_t nack = UART_ACK_ERR;
        HAL_UART_Transmit(&hlpuart1, &nack, 1, 10);
        return;
    }

    // CRC OK → parse payload
    uint8_t field_id = text_buf[0];
    uint16_t str_len = payload_len - 1;  // exclude field_id

    const uint8_t* src = &text_buf[1];

    char* dest = NULL;
    uint16_t dest_n = 0;

    switch (field_id) {
        case TEXT_FIELD_TITLE:
            dest = current_title;
            dest_n = sizeof(current_title);
            break;
        case TEXT_FIELD_ARTIST:
            dest = current_artist;
            dest_n = sizeof(current_artist);
            break;
        case TEXT_FIELD_ALBUM:
            dest = current_album;
            dest_n = sizeof(current_album);
            break;
        case TEXT_FIELD_OTHER:
        default:
            dest = current_other;
            dest_n = sizeof(current_other);
            break;
    }

    if (dest && dest_n > 0) {
        if (str_len >= dest_n) str_len = dest_n - 1;
        memcpy(dest, src, str_len);
        dest[str_len] = '\0';
    }

    // Clear full 160px footer: y = 320..479
    tft_fill_rect(0, 320, 320, 160, 0x0000);  // black background

    // Artist & album: single-line, still big
    tft_draw_text(5, 332, current_artist, 0xFFE0);  // yellow
    tft_draw_text(5, 364, current_album, 0x07E0);   // green

    tft_draw_text_wrap(5, 396, current_title, 0xFFFF, 310);  // 320 - 2*5
    // Optional 4th line (e.g., current_other):
    // tft_draw_text(5, 428, current_other, 0xF81F); // magenta

    // ACK back to PC
    uint8_t ack = UART_ACK_OK;
    HAL_UART_Transmit(&hlpuart1, &ack, 1, 10);
}

static void handle_button_presses(void) {
    if (forward_flag == 0 && rewind_flag == 0 && pause_button_flag == 0) {
        return;
    }

    uint8_t btn[2];
    btn[0] = BUTTON_FRAME_PREFIX;
    btn_event_t button_pressed = 0;

    if (forward_flag == 1) {
        button_pressed = BTN_EVENT_NEXT;
        forward_flag = 0;
    } else if (rewind_flag == 1) {
        button_pressed = BTN_EVENT_PREV;
        rewind_flag = 0;
    } else if (pause_button_flag == 1) {
        button_pressed = BTN_EVENT_PLAY_PAUSE;
        pause_button_flag = 0;
    }
    btn[1] = (uint8_t)button_pressed;

    HAL_UART_Transmit(&hlpuart1, btn, sizeof(btn), 10);
}

// Static Function declarations

static void ADC_ReadAll_Polling() {
    HAL_ADC_Start(&hadc1);
    for (int i = 0; i < 4; i++) {
        if (HAL_ADC_PollForConversion(&hadc1, 5) == HAL_OK) {
            adc_buf[i] = HAL_ADC_GetValue(&hadc1);
        }
    }
    float low_dB = mapSliderToDB(adc_buf[0]);
    float mid_dB = mapSliderToDB(adc_buf[1]);
    float high_dB = mapSliderToDB(adc_buf[2]);
    uint8_t windowFilterStatus = (adc_buf[3] < 4000);
    float window_freq = mapKnobtoFreq(adc_buf[3]);

    EQ_setGains(low_dB, mid_dB, high_dB, windowFilterStatus, window_freq);
}

static void processFFT(const q15_t* in, q15_t* out, uint16_t num_samples) {
    /*
     * This implementation keeps FFT continuity across packet boundaries by
     * maintaining the last FFT_INCREMENT samples from the previous call.
     * For each incoming packet we form a combined buffer = prev_samples + current_samples
     * and run overlapping FFT blocks across that stream. We only write the
     * portion of the processed stream that corresponds to the current packet.
     */

    static float prev_samples[FFT_INCREMENT];  // last hop of previous packet (float)
    static uint8_t prev_inited = 0;
    if (!prev_inited) {
        for (int i = 0; i < FFT_INCREMENT; ++i) prev_samples[i] = 0.0f;
        prev_inited = 1;
    }

    // Convert incoming input to float and build combined buffer: prev + current
    // combined_len = FFT_INCREMENT + num_samples
    uint32_t combined_len = (uint32_t)FFT_INCREMENT + (uint32_t)num_samples;
    // Use a local stack buffer sized to FFT_SIZE + FFT_INCREMENT (safe for our sizes)
    float combined[FFT_SIZE + FFT_INCREMENT];

    // copy prev
    for (int i = 0; i < FFT_INCREMENT; ++i) combined[i] = prev_samples[i];

    // copy current (convert q15 -> float)
    for (uint32_t i = 0; i < (uint32_t)num_samples; ++i) {
        combined[FFT_INCREMENT + i] = (float)in[i] / 32768.0f;
    }

    // Process windows across the combined stream
    uint32_t pos = 0;
    while ((pos + FFT_SIZE) <= combined_len) {
        // Analysis window
        for (int i = 0; i < FFT_SIZE; ++i) {
            fft_in[i] = combined[pos + i] * window[i];
        }

        arm_rfft_fast_f32(&rfft, fft_in, fft_out, 0);

        // EQ gains (same as before)
        int low_end = 60;
        int mid_end = 300;
        int max_bin = (FFT_SIZE / 2);

        uint32_t d_low = adc_buf[0];
        if (d_low < 15) d_low = 0;
        if (d_low > 4000) d_low = 4000;
        uint32_t d_mid = adc_buf[1];
        if (d_mid < 15) d_mid = 0;
        if (d_mid > 4000) d_mid = 4000;
        uint32_t d_high = adc_buf[2];
        if (d_high < 15) d_high = 0;
        if (d_high > 4000) d_high = 4000;

        float32_t g_low = ((float)d_low) / 2000.0f;
        float32_t g_mid = ((float)d_mid) / 2000.0f;
        float32_t g_high = ((float)d_high) / 2000.0f;

        for (int b = 0; b < max_bin; ++b) {
            float32_t weight = 1.0f;
            if (b < low_end) {
                float32_t t = (float)b / (float)low_end;
                weight = g_low + (1.0f - g_low) * t;
            } else if (b < mid_end) {
                float32_t t = (float)(b - low_end) / (float)(mid_end - low_end);
                float32_t envelope = (t <= 0.5f) ? (t * 2.0f) : (2.0f * (1.0f - t));
                weight = 1.0f + (g_mid - 1.0f) * envelope;
            } else {
                float32_t t = (float)(b - mid_end) / (float)(max_bin - mid_end);
                weight = 1.0f + (g_high - 1.0f) * t;
            }

            // CMSIS rfft_fast layout:
            // index 0 = Re(0)
            // index 1 = Re(N/2) (Nyquist)
            // for k=1..N/2-1: Re(k)=fft_out[2*k], Im(k)=fft_out[2*k+1]
            if (b == 0) {
                // Scale DC (fft_out[0]) and Nyquist (fft_out[1]) by low-weight
                fft_out[0] *= weight;
                fft_out[1] *= weight;
            } else {
                fft_out[2 * b] *= weight;      // Re(k)
                fft_out[2 * b + 1] *= weight;  // Im(k)
            }
        }

        arm_rfft_fast_f32(&rfft, fft_out, fft_in, 1);

        // Synthesis window + overlap-add. We want to produce output samples corresponding to
        // the range [FFT_INCREMENT .. FFT_INCREMENT + num_samples - 1] of the combined stream.
        for (int i = 0; i < FFT_INCREMENT; ++i) {
            float32_t windowed = fft_in[i] * window[i];
            float32_t x = windowed + overlap[i];

            // gentle soft clip
            if (x > 1.0f) x = 1.0f - (x - 1.0f) * 0.3f;
            if (x < -1.0f) x = -1.0f + (-x - 1.0f) * 0.3f;

            uint32_t combined_idx = pos + i;
            // Only write samples that fall into the current packet region
            if (combined_idx >= (uint32_t)FFT_INCREMENT &&
                combined_idx < (uint32_t)FFT_INCREMENT + (uint32_t)num_samples) {
                uint32_t out_idx = combined_idx - (uint32_t)FFT_INCREMENT;
                out[out_idx] = (int16_t)(x * 32767.0f);
            }
        }

        // save second half (synthesis-windowed) into overlap for next block
        for (int i = 0; i < FFT_INCREMENT; ++i) {
            overlap[i] = fft_in[i + FFT_INCREMENT] * window[i + FFT_INCREMENT];
        }

        pos += FFT_INCREMENT;
    }

    // Save last FFT_INCREMENT samples from combined for next call's prev_samples
    // If combined_len < FFT_INCREMENT this still copies valid values or zeros.
    if (combined_len >= (uint32_t)FFT_INCREMENT) {
        uint32_t start = combined_len - (uint32_t)FFT_INCREMENT;
        for (int i = 0; i < FFT_INCREMENT; ++i) prev_samples[i] = combined[start + i];
    } else {
        // uncommon: pad start of prev_samples with zeros then copy combined
        int pad = FFT_INCREMENT - (int)combined_len;
        for (int i = 0; i < pad; ++i) prev_samples[i] = 0.0f;
        for (uint32_t i = 0; i < combined_len; ++i) prev_samples[pad + i] = combined[i];
    }
}

static char read_keypad(void) {
    static uint32_t last_key = 0;
    uint32_t now = HAL_GetTick();

    if (now - last_key < 80) {
        keyPress = ' ';
        return 0;  // debounce
    }
    last_key = now;

    // Row pins
    GPIO_TypeDef* ROW_PORT[4] = {Row_1_GPIO_Port, Row_2_GPIO_Port, Row_3_GPIO_Port,
                                 Row_4_GPIO_Port};
    uint16_t ROW_PIN[4] = {Row_1_Pin, Row_2_Pin, Row_3_Pin, Row_4_Pin};

    // Col pins
    GPIO_TypeDef* COL_PORT[4] = {Col_1_GPIO_Port, Col_2_GPIO_Port, Col_3_GPIO_Port,
                                 Col_4_GPIO_Port};
    uint16_t COL_PIN[4] = {Col_1_Pin, Col_2_Pin, Col_3_Pin, Col_4_Pin};

    // Key map (customize to your keypad)
    char KEY_MAP[4][4] = {
        {'1', '2', '3', 'A'}, {'4', '5', '6', 'B'}, {'7', '8', '9', 'C'}, {'*', '0', '#', 'D'}};

    for (int c = 0; c < 4; c++) {
        // Drive all columns HIGH
        for (int i = 0; i < 4; i++) HAL_GPIO_WritePin(COL_PORT[i], COL_PIN[i], GPIO_PIN_SET);

        // Drive only this column LOW
        HAL_GPIO_WritePin(COL_PORT[c], COL_PIN[c], GPIO_PIN_RESET);

        HAL_Delay(1);  // settle time

        // Read each row
        for (int r = 0; r < 4; r++) {
            if (HAL_GPIO_ReadPin(ROW_PORT[r], ROW_PIN[r]) == GPIO_PIN_RESET) {
                HAL_Delay(20);  // debounce
                if (HAL_GPIO_ReadPin(ROW_PORT[r], ROW_PIN[r]) == GPIO_PIN_RESET)
                    return KEY_MAP[r][c];
            }
        }
    }

    return 0;
}

static void MIDI_Check() {
    switch (keyPress) {
        case '1':
            sound_effect_index = 0;
            MIDI_index = 0;
            MIDI_playing = 1;
            break;
        case '2':
            sound_effect_index = 1;
            MIDI_index = 0;
            MIDI_playing = 1;
            break;
        case '3':
            sound_effect_index = 2;
            MIDI_index = 0;
            MIDI_playing = 1;
            break;
        case 'A':
            sound_effect_index = 3;
            MIDI_index = 0;
            MIDI_playing = 1;
            break;
        case '4':
            sound_effect_index = 4;
            MIDI_index = 0;
            MIDI_playing = 1;
            break;
        case '5':
            sound_effect_index = 5;
            MIDI_index = 0;
            MIDI_playing = 1;
            break;
        case '6':
            sound_effect_index = 6;
            MIDI_index = 0;
            MIDI_playing = 1;
            break;
        case 'B':
            sound_effect_index = 7;
            MIDI_index = 0;
            MIDI_playing = 1;
            break;
        case '7':
            break;
        case '8':
            break;
        case '9':
            break;
        case 'C':
            break;
        case '*':
            break;
        case '0':
            break;
        case '#':
            break;
        case 'D':
            break;
        default:
            break;
    }
}

static void protocol_soft_reset(void) {
    __disable_irq();
    HAL_UART_AbortReceive(&hlpuart1);
    g_uart_rx.state = RX_STATE_WAIT_PREFIX;
    g_uart_rx.payload_len = 0;
    g_uart_rx.payload_dst = NULL;

    image_pkt_ready = 0;
    image_pkt_len = 0;
    audio_pkt_ready = 0;
    audio_pkt_len = 0;

    half0_free = 1;
    half1_free = 1;
    audio_stream_active = 0;
    audio_prime_count = 0;  // NEW: reset priming
    __enable_irq();

    uart_start_header_rx();
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    static uint32_t last_btn = 0;
    uint32_t now = HAL_GetTick();

    if (now - last_btn < 30) return;  // debounce
    last_btn = now;

    if (GPIO_Pin == REWIND_Pin)  // Rewind button
        rewind_flag = 1;

    else if (GPIO_Pin == PAUSE_Pin) {  // Pause/unpause toggle
        pause_button_flag = 1;         // still send 0xF0 + 'P' to PC in handle_button_presses()

        if (!paused) {
            // Enter PAUSED state: stop sending 'A' and stop DAC
            paused = 1;

            // Stop requesting more audio from PC
            audio_stream_active = 0;

            // Stop DAC DMA if it is running
            if (dac_running) {
                HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
                dac_running = 0;
            }

            // (Optional) zero out the buffer to guarantee silence if
            // something accidentally restarts DAC:
            // for (uint32_t i = 0; i < DAC_BUF_SAMPLES; ++i) {
            //     dac_buf[i] = 2048;
            // }

        } else {
            // Exit PAUSED state: resume DAC and audio requests
            paused = 0;

            // Only restart if we had already primed the buffer at least once
            // (stream in progress, not idle)
            if (!dac_running && (audio_prime_count >= 2)) {
                if (HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*)dac_buf, DAC_BUF_SAMPLES,
                                      DAC_ALIGN_12B_R) == HAL_OK) {
                    dac_running = 1;
                    audio_stream_active = 1;  // let callbacks send 'A' again
                }
            }
        }
    } else if (GPIO_Pin == FORWARD_Pin)  // Forward button
        forward_flag = 1;
}

static void uart_start_header_rx(void) {
    g_uart_rx.state = RX_STATE_WAIT_PREFIX;
    HAL_UART_Receive_DMA(&hlpuart1, g_uart_rx.prefix_buf,
                         PKT_PREFIX_SIZE);  // 9 bytes: sync + header
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
    if (huart->Instance != LPUART1) return;

    if (g_uart_rx.state == RX_STATE_WAIT_PREFIX) {
        // We just received sync + header (9 bytes)
        uint8_t* buf = g_uart_rx.prefix_buf;

        // 1) Check sync bytes
        if (buf[0] != 0xA5 || buf[1] != 0x5A) {
            uart_start_header_rx();
            return;
        }

        // 2) Extract payload length
        uint16_t payload_len = ((uint16_t)buf[7] << 8) | buf[8];

        // 3) Parse header fields
        g_uart_rx.header.version = buf[2];
        g_uart_rx.header.data_type = buf[3];
        g_uart_rx.header.flags = buf[4];
        g_uart_rx.header.seq = ((uint16_t)buf[5] << 8) | buf[6];
        g_uart_rx.header.len = payload_len;
        g_uart_rx.payload_len = payload_len;

        // 4) Choose destination buffer based on packet type
        switch (g_uart_rx.header.data_type) {
            case PKT_DATA_IMAGE:
                if (payload_len + PKT_CRC_SIZE > sizeof(image_pkt_buf)) {
                    uart_start_header_rx();
                    return;
                }
                g_uart_rx.payload_dst = image_pkt_buf;
                break;

            case PKT_DATA_AUDIO:
                // Sanity-check payload length in BYTES (just payload, *not* CRC)
                if (payload_len > AUD_MAX_PAYLOAD_BYTES) {
                    uart_start_header_rx();
                    return;
                }

                // Choose a free half of dac_buf as DMA destination for the *payload*
                if (half0_free) {
                    g_uart_rx.payload_dst = audio_pkt_buf0;  // first half of dac_buf
                    audio_write_idx = 0;
                    half0_free = 0;  // now reserved
                } else if (half1_free) {
                    g_uart_rx.payload_dst = audio_pkt_buf1;  // second half of dac_buf
                    audio_write_idx = 1;
                    half1_free = 0;  // now reserved
                } else {
                    // No free half; can't accept this packet safely
                    uart_start_header_rx();
                    return;
                }
                break;

            case PKT_DATA_CMD:
                if (payload_len > sizeof(ctrl_buf)) {
                    uart_start_header_rx();
                    return;
                }
                g_uart_rx.payload_dst = ctrl_buf;
                break;

            case PKT_DATA_TEXT:
                if (payload_len + PKT_CRC_SIZE > sizeof(text_buf)) {
                    uart_start_header_rx();
                    return;
                }
                g_uart_rx.payload_dst = text_buf;
                break;

            default:
                // Unknown type -> ignore this frame
                uart_start_header_rx();
                return;
        }

        // 5) Set state and start DMA for payload + CRC
        g_uart_rx.state = RX_STATE_WAIT_PAYLOAD;

        if (g_uart_rx.header.data_type == PKT_DATA_IMAGE) {
            HAL_UART_Receive_DMA(&hlpuart1, g_uart_rx.payload_dst, payload_len + PKT_CRC_SIZE);
        } else if (g_uart_rx.header.data_type == PKT_DATA_AUDIO) {
            HAL_UART_Receive_DMA(&hlpuart1, g_uart_rx.payload_dst, payload_len);
        } else if (g_uart_rx.header.data_type == PKT_DATA_CMD) {
            HAL_UART_Receive_DMA(&hlpuart1, g_uart_rx.payload_dst, payload_len + PKT_CRC_SIZE);
        } else if (g_uart_rx.header.data_type == PKT_DATA_TEXT) {
            // TEXT behaves like IMAGE/CMD: payload + CRC in one shot
            HAL_UART_Receive_DMA(&hlpuart1, g_uart_rx.payload_dst, payload_len + PKT_CRC_SIZE);
        }

        // 6) Tell PC "header OK, I'm ready for payload"
        uint8_t ack_hdr = UART_ACK_HEADER;  // 'H'
        HAL_UART_Transmit(&hlpuart1, &ack_hdr, 1, 10);
    } else if (g_uart_rx.state == RX_STATE_WAIT_PAYLOAD) {
        if (g_uart_rx.header.data_type == PKT_DATA_IMAGE) {
            // IMAGE: we already DMA'd payload+CRC into image_pkt_buf
            image_pkt_len = g_uart_rx.payload_len;  // payload length (no CRC)
            image_pkt_ready = 1;

            // Back to waiting for next header
            uart_start_header_rx();
        } else if (g_uart_rx.header.data_type == PKT_DATA_AUDIO) {
            // AUDIO: we have JUST the payload in dac_buf now.
            // Next, we need to grab the 2 CRC bytes.

            g_uart_rx.state = RX_STATE_WAIT_CRC;

            // Start a tiny 2-byte DMA into audio_crc_buf
            HAL_UART_Receive_DMA(&hlpuart1, audio_crc_buf, PKT_CRC_SIZE);
            // Do NOT call uart_start_header_rx() yet; we still need CRC.
        } else if (g_uart_rx.header.data_type == PKT_DATA_CMD) {
            // CTRL: payload + CRC already in ctrl_buf
            ctrl_pkt_len = g_uart_rx.payload_len;  // payload length (no CRC)
            ctrl_pkt_ready = 1;

            // Go back to waiting for next header
            uart_start_header_rx();
        } else if (g_uart_rx.header.data_type == PKT_DATA_TEXT) {
            text_pkt_len = g_uart_rx.payload_len;  // payload bytes (no CRC)
            text_pkt_ready = 1;
            uart_start_header_rx();
        }

    } else if (g_uart_rx.state == RX_STATE_WAIT_CRC) {
        // We just finished receiving the 2 CRC bytes for an audio packet
        if (g_uart_rx.header.data_type == PKT_DATA_AUDIO) {
            audio_pkt_len = g_uart_rx.payload_len;  // just payload length
            audio_pkt_ready = 1;
        }

        // Do NOT start the next header RX here.
        // Just mark that we're logically back in WAIT_PREFIX;
        // the main loop will call uart_start_header_rx() once it
        // has validated this packet and sent 'S' or 'E'.
        g_uart_rx.state = RX_STATE_WAIT_PREFIX;
    }
}

void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef* hdac) {
    if (hdac->Instance != DAC1) return;

    half0_free = 1;

    if (audio_stream_active) {
        uint8_t req = UART_ACK_NEXT_AUDIO_CHUNK;  // 'A'
        HAL_UART_Transmit(&hlpuart1, &req, 1, 10);
    }
}

void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef* hdac) {
    if (hdac->Instance != DAC1) return;

    half1_free = 1;

    if (audio_stream_active) {
        uint8_t req = UART_ACK_NEXT_AUDIO_CHUNK;  // 'A'
        HAL_UART_Transmit(&hlpuart1, &req, 1, 10);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart) {
    if (huart->Instance == LPUART1) {
        // Log something if you have a debug LED/UART
        protocol_soft_reset();
    }
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1) {
    }
    /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line) {
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
