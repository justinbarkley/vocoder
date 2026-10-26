// #include <cstdint>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"

#include "driver/i2s_common.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"

#include "driver/i2s_std.h"
#include "esp_log.h"
#include "portmacro.h"

#include <math.h>

static const char *TAG = "audio";

#define SAMPLE_RATE     64000

#define GPIO_BCLK       40
#define GPIO_WS         41

#define GPIO_DIN        39
#define GPIO_DOUT       11

#define ENCODER_LEFT 21
#define ENCODER_RIGHT 48
#define ENCODER_PUSH 47

#define BUTTON_PUSH 13


#define D0      5
#define D1      4
#define D2      16
#define D3      15
#define SD_CLK  6
#define SD_CMD  7


static i2s_chan_handle_t tx_chan;
static i2s_chan_handle_t rx_chan;




void i2s_setup() {
    esp_err_t ret;

    // full duplex
    i2s_chan_config_t chan_cfg =
        I2S_CHANNEL_DEFAULT_CONFIG(
            I2S_NUM_AUTO,
            I2S_ROLE_MASTER);

    ret = i2s_new_channel(
        &chan_cfg,
        &tx_chan,
        &rx_chan);

    ESP_ERROR_CHECK(ret);

    // Standard Philips I2S
    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),

        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
            I2S_DATA_BIT_WIDTH_32BIT,
            I2S_SLOT_MODE_STEREO),

        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = GPIO_BCLK,
            .ws   = GPIO_WS,
            .dout = GPIO_DOUT,
            .din  = GPIO_DIN,

            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv   = false,
            },
        },
    };

    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_chan, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_chan, &std_cfg));

    ESP_ERROR_CHECK(i2s_channel_enable(tx_chan));
    ESP_ERROR_CHECK(i2s_channel_enable(rx_chan));

    ESP_LOGI(TAG, "Audio started");
}

QueueHandle_t audio_input_queue, audio_processing_queue, audio_output_queue;

#define BUFFER_AMOUNT 4
#define BUFFER_SIZE 512

int32_t audio_pool[BUFFER_AMOUNT][BUFFER_SIZE];

void create_audio_queues() {
    audio_input_queue = xQueueCreate(BUFFER_AMOUNT, sizeof(int32_t *));
    audio_processing_queue = xQueueCreate(BUFFER_AMOUNT, sizeof(int32_t *));
    audio_output_queue = xQueueCreate(BUFFER_AMOUNT, sizeof(int32_t *));

    for (int i = 0; i < BUFFER_AMOUNT; i++) {
        int32_t *ptr = audio_pool[i];
        xQueueSend(audio_input_queue, &ptr, portMAX_DELAY);
    }

}


void audio_input(void *parameters) {
    int32_t *buffer;

    while(1) {
        size_t bytes_read = 0;
        if (xQueueReceive(audio_input_queue, &buffer, portMAX_DELAY) == pdPASS) {
            ESP_ERROR_CHECK(
            i2s_channel_read(
                rx_chan,
                buffer,
                BUFFER_SIZE * sizeof(int32_t),
                &bytes_read,
                portMAX_DELAY));
            xQueueSend(audio_processing_queue, &buffer, portMAX_DELAY);
        }
    }
}

volatile float g_peak = 0.0f, g_rms = 0.0f;
volatile uint32_t g_overrun = 0;

static void scope_task(void *arg) {
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(500));
        ESP_LOGI(TAG, "peak=%.3f rms=%.4f glitches=%lu",
                 g_peak, g_rms, (unsigned long)g_overrun);
        g_peak = 0.0f;
    }
}

void audio_processing(void *parameters) {
    int32_t *buffer;
    while(1) {
        if (xQueueReceive(audio_processing_queue, &buffer, portMAX_DELAY) == pdPASS) {
            float peak = 0.0f, sum2 = 0.0f;
            for (int i = 0; i < BUFFER_SIZE; i++) {
                float x = buffer[i] * (1.0f / 2147483648.0f);
                float a = fabsf(x);
                if (a > peak) peak = a;
                sum2 += x * x;
            }
            if (peak > g_peak) g_peak = peak;
            g_rms = sqrtf(sum2 / BUFFER_SIZE);

            if (peak >= 0.999f) g_overrun++;

            xQueueSend(audio_output_queue, &buffer, portMAX_DELAY);
        }
       
    } 
}

void audio_output(void *parameters) {
    int32_t *buffer;

    while(1) {
        size_t bytes_written = 0;
        if (xQueueReceive(audio_output_queue, &buffer, portMAX_DELAY) == pdPASS) {
            ESP_ERROR_CHECK(
                i2s_channel_write(
                    tx_chan, 
                    buffer, 
                    BUFFER_SIZE * sizeof(int32_t), 
                    &bytes_written, 
                portMAX_DELAY));
            xQueueSend(audio_input_queue, &buffer, portMAX_DELAY);
        }
    }
}


QueueHandle_t debug_queue;

void debug_serial(void *parameters){
    const char *buffer;
    while (1) {
        if (xQueueReceive(debug_queue, &buffer, portMAX_DELAY) == pdPASS) {
            ESP_LOGI("debug", ": %c", *buffer);
        }
    }
}

void start_debug() {
    debug_queue = xQueueCreate(5, sizeof(const char *));
    xTaskCreatePinnedToCore(debug_serial, "debug to serial output", 2048, NULL, 2, NULL, 0);
}






void app_main(void) {

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << 38),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    ESP_ERROR_CHECK(gpio_set_level(38, 1));

    i2s_setup();
    create_audio_queues();


    // xTaskCreatePinnedToCore(audio_processing, "test task", 4096, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(audio_input, "input", 4096, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(audio_processing, "processing", 4096, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(audio_output, "output", 4096, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(scope_task, "scope task", 4096, NULL, 1, NULL, 0);

    start_debug();

    while (1){
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}