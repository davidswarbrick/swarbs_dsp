#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <algorithm> //std max/min
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s_std.h"
#include "driver/gpio.h"
#include "esp_check.h"
#include "WM8978.h"
#include "reverb.h"
#define I2S_SAMPLE_RATE     (48000) // I2S sample rate 
// #define BUFFER_SIZE         32      // Borrowed from Faust boilerplate, 2048 works too.
#define BUFFER_SIZE 1024
#define DELAY_MS 20
#define DRY_WET_RATIO 0.5
#define MULT_S32 2147483647 // Max value for int32
#define MULT_S16 32767 // Max value for int16
#define DIV_S16 3.0518509e-5 // 1/MULT+S16
#define clip(sample) std::max(-MULT_S16, std::min(MULT_S16, ((int)(sample * MULT_S16))));



// Sine wave example parameters 
#define WAVE_FREQ_HZ    (200)  // test waveform frequency
#define PI              (3.14159265)
#define SAMPLES_PER_CYCLE (I2S_SAMPLE_RATE/WAVE_FREQ_HZ)

// Initialise I2S
i2s_chan_handle_t tx_handle;
i2s_chan_handle_t rx_handle;

static void init_i2s(void)
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    // Allocate both tx and rx channel at the same time for full-duplex mode.
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &tx_handle, &rx_handle));
    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(I2S_SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = GPIO_NUM_0,
            .bclk = GPIO_NUM_23,
            .ws = GPIO_NUM_25,
            .dout = GPIO_NUM_26,
            .din = GPIO_NUM_27,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };
    std_cfg.clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_256;
    // Initialise the channels
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_handle, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_handle, &std_cfg));
}


static void passthrough(void *args)
{
    uint16_t *buf = (uint16_t *)calloc(1, BUFFER_SIZE);
    if (!buf) {
        printf("No memory for read data buffer");
        abort();
    }
    esp_err_t ret = ESP_OK;
    size_t bytes_read = 0;
    size_t bytes_write = 0;
    printf("Passthrough start");
    while (1) {
        /* Read sample data from ADC */
        ret = i2s_channel_read(rx_handle, buf, BUFFER_SIZE, &bytes_read, 1000);
        if (ret != ESP_OK) {
            printf("i2s read failed");
            abort();
        }
        /* Write sample data to DAC */
        ret = i2s_channel_write(tx_handle, buf, BUFFER_SIZE, &bytes_write, 1000);
        if (ret != ESP_OK) {
            printf("i2s write failed");
            abort();
        }
        if (bytes_read != bytes_write) {
            printf("%d bytes read but only %d bytes are written", bytes_read, bytes_write);
        }
    }
    vTaskDelete(NULL);
}

static void sine_wave(void *args)             // function to generate a test sine wave
{
    float freq = (WAVE_FREQ_HZ);
    unsigned sample_rate = I2S_SAMPLE_RATE;
    size_t buf_size = (SAMPLES_PER_CYCLE*8);   // number of cycles to store in the buffer
    short *samples_sine = new short[buf_size];    // create buffer to store 16bit samples 
    for(int i=0; i<buf_size; ++i) {
        samples_sine[i] = (pow(2,16)/2-1) * sin(PI*(freq/sample_rate)*i);  // 16bit 
        uint value = 0;
        value += samples_sine[i];
        //printf("Samples_sine=%d\n", value);   // comment out to print the samples data, only use one call to printf in the function
    }
    size_t w_bytes = 0;
    while (1){
        i2s_channel_write(tx_handle, samples_sine, buf_size,&w_bytes,1000);
    };
    vTaskDelete(NULL);
}

static void process(float ** inSlot, float ** outSlot, int buffer_size)
{
    // Process samples here, for now just copy.
    for (int i = 0; i< buffer_size; i++){
        outSlot[0][i] = inSlot[0][i];
        outSlot[1][i] = inSlot[1][i];
    }
}

static void delay(void *args)
{
    int delay_samples = (int) (DELAY_MS * (I2S_SAMPLE_RATE/1000));
    float dry_wet = (float) DRY_WET_RATIO;
    // Set up stereo buffers.
    int16_t *samples_data_buf = (int16_t *)calloc(1, 2*BUFFER_SIZE);
    float ** prevInSlot = new float*[2];
    prevInSlot[0] = new float[BUFFER_SIZE];
    prevInSlot[1] = new float[BUFFER_SIZE];
    // Initialise previous inslot to zero
    for (int i = 0; i< BUFFER_SIZE; i++){
        prevInSlot[0][i] = 0.0;
        prevInSlot[1][i] = 0.0;
    }

    float ** inSlot = new float*[2];
    inSlot[0] = new float[BUFFER_SIZE];
    inSlot[1] = new float[BUFFER_SIZE];

    float ** outSlot = new float*[2];
    outSlot[0] = new float[BUFFER_SIZE];
    outSlot[1] = new float[BUFFER_SIZE];
    float ** tempSlot = new float*[2];
    if (!inSlot || !samples_data_buf || !outSlot || !prevInSlot) {
        printf("No memory for data buffers");
        abort();
    }
    esp_err_t ret = ESP_OK;
    size_t bytes_read = 0;
    size_t bytes_write = 0;
    printf("Delay start: %d Samples, Dry/Wet Ratio: %f \n",delay_samples,dry_wet);
    while (1) {
        /* Read sample data from ADC */
        ret = i2s_channel_read(rx_handle, samples_data_buf, BUFFER_SIZE, &bytes_read, 1000);
        if (ret != ESP_OK) {
            printf("i2s read failed");
            abort();
        }
        // Convert signed 16 to float(32).
        for (int i = 0; i< BUFFER_SIZE; i++){
            inSlot[0][i] = (float)samples_data_buf[i*2]*DIV_S16;
            inSlot[1][i] = (float)samples_data_buf[i*2+1]*DIV_S16;

            if (i - delay_samples < 0){
                // Use data from previous buffer
                // e.g. if len(prevOut) = 100, i = 5, delay_samples = 20, want sample 85 
                outSlot[0][i] = prevInSlot[0][BUFFER_SIZE-delay_samples+i];
                outSlot[1][i] = prevInSlot[1][BUFFER_SIZE-delay_samples+i];
            } else {
                outSlot[0][i] = inSlot[0][i-delay_samples];
                outSlot[1][i] = inSlot[1][i-delay_samples];
            }
            // Convert from float back to u16.
            samples_data_buf[i*2] = clip(outSlot[0][i]);
            samples_data_buf[i*2+1] = clip(outSlot[1][i]);
        }
        // store our previous in slot pointer - we'll use it again
        tempSlot = prevInSlot;
        // we're done with the current inslot, make it previous.
        prevInSlot = inSlot;
        // Reuse pointer to previous In Slot for the incoming new data
        inSlot = tempSlot;
        /* Write sample data to DAC */
        ret = i2s_channel_write(tx_handle, samples_data_buf, BUFFER_SIZE, &bytes_write, 1000);
        if (ret != ESP_OK) {
            printf("i2s write failed");
            abort();
        }
        if (bytes_read != bytes_write) {
            printf("%d bytes read but only %d bytes are written", bytes_read, bytes_write);
        }
    }
    vTaskDelete(NULL);
}

static void reverb(void *args)
{
    // Set up stereo buffers.
    int16_t *samples_data_buf = (int16_t *)calloc(1, 2*BUFFER_SIZE);
    float ** inSlot = new float*[2];
    inSlot[0] = new float[BUFFER_SIZE];
    inSlot[1] = new float[BUFFER_SIZE];

    float ** outSlot = new float*[2];
    outSlot[0] = new float[BUFFER_SIZE];
    outSlot[1] = new float[BUFFER_SIZE];
    if (!inSlot || !samples_data_buf || !outSlot) {
        printf("No memory for data buffers");
        abort();
    }
    esp_err_t ret = ESP_OK;
    size_t bytes_read = 0;
    size_t bytes_write = 0;
    printf("Float conversion start");
    while (1) {
        /* Read sample data from ADC */
        ret = i2s_channel_read(rx_handle, samples_data_buf, BUFFER_SIZE, &bytes_read, 1000);
        if (ret != ESP_OK) {
            printf("i2s read failed");
            abort();
        }

        // Convert signed 16 to float(32).
        for (int i = 0; i< BUFFER_SIZE-1; i++){
            inSlot[0][i] = (float)samples_data_buf[i*2]*DIV_S16;
            inSlot[1][i] = (float)samples_data_buf[i*2+1]*DIV_S16;
        }
        
        process(inSlot, outSlot, BUFFER_SIZE);
        // comb_filter(inSlot[0], outSlot[0], BUFFER_SIZE, I2S_SAMPLE_RATE,5,0.7);
        // comb_filter(inSlot[1], outSlot[1], BUFFER_SIZE, I2S_SAMPLE_RATE,20,0.7);
        // Convert from float back to u16.
        for (int i = 0; i< BUFFER_SIZE-1; i++){
            samples_data_buf[i*2] = clip(outSlot[0][i]);
            samples_data_buf[i*2+1] = clip(outSlot[1][i]);
        }

        /* Write sample data to DAC */
        ret = i2s_channel_write(tx_handle, samples_data_buf, BUFFER_SIZE, &bytes_write, 1000);
        if (ret != ESP_OK) {
            printf("i2s write failed");
            abort();
        }
        if (bytes_read != bytes_write) {
            printf("%d bytes read but only %d bytes are written", bytes_read, bytes_write);
        }
    }
    vTaskDelete(NULL);
}




extern "C" {
    void app_main(void);
}

WM8978 wm8978;   // create an instance of WM8978 class for the audio codec 
void app_main() { 
  
    init_i2s();
    ESP_ERROR_CHECK(i2s_channel_enable(tx_handle));
    ESP_ERROR_CHECK(i2s_channel_enable(rx_handle));

    wm8978.init();            // WM8978 codec initialisation
    wm8978.addaCfg(1,1);      // enable the adc and the dac
    wm8978.inputCfg(0,1,0);   // Enable linein
    wm8978.lineinGain(3);
    wm8978.outputCfg(1,0);    // Enable dac out, no bypass.
    wm8978.spkVolSet(0);      // speaker volume not required, set to 0.
    wm8978.hpVolSet(40,40);   // headphone volume
    wm8978.sampleRate(0);     // set sample rate to 48kHz
    wm8978.i2sCfg(2,0);       // I2S format Philips, 16bit

    // xTaskCreate(sine_wave, "sine_wave", 4096, NULL, 5, NULL);
    // xTaskCreate(passthrough, "passthrough", 4096, NULL, 5, NULL);
    // xTaskCreate(reverb, "reverb",65536, NULL, 5, NULL);
    xTaskCreate(delay, "delay",65536, NULL, 5, NULL);
}
