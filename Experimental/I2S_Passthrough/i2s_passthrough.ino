#include <driver/i2s.h>
#include <Arduino.h>

#define I2S_WS    33  // Word Select (LRCLK)
#define I2S_BCK   5  // Bit Clock (SCLK)
#define I2S_DOUT  25  // Serial Data Out (SDOUT)
#define I2S_DIN   26  // Serial Data In (SDIN)
#define I2S_MCLK   0  // Master Clock (MCLK) on GPIO 0

void setup() {
    Serial.begin(115200);

    // I2S configuration
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_RX),  // Master mode, TX and RX
        .sample_rate = 44100,  // Sample rate (44.1kHz)
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,  // 16-bit per sample
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,  // Stereo
        .communication_format = I2S_COMM_FORMAT_I2S_MSB,  // I2S communication format
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,  // Interrupt level
        .dma_buf_count = 8,  // Number of DMA buffers
        .dma_buf_len = 64,  // DMA buffer length
        .use_apll = true,  // Use APLL for accurate MCLK
        .tx_desc_auto_clear = true,  // Auto clear TX descriptor
        .fixed_mclk = 0  // No fixed MCLK frequency
    };

    // Pin configuration (without MCLK)
    i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_BCK,    // Bit Clock pin (SCLK)
        .ws_io_num = I2S_WS,      // Word Select pin (LRCLK)
        .data_out_num = I2S_DOUT, // Data Out pin (SDOUT)
        .data_in_num = I2S_DIN    // Data In pin (SDIN)
    };

    // Install and start I2S driver
    i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_NUM_0, &pin_config);
    i2s_set_clk(I2S_NUM_0, 44100, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_STEREO);

    // Manually configure MCLK on GPIO 0 using APLL (Audio PLL)
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO0_U, FUNC_GPIO0_CLK_OUT1);  // Set GPIO0 as CLK_OUT1
    WRITE_PERI_REG(PIN_CTRL, READ_PERI_REG(PIN_CTRL) & 0xFFFFFFF0);  // Set Clock Out Source

    Serial.println("I2S Passthrough with MCLK on GPIO 0 initialized.");
}

void loop() {
    // Create buffers for I2S read and write
    const int buffer_size = 512;
    char i2s_read_buffer[buffer_size];
    size_t bytes_read, bytes_written;

    // Read data from I2S input (e.g., microphone or ADC)
    i2s_read(I2S_NUM_0, &i2s_read_buffer, buffer_size, &bytes_read, portMAX_DELAY);
    
    // Write data to I2S output (e.g., DAC or speaker)
    i2s_write(I2S_NUM_0, &i2s_read_buffer, bytes_read, &bytes_written, portMAX_DELAY);
}
