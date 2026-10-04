#include <driver/i2s.h>
#include <math.h>

// Pin configuration
#define I2S_WS    33  // Word Select (LRCLK)
#define I2S_BCK   5   // Bit Clock (SCLK)
#define I2S_DOUT  25  // Serial Data Out (SDOUT)
#define I2S_DIN   26  // Serial Data In (SDIN)
#define I2S_MCLK   0  // Master Clock (MCLK) on GPIO 0

// Switch pin configuration
#define SW_1_PIN 19  // Increase filters
#define SW_2_PIN 16  // Decrease filters

// Define the maximum number of filters (biquads)
#define MAX_FILTERS 8

// Debounce settings
unsigned long last_debounce_time_1 = 0;
unsigned long last_debounce_time_2 = 0;
const unsigned long debounce_delay = 250;  // 50 ms debounce delay

// Filter coefficients for each biquad
float a0_1, a1_1, a2_1, b0_1, b1_1, b2_1;  // Coefficients for all biquads (same for simplicity)
float x1[MAX_FILTERS] = {0}, x2[MAX_FILTERS] = {0}, y1_prev[MAX_FILTERS] = {0}, y2_prev[MAX_FILTERS] = {0};  // States for biquads

// Sample rate and filter parameters
const float sample_rate = 44100.0;
const float center_frequency = 700.0;  // Center frequency of 700Hz (Morse code filter)
const float Q = 5.0;  // Quality factor for sharper selectivity around 700Hz

// Current number of active filters
int current_filter_count = 0;

// Function to calculate filter coefficients for one biquad
void calculate_biquad_coefficients(float &a0, float &a1, float &a2, float &b0, float &b1, float &b2) {
    float w0 = 2.0 * M_PI * center_frequency / sample_rate;
    float alpha = sin(w0) / (2.0 * Q);
    float cos_w0 = cos(w0);

    // Coefficients for the IIR bandpass filter (biquad form)
    b0 = alpha;
    b1 = 0;
    b2 = -alpha;
    a0 = 1 + alpha;
    a1 = -2 * cos_w0;
    a2 = 1 - alpha;

    // Normalize coefficients
    b0 /= a0;
    b1 /= a0;
    b2 /= a0;
    a1 /= a0;
    a2 /= a0;
}

// Apply a single biquad filter to a sample
float biquad_filter(float input, int index, float b0, float b1, float b2, float a1, float a2) {
    // Apply the IIR filter equation
    float output = b0 * input + b1 * x1[index] + b2 * x2[index] - a1 * y1_prev[index] - a2 * y2_prev[index];

    // Shift input and output values for the next sample
    x2[index] = x1[index];
    x1[index] = input;
    y2_prev[index] = y1_prev[index];
    y1_prev[index] = output;

    return output;
}

// Function to handle button presses and update filter count
void handle_switches() {
    unsigned long current_time = millis();

    // Read the state of SW1 (increase filters)
    int sw1_state = digitalRead(SW_1_PIN);
    if (sw1_state == HIGH && (current_time - last_debounce_time_1 > debounce_delay)) {
        // Increase the filter count
        if (current_filter_count < MAX_FILTERS) {
            current_filter_count++;
            Serial.print("Increasing filters: ");
            Serial.println(current_filter_count);
        }
        last_debounce_time_1 = current_time;  // Reset debounce timer
    }

    // Read the state of SW2 (decrease filters)
    int sw2_state = digitalRead(SW_2_PIN);
    if (sw2_state == HIGH && (current_time - last_debounce_time_2 > debounce_delay)) {
        // Decrease the filter count
        if (current_filter_count > 0) {
            current_filter_count--;
            Serial.print("Decreasing filters: ");
            Serial.println(current_filter_count);
        }
        last_debounce_time_2 = current_time;  // Reset debounce timer
    }
}


void setup() {
    Serial.begin(115200);

    // Initialize switches
    pinMode(SW_1_PIN, INPUT_PULLUP);  // Use internal pull-up resistor
    pinMode(SW_2_PIN, INPUT_PULLUP);  // Use internal pull-up resistor

    // Calculate filter coefficients for all biquads (same coefficients for each filter)
    calculate_biquad_coefficients(a0_1, a1_1, a2_1, b0_1, b1_1, b2_1);

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

    Serial.println("I2S 16th-order Bandpass Filter (700Hz) initialized.");
}

void loop() {
    // Handle switches for increasing/decreasing the number of filters
    handle_switches();

    // Create buffers for I2S read and write
    const int buffer_size = 512;
    char i2s_read_buffer[buffer_size];
    size_t bytes_read, bytes_written;

    // Read data from I2S input (e.g., microphone or ADC)
    i2s_read(I2S_NUM_0, &i2s_read_buffer, buffer_size, &bytes_read, portMAX_DELAY);

    // Cast buffer to int16_t for 16-bit samples
    int16_t* samples = (int16_t*) i2s_read_buffer;
    int num_samples = bytes_read / sizeof(int16_t);  // Total number of samples

    // Apply the programmable number of biquads to only the left channel (skip right channel samples)
    for (int i = 0; i < num_samples; i += 2) {  // Step by 2 to only process left channel
        float input = (float)samples[i];  // Process only the left channel sample
        

        // Apply the current number of biquads in cascade
        float filtered_sample = input;
        for (int j = 0; j < current_filter_count; j++) {
            filtered_sample = biquad_filter(filtered_sample, j, b0_1, b1_1, b2_1, a1_1, a2_1);
        }

        // Clipping to avoid overflow
        if (filtered_sample > 32767) {
            filtered_sample = 32767;
        } else if (filtered_sample < -32768) {
            filtered_sample = -32768;
        }
        
        // Set the processed left channel sample
        samples[i] = (int16_t)filtered_sample;
        
        // Optionally: Copy left channel sample to the right channel for symmetrical output
        samples[i + 1] = (int16_t)filtered_sample;  // Copy the same filtered left sample to right channel
    }

    // Write the filtered data back to I2S output (e.g., DAC or speaker)
    i2s_write(I2S_NUM_0, &i2s_read_buffer, bytes_read, &bytes_written, portMAX_DELAY);
}

