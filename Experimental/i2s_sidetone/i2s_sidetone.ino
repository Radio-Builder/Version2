#include "driver/i2s.h"
#include <math.h>  // For exp() function

#define I2S_SAMPLE_RATE   22050 //44100
#define I2S_SAMPLE_BITS   I2S_BITS_PER_SAMPLE_16BIT
#define TONE_FREQUENCY    650  // Frequency of the sidetone (Hz)
#define RAMP_DURATION_MS  5    // Ramp up/down duration (ms)
#define GPIO_SW1          19   // Key (SW1) GPIO pin
#define SAMPLES_PER_CYCLE (I2S_SAMPLE_RATE / TONE_FREQUENCY)
#define RAMP_SAMPLES      (I2S_SAMPLE_RATE * RAMP_DURATION_MS / 1000)  // Number of samples for ramp
#define SIGMOID_K         10.0  // Steepness of the S-curve

int16_t ramp_up[2 * RAMP_SAMPLES];  // Two samples per cycle (left + right)
int16_t steady_tone[2 * SAMPLES_PER_CYCLE];
int16_t ramp_down[2 * RAMP_SAMPLES];
int16_t silence[2 * SAMPLES_PER_CYCLE];  // Silence buffer for both channels

bool key_was_pressed = false;
float phase = 0.0f;  // Global phase tracking for smooth transition
float phase_increment = 2.0f * 3.14159265f / SAMPLES_PER_CYCLE;  // Phase increment per sample

// I2S pin configuration
i2s_pin_config_t pin_config = {
    .bck_io_num = 5,
    .ws_io_num = 33,
    .data_out_num = 25,
    .data_in_num = -1  // Not used
};

// Sigmoid function to generate the S-curve
float sigmoid(float x) {
    return 1.0 / (1.0 + exp(-SIGMOID_K * (x - 0.5)));
}

void generate_tone_arrays() {
    // Generate ramp-up samples using sigmoid S-curve
    for (int i = 0; i < RAMP_SAMPLES; i++) {
        float x = (float)i / (RAMP_SAMPLES - 1);  // Normalize x to range [0, 1]
        float ramp_factor = sigmoid(x);  // S-curve ramp-up factor
        int16_t sample = (int16_t)(ramp_factor * sin(phase) * 32767);

        ramp_up[2 * i] = sample;      // Left channel
        ramp_up[2 * i + 1] = 0;      // Right channel (silence)

        // Update phase for each sample
        phase += phase_increment;
        if (phase >= 2.0f * 3.14159265f) phase -= 2.0f * 3.14159265f;
    }

    // Generate steady tone samples (continue from the last phase of ramp-up)
    for (int i = 0; i < SAMPLES_PER_CYCLE; i++) {
        int16_t sample = (int16_t)(sin(phase) * 32767);

        steady_tone[2 * i] = sample;      // Left channel
        steady_tone[2 * i + 1] = 0;      // Right channel (silence)

        phase += phase_increment;
        if (phase >= 2.0f * 3.14159265f) phase -= 2.0f * 3.14159265f;
    }

    // Generate ramp-down samples using inverse sigmoid S-curve
    for (int i = 0; i < RAMP_SAMPLES; i++) {
        float x = (float)i / (RAMP_SAMPLES - 1);  // Normalize x to range [0, 1]
        float ramp_factor = sigmoid(1.0 - x);  // Inverse S-curve ramp-down
        int16_t sample = (int16_t)(ramp_factor * sin(phase) * 32767);

        ramp_down[2 * i] = sample;      // Left channel
        ramp_down[2 * i + 1] = 0;       // Right channel (silence)

        phase += phase_increment;
        if (phase >= 2.0f * 3.14159265f) phase -= 2.0f * 3.14159265f;
    }

    // Generate silence samples
    for (int i = 0; i < SAMPLES_PER_CYCLE; i++) {
        silence[2 * i] = 0;         // Left channel
        silence[2 * i + 1] = 0;     // Right channel
    }
}

void do_sidetone(bool key_pressed) {
    int16_t *current_buffer = nullptr;
    size_t current_buffer_len = 0;

    if (key_pressed && !key_was_pressed) {
        // Key was just pressed, send ramp-up
        current_buffer = ramp_up;
        current_buffer_len = sizeof(ramp_up);
    } else if (key_pressed) {
        // Key is still pressed, send steady tone
        current_buffer = steady_tone;
        current_buffer_len = sizeof(steady_tone);
    } else if (!key_pressed && key_was_pressed) {
        // Key was just released, send ramp-down (now phase continues smoothly)
        current_buffer = ramp_down;
        current_buffer_len = sizeof(ramp_down);
    } else {
        // Silence buffer after ramp down
        current_buffer = silence;
        current_buffer_len = sizeof(silence);
    }

    // Send the current buffer (if any)
    if (current_buffer != NULL) {
        size_t bytes_written;
        i2s_write(I2S_NUM_0, current_buffer, current_buffer_len, &bytes_written, portMAX_DELAY);
    }

    // Update the key state
    key_was_pressed = key_pressed;
}

void setup() {
    // Initialize I2S driver
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
        .sample_rate = I2S_SAMPLE_RATE,
        .bits_per_sample = I2S_SAMPLE_BITS,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_I2S,
        .intr_alloc_flags = 0,
        .dma_buf_count = 2,  // Minimal buffer setup
        .dma_buf_len = 64,   // Small buffer length
        .use_apll = false,
    };

    i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_NUM_0, &pin_config);

    // Set up SW1 key (normally pulled down)
    pinMode(GPIO_SW1, INPUT);

    // Pre-generate tone and ramp arrays
    generate_tone_arrays();
}

void loop() {
    // Read the key state (SW1) and call do_sidetone function
    bool key_pressed = (digitalRead(GPIO_SW1) == HIGH);
    do_sidetone(key_pressed);
}
