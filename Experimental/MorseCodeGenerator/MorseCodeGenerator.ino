#include "driver/i2s.h"
#include <Arduino.h>
#include <math.h>  // For sine function

// Morse code dot and dash durations at 20 WPM
#define DOT_DURATION      60    // 60 ms for dot
#define DASH_DURATION     180   // 180 ms for dash
#define INTRA_SYMBOL_SPACE 60   // 60 ms between dots and dashes within the same letter
#define LETTER_SPACE      180   // 180 ms between letters
#define WORD_SPACE        420   // 420 ms between words

#define TONE_FREQUENCY    500   // Frequency of the morse tone
#define RAMP_DURATION     5     // 5 ms ramp duration to avoid clicks
#define GPIO_SW1          19    // Start key (SW1) GPIO pin
#define GPIO_SW2          16    // Stop key (SW2) GPIO pin
#define I2S_SAMPLE_RATE   44100
#define I2S_SAMPLE_BITS   I2S_BITS_PER_SAMPLE_16BIT

// 1ms silence buffer
#define SMALL_SILENCE_DURATION 1 // 1ms silence buffer
#define SMALL_SILENCE_SAMPLES (I2S_SAMPLE_RATE / 1000 * SMALL_SILENCE_DURATION)

int16_t dot_buffer[2 * (I2S_SAMPLE_RATE / 1000 * DOT_DURATION)];   // Double for stereo (left + right)
int16_t dash_buffer[2 * (I2S_SAMPLE_RATE / 1000 * DASH_DURATION)];  // Double for stereo (left + right)
int16_t small_silence_buffer[2 * SMALL_SILENCE_SAMPLES];           // Double for stereo (left + right)

bool generator_running = false;
size_t bytes_written;

// I2S pin configuration
i2s_pin_config_t pin_config = {
    .bck_io_num = 5,
    .ws_io_num = 33,
    .data_out_num = 25,
    .data_in_num = -1  // Not used
};

// Morse code table for A-Z, 0-9, '.', and ','
const char* morse_table[] = {
    ".-", "-...", "-.-.", "-..", ".", "..-.", "--.", "....", "..",  // A-I
    ".---", "-.-", ".-..", "--", "-.", "---", ".--.", "--.-", ".-.", // J-R
    "...", "-", "..-", "...-", ".--", "-..-", "-.--", "--..",        // S-Z
    "-----", ".----", "..---", "...--", "....-", ".....", "-....",   // 0-5
    "--...", "---..", "----.", ".-.-.-", "--..--"                    // 6-9, '.' and ','
};

// Function to apply ramp-up and ramp-down
void apply_ramp(int16_t* buffer, int num_samples) {
    int ramp_samples = I2S_SAMPLE_RATE / 1000 * RAMP_DURATION;

    // Ramp-up
    for (int i = 0; i < ramp_samples; i++) {
        float multiplier = (float)i / ramp_samples;
        buffer[2 * i] *= multiplier;
        buffer[2 * i + 1] *= multiplier;
    }

    // Ramp-down
    for (int i = 0; i < ramp_samples; i++) {
        float multiplier = (float)(ramp_samples - i) / ramp_samples;
        buffer[2 * (num_samples - ramp_samples + i)] *= multiplier;
        buffer[2 * (num_samples - ramp_samples + i) + 1] *= multiplier;
    }
}

void setup() {
    // Initialize the I2S driver
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
        .sample_rate = I2S_SAMPLE_RATE,
        .bits_per_sample = I2S_SAMPLE_BITS,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_I2S,
        .intr_alloc_flags = 0,
        .dma_buf_count = 2,
        .dma_buf_len = 64,
        .use_apll = false,
    };
    i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_NUM_0, &pin_config);

    // Pre-generate dot tone with ramp
    int num_samples_dot = I2S_SAMPLE_RATE / 1000 * DOT_DURATION;
    for (int i = 0; i < num_samples_dot; i++) {
        int16_t sample = (int16_t)(sin(2.0f * 3.14159265f * i * TONE_FREQUENCY / I2S_SAMPLE_RATE) * 32767);
        dot_buffer[2 * i] = sample;      // Left channel
        dot_buffer[2 * i + 1] = sample;  // Right channel
    }
    apply_ramp(dot_buffer, num_samples_dot);

    // Pre-generate dash tone with ramp
    int num_samples_dash = I2S_SAMPLE_RATE / 1000 * DASH_DURATION;
    for (int i = 0; i < num_samples_dash; i++) {
        int16_t sample = (int16_t)(sin(2.0f * 3.14159265f * i * TONE_FREQUENCY / I2S_SAMPLE_RATE) * 32767);
        dash_buffer[2 * i] = sample;      // Left channel
        dash_buffer[2 * i + 1] = sample;  // Right channel
    }
    apply_ramp(dash_buffer, num_samples_dash);

    // Pre-generate small silence buffer for pauses
    memset(small_silence_buffer, 0, sizeof(small_silence_buffer));

    // Set up SW1 and SW2 keys
    pinMode(GPIO_SW1, INPUT);
    pinMode(GPIO_SW2, INPUT);
}

void loop() {
    // Start Morse code generation when SW1 is pressed
    if (digitalRead(GPIO_SW1) == HIGH && !generator_running) {
        generator_running = true;
    }

    // Stop Morse code generation when SW2 is pressed
    if (digitalRead(GPIO_SW2) == HIGH && generator_running) {
        generator_running = false;
    }

    // If the generator is running, produce random Morse code
    if (generator_running) {
        // Randomly select a character (A-Z, 0-9, '.', or ',')
        int rand_index = random(0, sizeof(morse_table) / sizeof(morse_table[0]));
        const char* morse_code = morse_table[rand_index];

        // Play the Morse code sequence for the selected character
        while (*morse_code != '\0') {
            if (*morse_code == '.') {
                play_dot();
            } else if (*morse_code == '-') {
                play_dash();
            }
            play_pause(INTRA_SYMBOL_SPACE);  // Space between parts of the same letter (60ms)
            morse_code++;
        }
        play_pause(LETTER_SPACE);  // Space between letters (180ms)
    }
}

// Function to play a dot using pre-generated buffer
void play_dot() {
    i2s_write(I2S_NUM_0, dot_buffer, sizeof(dot_buffer), &bytes_written, portMAX_DELAY);
}

// Function to play a dash using pre-generated buffer
void play_dash() {
    i2s_write(I2S_NUM_0, dash_buffer, sizeof(dash_buffer), &bytes_written, portMAX_DELAY);
}

// Function to play a pause (silence) using the 1ms silence buffer repeatedly
void play_pause(int duration) {
    int total_samples = I2S_SAMPLE_RATE / 1000 * duration;
    int chunks = total_samples / SMALL_SILENCE_SAMPLES;

    // Send the small silence buffer repeatedly to match the total duration
    for (int i = 0; i < chunks; i++) {
        i2s_write(I2S_NUM_0, small_silence_buffer, sizeof(small_silence_buffer), &bytes_written, portMAX_DELAY);
    }

    // If there are remaining samples (not a full chunk), write the remainder
    int remainder_samples = total_samples % SMALL_SILENCE_SAMPLES;
    if (remainder_samples > 0) {
        i2s_write(I2S_NUM_0, small_silence_buffer, 2 * remainder_samples * sizeof(int16_t), &bytes_written, portMAX_DELAY);
    }
}
