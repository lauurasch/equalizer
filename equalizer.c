#include <pdl_header.h>
#include <platform.h>
#include <utils.h>
#include <stdio.h>
#include <math.h>
#include <string.h>  // Fuer strcmp und andere String-Operationen

//Baudrate: 460800
#define FS 8000    // CODEC sampling frequency in Hz
#define NUMTAPS 121  // Number of taps for FIR filters

// Prototypes
void I2S_HANDLER(void);
void calculate_lowpass_fir(float *coeff, int numtaps, float cutoff, float fs);
void calculate_bandpass_fir(float *coeff, int numtaps, float cutoff_low, float cutoff_high, float fs);
float apply_fir_circular(float *buffer, const float *coeff, float input, int *index);
void process_uart_input(void);
int readUart0(char *buffer, size_t size);

// Filter buffers and coefficients
float buffer_lowpass[NUMTAPS] = {0};
float coeff_lowpass[NUMTAPS];
int index_lowpass = 0; // Circular Buffer Index fuer Lowpass

float buffer_bandpass[NUMTAPS] = {0};
float coeff_bandpass[NUMTAPS];
int index_bandpass = 0; // Circular Buffer Index fuer Bandpass

float buffer_highpass[NUMTAPS] = {0};
float coeff_highpass[NUMTAPS];
int index_highpass = 0; // Circular Buffer Index fuer Highpass

// Gain factors (default values)
volatile float a_lp = 1.0f;  // Lowpass gain
volatile float b_bp = 1.0f;  // Bandpass gain
volatile float c_hp = 1.0f;  // Highpass gain

int main(void) {
    // Cutoff frequencies for each filter
    float cutoff_low = 200.0f;          // Lowpass filter cutoff
    float cutoff_band_low = 300.0f;    // Bandpass lower cutoff
    float cutoff_band_high = 2000.0f;  // Bandpass upper cutoff
    float cutoff_high_low = 2100.0f;   // Highpass as bandpass lower cutoff
    float cutoff_high_high = 3900.0f;   // Highpass as bandpass upper cutoff

    // Initialize platform resources
    platform_init(BAUDRATE, FS, line_in, intr, I2S_HANDLER, NULL);
    writeUart0("UART initialized. Ready for commands.\n");

    // Calculate FIR filter coefficients for all filters
    calculate_lowpass_fir(coeff_lowpass, NUMTAPS, cutoff_low, FS);
    calculate_bandpass_fir(coeff_bandpass, NUMTAPS, cutoff_band_low, cutoff_band_high, FS);
    calculate_bandpass_fir(coeff_highpass, NUMTAPS, cutoff_high_low, cutoff_high_high, FS);

    writeUart0("System ready. Adjust filter gains using UART commands.\n");
    writeUart0("Commands: a <value>, b <value>, c <value>, STATUS\n");

    while (1) {
        process_uart_input();  // Check and process UART input

        // Update slider parameters (optional for GUI interaction)
        update_slider_parameters(&FM4_GUI);

        // Update line-in and headphone levels (optional)
        setLIandHPlevels(&FM4_GUI);

        // Toggle activity indicator (e.g., LED)
        gpio_set(LED_B, HIGH);  // LED_B off
    }
}

void I2S_HANDLER(void) {
    union audio audioIO;
    gpio_set(TEST_PIN, HIGH);  // TestPin P10, 110 ns

    // Read input audio samples (left and right channels)
    audioIO.audioSample = I2s_ReadRxFifo(&I2S0);
    float left = audioIO.audio_ch[LEFT];
    float right = audioIO.audio_ch[RIGHT];

    // Apply filters using Circular Buffer
    float lowpass_out = apply_fir_circular(buffer_lowpass, coeff_lowpass, left, &index_lowpass);
    float bandpass_out = apply_fir_circular(buffer_bandpass, coeff_bandpass, left, &index_bandpass);
    float highpass_out = apply_fir_circular(buffer_highpass, coeff_highpass, left, &index_highpass);

    // Combine filter outputs with dynamic gain adjustments
    audioIO.audio_ch[LEFT] = a_lp * lowpass_out + b_bp * bandpass_out + c_hp * highpass_out;
    audioIO.audio_ch[RIGHT] = a_lp * lowpass_out + b_bp * bandpass_out + c_hp * highpass_out;

    // Write the processed audio to the transmit FIFO
    I2s_WriteTxFifo(&I2S0, audioIO.audioSample);
    gpio_set(TEST_PIN, LOW);   // TestPin P10, 110 ns
    gpio_set(LED_B, LOW);      // LED_B on
}

void process_uart_input(void) {
    char buffer[64];
    if (readUart0(buffer, sizeof(buffer))) {
        float value;
        if (sscanf(buffer, "a %f", &value) == 1) {
            a_lp = value;
            char output[64];
            sprintf(output, "Lowpass gain set to %.2f\n", a_lp);
            writeUart0(output);
        } else if (sscanf(buffer, "b %f", &value) == 1) {
            b_bp = value;
            char output[64];
            sprintf(output, "Bandpass gain set to %.2f\n", b_bp);
            writeUart0(output);
        } else if (sscanf(buffer, "c %f", &value) == 1) {
            c_hp = value;
            char output[64];
            sprintf(output, "Highpass gain set to %.2f\n", c_hp);
            writeUart0(output);
        } else if (strcmp(buffer, "STATUS") == 0) {
            char output[128];
            sprintf(output, "Current Gains:\nLowpass: %.2f\nBandpass: %.2f\nHighpass: %.2f\n", a_lp, b_bp, c_hp);
            writeUart0(output);
        } else {
            writeUart0("Invalid command. Use: a <value>, b <value>, c <value>, STATUS\n");
        }
    }
}

int readUart0(char *buffer, size_t size) {
    int received = 0; // Anzahl der empfangenen Zeichen
    memset(buffer, 0, size); // Puffer leeren

    while (received < size - 1) { // Einen Platz fuer die Nullterminierung freihalten
        if (Mfs_Uart_GetStatus(&UART0, UartRxFull)) { // Pruefen, ob Daten verfuegbar sind
            buffer[received] = Mfs_Uart_ReceiveData(&UART0); // Zeichen empfangen
            if (buffer[received] == '\n' || buffer[received] == '\r') { // Eingabeende erkennen
                buffer[received] = '\0'; // Nullterminierung
                break;
            }
            received++;
        }
    }

    return received; // Anzahl der empfangenen Zeichen zurueckgeben
}

void calculate_lowpass_fir(float *coeff, int numtaps, float cutoff, float fs) {
    int M = numtaps - 1;
    float fc = cutoff / fs;

    for (int n = 0; n <= M; n++) {
        if (n == M / 2) {
            coeff[n] = 2 * fc;
        } else {
            float x = n - M / 2.0;
            coeff[n] = sin(2 * M_PI * fc * x) / (M_PI * x);
        }
    }

    // Apply Hamming window
    for (int n = 0; n <= M; n++) {
        coeff[n] *= 0.54 - 0.46 * cos(2 * M_PI * n / M);
    }
}

void calculate_bandpass_fir(float *coeff, int numtaps, float cutoff_low, float cutoff_high, float fs) {
    int M = numtaps - 1;
    float fc_low = cutoff_low / fs;
    float fc_high = cutoff_high / fs;

    for (int n = 0; n <= M; n++) {
        if (n == M / 2) {
            coeff[n] = 2 * (fc_high - fc_low);
        } else {
            float x = n - M / 2.0;
            coeff[n] = (sin(2 * M_PI * fc_high * x) - sin(2 * M_PI * fc_low * x)) / (M_PI * x);
        }
    }

    // Apply Hamming window
    for (int n = 0; n <= M; n++) {
        coeff[n] *= 0.54 - 0.46 * cos(2 * M_PI * n / M);
    }
}

float apply_fir_circular(float *buffer, const float *coeff, float input, int *index) {
    // Fuege neues Sample in den Puffer ein
    buffer[*index] = input;

    // Berechne FIR-Filter
    float output = 0.0f;
    for (int i = 0; i < NUMTAPS; i++) {
        int circular_index = (*index - i + NUMTAPS) % NUMTAPS;
        output += buffer[circular_index] * coeff[i];
    }

    // Inkrementiere den Index
    *index = (*index + 1) % NUMTAPS;

    return output;
}
