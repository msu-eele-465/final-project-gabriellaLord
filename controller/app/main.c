/**
 * @file aoa_detection_msp430.c
 * @brief Calculates angle of arrival (AOA) using cross-correlation on MSP430
 *
 * This project uses two ADC channels to sample left and right microphones,
 * calculates the cross-correlation, and determines the direction of sound.
 * The result is sent to an LCD via I2C and displayed using RGB LEDs.
 */

#include <msp430.h>
#include <stdint.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include "intrinsics.h"
#include "../../src/master_i2c.h"
#include "../../src/heartbeat.h"
#include "../../src/rgb_led.h"

//------------------------------------------------------------------------------
// Constants and Definitions
//------------------------------------------------------------------------------
#define SAMPLE_SIZE     100         // Number of samples per channel
#define SAMPLE_STEP     5           // Step interval to trigger AoA calculation
#define SPEED_OF_SOUND  343.0f      // Speed of sound in m/s
#define MIC_DISTANCE    0.26f       // Distance between microphones in meters
#define PI              3.1415926f
#define FS              12000.0f    // Sampling frequency in Hz
#define ADDR_LCD        0x38        // I2C address of the LCD slave

//------------------------------------------------------------------------------
// Global Variables
//------------------------------------------------------------------------------
float crossCorr[2 * SAMPLE_SIZE - 1];           // Cross-correlation results
int lags[2 * SAMPLE_SIZE - 1];                  // Lags associated with each correlation value
volatile float aoa;                             // Calculated angle of arrival (degrees)
volatile float timeDelay;                       // Calculated time delay (s)
volatile bool adc_ready = false;                // Flag to trigger AoA calculation
volatile unsigned int adc_result = 0;           // Reading from ADC
volatile unsigned int sample_index = 0;         // Current index in sample buffer
volatile unsigned int filled_samples = 0;       // Number of valid samples stored
volatile float left_row[SAMPLE_SIZE] = {0};     // Circular buffer for left mic
volatile float right_row[SAMPLE_SIZE] = {0};    // Circular buffer for right mic

//------------------------------------------------------------------------------
// Initializations
//------------------------------------------------------------------------------
/**
 * @brief Configure watchdog, clock, and GPIO
 */
void system_init(void) {
    WDTCTL = WDTPW | WDTHOLD; // Stop watchdog timer
    PM5CTL0 &= ~LOCKLPM5;     // Enable GPIO
}

/**
 * @brief Configure ADC channels and timer for 12 kHz sampling
 */
void adc_init(void) {
    // Configure GPIO pins for ADC
    P1SEL0 |= BIT2 | BIT3;
    P1DIR &= ~(BIT2 | BIT3);

    // ADC setup: SMCLK source, 12-bit resolution
    ADCCTL0 = ADCSHT_2 | ADCON;
    ADCCTL1 = ADCSHP | ADCSSEL_2;
    ADCCTL2 = ADCRES_2;
    ADCMCTL0 = ADCINCH_2; // Start with left mic

    // Timer setup for 12 kHz sampling rate
    TB1CTL = TBSSEL__SMCLK | MC__UP | TBCLR;
    TB1CCR0 = 83; // 1 MHz / 12000 Hz = 83.3
    TB1CCTL0 = CCIE;
}

//------------------------------------------------------------------------------
// I2C LCD Output Helpers
//------------------------------------------------------------------------------
/**
 * @brief Send a 4-digit unsigned integer via I2C
 * @param mode Mode character ('A', 'B', etc.)
 * @param input Value to send
 */
void send_nat_4_digit(char mode, int input) {
    char buffer[5];
    int j;

    // Convert integer to characters
    buffer[0] = (input / 1000) % 10 + '0';
    buffer[1] = (input / 100) % 10 + '0';
    buffer[2] = (input / 10) % 10 + '0';
    buffer[3] = input % 10 + '0';
    buffer[4] = '\0';

    master_i2c_send(mode, ADDR_LCD);            // Send mode
    for (j = 0; buffer[j] != '\0'; j++) {
        master_i2c_send(buffer[j], ADDR_LCD);   // Send digits
    }
}

/**
 * @brief Send a 3-digit signed integer via I2C
 * @param mode Mode character ('A', 'B', etc.)
 * @param input Value to send
 */
void send_int_3_digit(char mode, int input) {
    char buffer[5];
    int j;
    int value = input;

    // Handle sign
    buffer[0] = (input < 0) ? '-' : '+';
    if (input < 0) value = -input;

    // Convert integer to characters
    buffer[1] = (value / 100) % 10 + '0';
    buffer[2] = (value / 10) % 10 + '0';
    buffer[3] = value % 10 + '0';
    buffer[4] = '\0';

    master_i2c_send(mode, ADDR_LCD);
    for (j = 0; buffer[j] != '\0'; j++) {
        master_i2c_send(buffer[j], ADDR_LCD);
    }
}

//------------------------------------------------------------------------------
// Triangulation Calculations
//------------------------------------------------------------------------------
/**
 * @brief Custom fabsf replacement for portability
 */
float my_fabs(float x) {
    return (x < 0) ? -x : x;
}

/**
 * @brief Approximate arccos(x) using Taylor series
 * @param x Input value in range [-1, 1]
 * @return Approximated arccos(x)
 */
float my_acos(float x) {
    if (x > 1.0f) x = 1.0f;
    if (x < -1.0f) x = -1.0f;

    float x2 = x * x;
    float x3 = x2 * x;
    float x5 = x3 * x2;
    float x7 = x5 * x2;

    float result = (PI / 2.0f)
                   - x
                   - (x3 / 6.0f)
                   - (3.0f * x5 / 40.0f)
                   - (5.0f * x7 / 112.0f);

    return result;
}

/**
 * @brief Find index of maximum absolute value in array
 * @param arr Input array
 * @param len Array length
 * @return Index of maximum element
 */
int find_max_index(const float *arr, int len) {
    int maxIndex = 0;
    float maxVal = my_fabs(arr[0]);
    int i;
    for (i = 1; i < len; i++) {
        float val = my_fabs(arr[i]);
        if (val > maxVal) {
            maxVal = val;
            maxIndex = i;
        }
    }
    return maxIndex;
}

/**
 * @brief Perform circular cross-correlation between two buffers
 * @param x Left microphone buffer
 * @param y Right microphone buffer
 * @param corr Output correlation array
 * @param lag Output lags array
 */
void xcorr(volatile float *x, volatile float *y, float *corr, int *lag) {
    int len = 2 * SAMPLE_STEP - 1;
    int mid = SAMPLE_STEP - 1;
    int i, j;

    for (i = 0; i < len; i++) {
        int shift = i - mid;
        lag[i] = shift;
        corr[i] = 0.0f;

        for (j = 0; j < SAMPLE_STEP; j++) {
            int xi = (sample_index + j - SAMPLE_STEP + SAMPLE_SIZE) % SAMPLE_SIZE;
            int yi = (xi - shift + SAMPLE_SIZE) % SAMPLE_SIZE;
            corr[i] += x[xi] * y[yi];
        }
    }
}

/**
 * @brief Calculate angle of arrival (AOA) from sampled mic data
 */
void calculate_aoa(void) {
    int len = 2 * SAMPLE_STEP - 1;
    xcorr(left_row, right_row, crossCorr, lags);

    int index = find_max_index(crossCorr, len);
    timeDelay = (float)lags[index] / FS;

    float ratio = timeDelay * SPEED_OF_SOUND / MIC_DISTANCE;
    if (ratio > 1.0f) ratio = 1.0f;
    if (ratio < -1.0f) ratio = -1.0f;

    aoa = my_acos(ratio) * 180.0f / PI;
    __no_operation();
}

//------------------------------------------------------------------------------
// Main Function
//------------------------------------------------------------------------------
int main(void) {
    system_init();              // Configure MSP430
    heartbeat_init();
    rgb_led_init();
    master_i2c_init();
    adc_init();                 // Initialize ADC and timer
    __enable_interrupt();
    _no_operation();

    while (1) {
        if (adc_ready) {
            adc_ready = false;
            calculate_aoa();    // Compute angle of arrival

            int aoa_int = aoa * 10;              // For display: AoA in tenths of degree
            int timeDelay_int = timeDelay * 1e6; // Time delay in microseconds

            send_nat_4_digit('A', aoa_int);
            send_int_3_digit('B', timeDelay_int);

            // Determine direction
            if (aoa <= 22.5f) {
                master_i2c_send('7', ADDR_LCD); // W
                rgb_led_continue(0);
            } else if (aoa <= 67.5f) {
                master_i2c_send('8', ADDR_LCD); // NW
                rgb_led_continue(1);
            } else if (aoa <= 112.5f) {
                master_i2c_send('1', ADDR_LCD); // N
                rgb_led_continue(2);
            } else if (aoa <= 157.5f) {
                master_i2c_send('2', ADDR_LCD); // NE
                rgb_led_continue(1);
            } else {
                master_i2c_send('3', ADDR_LCD); // E
                rgb_led_continue(0);
            }
        }
    }
}

//------------------------------------------------------------------------------
// Interrupt Service Routine
//------------------------------------------------------------------------------
#pragma vector = TIMER1_B0_VECTOR
__interrupt void ISR_TB1_CCR0(void) {
    static bool readLeft = true;

    ADCCTL0 |= ADCENC | ADCSC;              // Start ADC conversion
    while (ADCCTL1 & ADCBUSY);              // Wait for conversion to complete
    adc_result = ADCMEM0;                   // Read result
    float voltage = (adc_result * 3.3f) / 4095.0f; // Convert to voltage

    if (readLeft) {
        left_row[sample_index] = voltage;
        ADCMCTL0 = ADCINCH_3;               // Switch to A3 (right)
    } else {
        right_row[sample_index] = voltage;
        ADCMCTL0 = ADCINCH_2;               // Switch to A2 (left)
        sample_index = (sample_index + 1) % SAMPLE_SIZE;

        if (filled_samples < SAMPLE_SIZE) {
            filled_samples++;
        } else if (sample_index % SAMPLE_STEP == 0) {
            adc_ready = true;
        }
    }

    readLeft = !readLeft;
    TB1CCTL0 &= ~CCIFG;                     // Clear interrupt flag
}
