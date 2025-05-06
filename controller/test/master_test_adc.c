#include <msp430.h>
#include <math.h>

volatile unsigned int window_size = 3;
volatile unsigned int adc_result = 0;
volatile unsigned int samples[9] = {0};
volatile unsigned int sample_index = 0;
volatile unsigned int temp_C = 0;

void compute_moving_average(void) {
    unsigned long sum = 0;
    int i;
    for (i = 0; i < window_size; i++) {
        sum += samples[i];
    }
    unsigned int avg_adc = sum / window_size;

    // Convert ADC to voltage
    float voltage = (avg_adc * 3.3f) / 4095.0f;

    // LM19 sensor voltage to temperature conversion
    // Vtemp = -11.69 mV/°C * T + 1.8639 V → Solve for T
    temp_C = (unsigned int)((1.8639f - voltage) / 0.01169f);
}

void main(void)
{
    WDTCTL = WDTPW | WDTHOLD;   // Stop watchdog timer

    // Configure GPIO
    P1SEL0 |= BIT3;             // P1.3 = A3 (analog)
    P1DIR &= ~BIT3;             // Input

    // Configure ADC
    ADCCTL0 = ADCSHT_2 | ADCON;
    ADCCTL1 = ADCSHP | ADCSSEL_1;    // Use sampling timer, ACLK
    ADCCTL2 = ADCRES_2;              // 12-bit
    ADCMCTL0 = ADCINCH_3;            // Channel A3

    // Timer setup (ACLK = 32768 Hz → 0.5s = 16384)
    TB1CTL = TBSSEL__ACLK | MC__UP | TBCLR;
    TB1CCR0 = 16384;
    TB1CCTL0 = CCIE;

    __enable_interrupt();

    while (1)
    {
        __low_power_mode_3();
    }
}

#pragma vector = TIMER1_B0_VECTOR
__interrupt void ISR_TB1_CCR0(void)
{
    ADCCTL0 |= ADCENC | ADCSC;             // Start ADC conversion
    while (ADCCTL1 & ADCBUSY);             // Wait for result
    adc_result = ADCMEM0;                  // Get result

    samples[sample_index++] = adc_result;
    if (sample_index >= window_size)
    {
        sample_index = 0;
        compute_moving_average();
    }

    TB1CCTL0 &= ~CCIFG;                    // Clear interrupt flag
}
