#include <msp430.h>
#include "msp430fr2355.h"
#include "intrinsics.h"
#include <stdbool.h>
#include <stdio.h>
#include "rgb_led.h"

/**
 * @brief Configure RGB LED GPIO pins as outputs and initialize to HIGH
 */
void rgb_pins_init()
{       
    P1DIR |= BIT5 | BIT6 | BIT7; // Set P1.5, P1.6, P1.7 as outputs
    P1OUT |= BIT5 | BIT6 | BIT7; // Initialize outputs as high (LEDs off)
}

/**
 * @brief Configure TimerB3 for PWM-like control of RGB LED via CCR interrupts
 */
void interrupts_init()
{
    TB3CTL |= TBCLR;          // Clear timer and dividers
    TB3CTL |= TBSSEL__ACLK;   // Set ACLK as timer source
    TB3CTL |= MC__UP;         // Set timer mode to UP

    TB3CCR0 = 255;            // Period for PWM
    TB3CCTL0 |= CCIE;         // Enable CCR0 interrupt (reset RGB pins high)
    TB3CCTL0 &= ~CCIFG;       // Clear interrupt flag

    // RED duty cycle control
    TB3CCR1 = 220;            // RED
    TB3CCTL1 |= CCIE;         // Enable CCR1 interrupt
    TB3CCTL1 &= ~CCIFG;

    // GREEN duty cycle control
    TB3CCR2 = 160;            // GREEN
    TB3CCTL2 |= CCIE;         // Enable CCR2 interrupt
    TB3CCTL2 &= ~CCIFG;

    // BLUE duty cycle control
    TB3CCR3 = 196;            // BLUE
    TB3CCTL3 |= CCIE;         // Enable CCR3 interrupt
    TB3CCTL3 &= ~CCIFG;

    _enable_interrupt();      // Enable maskable interrupts globally
}

/**
 * @brief Set RGB to red (hex: #c43e1d)
 */
void led_c43e1d(void)
{
    TB3CCR1 = 220; // RED
    TB3CCR2 = 1;   // GREEN
    TB3CCR3 = 1;   // BLUE
}

/**
 * @brief Set RGB to yellow (hex: #c4921d)
 */
void led_c4921d(void)
{
    TB3CCR1 = 220; // RED
    TB3CCR2 = 160; // GREEN
    TB3CCR3 = 5;   // BLUE
}

/**
 * @brief Set RGB to blue (hex: #1da2c4)
 */
void led_1da2c4(void)
{
    TB3CCR1 = 29;   // RED
    TB3CCR2 = 162;  // GREEN
    TB3CCR3 = 196;  // BLUE
}

/**
 * @brief Initialize GPIOs, timer, and default LED color
 */
void rgb_led_init(void)
{
    WDTCTL = WDTPW | WDTHOLD;   // Stop watchdog timer
    PM5CTL0 &= ~LOCKLPM5;       // Unlock GPIO configuration

    rgb_pins_init();            // Setup LED pins
    interrupts_init();          // Setup TimerB3 and interrupts
    led_c43e1d();               // Default LED color (Red)
}

/**
 * @brief Change LED color depending on lock state
 * @param lockState 0: yellow (unlocking), 1: blue (unlocked), default: red (locked)
 */
void rgb_led_continue(int lockState)
{
    switch (lockState) {
        case 0:
            led_c4921d(); // Yellow
            break;
        case 1:
            led_1da2c4(); // Blue
            break;
        default:
            led_c43e1d(); // Red
            break;
    }
}

//------------------------------------------------------------------------------
// Interrupt Service Routines for RGB LED timing
//------------------------------------------------------------------------------

/**
 * @brief CCR0 ISR - Sets all RGB pins HIGH (beginning of PWM cycle)
 */
#pragma vector = TIMER3_B0_VECTOR
__interrupt void ISR_TB3_CCR0(void)
{
    P1OUT |= BIT5 | BIT6 | BIT7; // Turn off LEDs by setting pins HIGH
    TB3CCTL0 &= ~CCIFG;          // Clear interrupt flag
}

/**
 * @brief CCR1/2/3 ISR - Clears RGB pins LOW at duty cycles (PWM effect)
 */
#pragma vector = TIMER3_B1_VECTOR
__interrupt void ISR_TB3_CCRn(void)
{
    switch (__even_in_range(TB3IV, 14)) // TB3IV holds source of interrupt
    {
        case 0:
            break; // No interrupt
        case 2:
            P1OUT &= ~BIT5;     // RED off
            TB3CCTL1 &= ~CCIFG; // Clear flag
            break;
        case 4:
            P1OUT &= ~BIT6;     // GREEN off
            TB3CCTL2 &= ~CCIFG;
            break;
        case 6:
            P1OUT &= ~BIT7;     // BLUE off
            TB3CCTL3 &= ~CCIFG;
            break;
        case 14:
            TB3CTL &= ~TBIFG;   // Clear timer overflow flag
            break;
        default:
            break;
    }
}
