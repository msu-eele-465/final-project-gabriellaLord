#include "intrinsics.h"
#include "msp430fr2355.h"
#include <msp430.h>
#include <stdbool.h>
#include "heartbeat.h"

/**
 * @brief Initialize heartbeat LED on P6.6 with 1Hz toggle rate using Timer B0
 */
void heartbeat_init()
{
    WDTCTL = WDTPW | WDTHOLD;        // Stop watchdog timer

    // Configure GPIO for heartbeat LED
    P6DIR |= BIT6;                   // Set P6.6 as output
    P6OUT &= ~BIT6;                  // Start with LED off
    PM5CTL0 &= ~LOCKLPM5;            // Enable GPIO functionality

    // Configure Timer B0
    TB0CTL |= TBCLR;                 // Clear timer and dividers
    TB0CTL |= TBSSEL__ACLK;          // Source = ACLK (typically 32.768kHz)
    TB0CTL |= MC__UP;                // Count up mode
    TB0CCR0 = 32768;                 // 1 second overflow with ACLK

    // Enable Timer B0 CCR0 interrupt
    TB0CCTL0 &= ~CCIFG;              // Clear interrupt flag
    TB0CCTL0 |= CCIE;                // Enable CCR0 interrupt

    __enable_interrupt();            // Enable maskable interrupts
}

//------------------------------------------------------------------------------
// Interrupt Service Routine
//------------------------------------------------------------------------------

/**
 * @brief ISR for Timer B0 CCR0 - toggles P6.6 every 1s
 */
#pragma vector = TIMER0_B0_VECTOR
__interrupt void ISR_TB0_CCR0(void)
{
    P6OUT ^= BIT6;                  // Toggle LED
    TB0CCTL0 &= ~CCIFG;             // Clear interrupt flag
}
