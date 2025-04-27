#include "intrinsics.h"
#include "msp430fr2355.h"
#include <msp430.h>
#include <stdbool.h>
#include "heartbeat.h"

void heartbeat_init()
{
    // Stop watchdog timer
    WDTCTL = WDTPW | WDTHOLD;               // Stop watchdog timer
    
    // Setup Ports
    P6DIR |= BIT6;              // Config P1.0 as output
    P6OUT &= ~BIT6;             // Clear 1.0 to start
    PM5CTL0 &= ~LOCKLPM5;       // Turn on GPIO

    // Setup Timer
    TB0CTL |= TBCLR;            // Clear timers and dividers
    TB0CTL |= TBSSEL__ACLK;     // Source = ACLK
    TB0CTL |= MC__UP;           // Mode = UP
    TB0CCR0 = 32768;            // CCR0 = 32768 (1s overflow)
    // 8192 = 0.25 seconds

    // Setup Timer Compare IRQ
    TB0CCTL0 &= ~CCIFG;         //Clear CCR0 Flag
    TB0CCTL0 |= CCIE;           // Enable TB0 CCR0 Overflow IRQ
    __enable_interrupt();       // Enable Maskable IRQ
}

//-- Interrupt Service Routines ------------------
#pragma vector = TIMER0_B0_VECTOR
__interrupt void ISR_TB0_CCR0(void)
{
    P6OUT ^= BIT6;
    TB0CCTL0 &= ~CCIFG;
}
