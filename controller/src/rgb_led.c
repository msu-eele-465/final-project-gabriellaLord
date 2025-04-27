#include <msp430.h>
#include "msp430fr2355.h"
#include "intrinsics.h"
#include <stdbool.h>
#include <stdio.h>
#include "rgb_led.h"


void rgb_pins_init()
{       
    P1DIR |= BIT5 | BIT6 | BIT7; // Set P1.5, P1.6 y P1.7 as an output 
    P1OUT |= BIT5 | BIT6 | BIT7; // Initialize outputs as high
}

void interrupts_init()
{
    // Setup Timer B3
    TB3CTL |= TBCLR;        // Clear timer and dividers
    TB3CTL |= TBSSEL__ACLK; // Source=ACLK
    TB3CTL |= MC__UP;       // Mode=UP
    
    // Setup Timer Compare IRQ for CCR0 (Pull up)
    TB3CCR0 = 255;         // CCR0=255
    TB3CCTL0 |= CCIE;       //Enable TB3 CCR0 Overflow IRQ
    TB3CCTL0 &= ~CCIFG;     // Clear CCR0 Flag

    // Setup Timer Compare IRQ for CCR0 (Pull down RED)
    TB3CCR1 = 225-196;         // CCR1=196
    TB3CCTL1 |= CCIE;       //Enable TB3 CCR1 Overflow IRQ
    TB3CCTL1 &= ~CCIFG;     // Clear CCR1 Flag

    // Setup Timer Compare IRQ for CCR0 (Pull down GREEN)
    TB3CCR2 = 225-62;          // CCR2=62
    TB3CCTL2 |= CCIE;       //Enable TB3 CCR1 Overflow IRQ
    TB3CCTL2 &= ~CCIFG;     // Clear CCR2 Flag

    // Setup Timer Compare IRQ for CCR0 (Pull down BLUE)
    TB3CCR3 = 225-29;          //CCR3=29
    TB3CCTL3 |= CCIE;       //Enable TB3 CCR1 Overflow IRQ
    TB3CCTL3 &= ~CCIFG;     // Clear CCR3 Flag

    _enable_interrupt();    // Enable Maskable IRQs
}

void led_c43e1d(void)           // Red
{
    TB3CCR1 = 220;             // CCR1=196 (Red)
    TB3CCR2 = 1;              // CCR2=62  (Green)
    TB3CCR3 = 1;              //CCR3=29   (Blue)

}

void led_c4921d(void)           // Yellow
{
    TB3CCR1 = 220;             // CCR1=196 (Red)
    TB3CCR2 = 160;             // CCR2=146 (Green)
    TB3CCR3 = 5;              //CCR3=29   (Blue)
}

void led_1da2c4(void)           // Blue
{
    TB3CCR1 = 29;              // CCR1=29  (Red)
    TB3CCR2 = 162;             // CCR2=162 (Green)
    TB3CCR3 = 196;             //CCR3=196  (Blue)
}

void rgb_led_init(void)
{
    WDTCTL = WDTPW | WDTHOLD;   // Stop watchdog timer
    PM5CTL0 &= ~LOCKLPM5;       // Turn on GPIO

    rgb_pins_init();
    interrupts_init();
    led_c43e1d();
}

void rgb_led_continue(int lockState)
{
    switch (lockState) {
        case 0:                 // Unlocking, set yellow
            led_c4921d();
            break;
        case 1:                 // Unlocked, set blue
            led_1da2c4();
            break;
        default:                 // Locked, set red
            led_c43e1d();
            break;
    
    }
}

//----------------------------------------------------------------------
// Begin Interrupt Service Routine
//----------------------------------------------------------------------
#pragma vector = TIMER3_B0_VECTOR
__interrupt void ISR_TB3_CCR0(void)
{
    P1OUT |= BIT5 | BIT6 | BIT7;
    TB3CCTL0 &= ~CCIFG;
}
//----------------------------------------------------------------------
// CCR1, CCR2, CCR3, and overflow interrupt combined
#pragma vector = TIMER3_B1_VECTOR
__interrupt void ISR_TB3_CCRn(void)
{
    switch (__even_in_range(TB3IV, 14)) // Handle interrupts by priority
    {
        case 0: break;                  // No interrupt
        case 2:                         // CCR1 interrupt (RED)
            P1OUT &= ~BIT5;             // RED off
            TB3CCTL1 &= ~CCIFG;
            break;
        case 4:                         // CCR2 interrupt (GREEN)
            P1OUT &= ~BIT6;             // GREEN off
            TB3CCTL2 &= ~CCIFG;
            break;
        case 6:                         // CCR3 interrupt (BLUE)
            P1OUT &= ~BIT7;             // BLUE off
            TB3CCTL3 &= ~CCIFG;
            break;
        case 14:                        // Timer overflow
            TB3CTL &= ~TBIFG;
            break;
        default: break;
    }
}
//--End Interrupt Service Routine---------------------------------------
