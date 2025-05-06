#include "intrinsics.h"
#include <msp430.h>

int Data_Cnt = 0;                           // Set initially zero data packets sent
char Packet[] = {'1', '2', '3'};            // Create an array for the data to be sent

int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;               // Stop watchdog timer
    
    UCB1CTLW0 |= UCSWRST;                   // Put eUSCI_B0 into software reset

    UCB1CTLW0 |= UCSSEL_3;                  // Choose BRCLK=SMCLK=1MHz
    UCB1BRW = 10;                           // Divide BRCLK by 10 for SCL=100kHz

    UCB1CTLW0 |= UCMODE_3;                  // Put into I2C mode
    UCB1CTLW0 |= UCMST;                     // Put into master mode
    UCB1CTLW0 |= UCTR;                      // Put into Tx mode
    UCB1I2CSA = 0x0068;                     // Slave address = 0x68

    UCB1CTLW1 |= UCASTP_2;                  // Auto STOP when UCB0TBCNT reached
    UCB1TBCNT = sizeof(Packet);             // # of Bytes in Packet

    P4SEL1 &= ~BIT6;                        // We want P1.2 = SDA
    P4SEL0 |= BIT6;
    P4SEL1 &= ~BIT7;                        // We want P1.3 = SCL
    P4SEL0 |= BIT7;

    P6OUT &= ~BIT6;                         // Clear P1.0 output latch for a defined power-on state
    P6DIR |= BIT6;                          // Set P1.0 to output direction

    PM5CTL0 &= ~LOCKLPM5;                   // Disable the GPIO power-on default high-impedance mode
                                            // to activate previously configured port settings
    UCB1CTLW0 &= ~UCSWRST;                  // Take eUSCI_B0 out of SW reset
    UCB1IE |= UCTXIE0;                      // Enable I2C Tx0 IRQ
    __enable_interrupt();                   // Enable Maskable IRQs

    int i;
    
    while(1)
    {
        //P6OUT ^= BIT6;                      // Toggle P1.0 using exclusive-OR
        //__delay_cycles(100000);             // Delay for 100000*(1/MCLK)=0.1s
    
        UCB1CTLW0 |= UCTXSTT;               // Generate START condition
        __delay_cycles(50000);           // delay loop
    }
    return 0;
}

//----------------------------------------------------------------------
// Begin Interrupt Service Routine
//----------------------------------------------------------------------

#pragma vector=EUSCI_B1_VECTOR
__interrupt void EUSCI_B1_I2C_ISR(void){
    if (Data_Cnt == sizeof(Packet) - 1){
        UCB1TXBUF = Packet[Data_Cnt];
        Data_Cnt = 0;
    } else {
        UCB1TXBUF = Packet[Data_Cnt];
        Data_Cnt++;
    }
}

