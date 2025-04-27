#include <msp430.h>

int data_in = 0;

void i2c_init(void)
{
    UCB1CTLW0 |= UCSWRST;                   // Put eUSCI_B0 into software reset

    UCB1CTLW0 |= UCSSEL_3;                  // Choose BRCLK=SMCLK=1MHz
    UCB1BRW = 10;                           // Divide BRCLK by 10 for SCL=100kHz

    UCB1CTLW0 |= UCMODE_3;                  // Put into I2C mode
    UCB1CTLW0 |= UCMST;                     // Put into master mode
    UCB1I2CSA = 0x068;                      // Slave address = 0x68

    UCB1CTLW1 |= UCASTP_2;                  // Auto STOP when UCB1TBCNT reached
    UCB1TBCNT = 0x01;                       // # of Bytes in Packet

    P4SEL1 &= ~BIT6;                        // We want P4.9 = SDA
    P4SEL0 |= BIT6;
    P4SEL1 &= ~BIT7;                        // We want P4.7 = SCL
    P4SEL0 |= BIT7;

    UCB1CTLW0 &= ~UCSWRST;                  // Take eUSCI_B0 out of SW reset
    UCB1IE |= UCTXIE0;                      // Enable I2C Tx IRQ
    UCB1IE |= UCRXIE0;                      // Enable I2C Rx IRQ
    
}

int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;               // Stop watchdog timer
    i2c_init();
    PM5CTL0 &= ~LOCKLPM5;                   // Disable the GPIO power-on default high-impedance mode
                                            // to activate previously configured port settings
    __enable_interrupt();                   // Enable Maskable IRQs
    
    while(1)
    {
        //Transmit Register Address with Write Message
        UCB1CTLW0 |= UCTR;                  // Put into Tx mode
        UCB1CTLW0 |= UCTXSTT;               // Generate START condition
        
        while (UCB1CTLW0 & UCTXSTT);        // Wait until START is sent
            UCB1IFG &= ~UCSTPIFG;           // Clear STOP flag
        
        // Recieve Data from Rx
        UCB1CTLW0 &= ~UCTR;                 // Put into Rx mode
        UCB1CTLW0 |= UCTXSTT;               // Generate START condition
        
        while (UCB1CTLW0 & UCTXSTT);        // Wait until START is sent
            UCB1IFG &= ~UCSTPIFG;           // Clear STOP flag
    }
}

//----------------------------------------------------------------------
// Begin Interrupt Service Routine
//----------------------------------------------------------------------

#pragma vector=EUSCI_B1_VECTOR
__interrupt void EUSCI_B1_I2C_ISR(void){
    switch (UCB1IV)
    {
        case 0x16:                  // ID 16: RXIFG0
            data_in = UCB1RXBUF;    // Retrieve data
            break;
        case 0x18:                  // ID 18: TXIFG0
            UCB1TXBUF = 0x00;       // Send Reg Addr
            break;
        default:
            break;
    }
}
