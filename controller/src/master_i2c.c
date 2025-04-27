#include <msp430.h>
#include <stdbool.h>
#include <stdio.h>
#include "intrinsics.h"
#include "master_i2c.h"

char packet;               // Data to be sent using I2C
int time_in = 0;           // Data received via I2C
volatile bool i2c_done = false;

//----------------------------------------------------------------------
// Master I2C Initialization (TX + RX)
//----------------------------------------------------------------------
void master_i2c_init(void)
{
    WDTCTL = WDTPW | WDTHOLD;               // Stop watchdog timer
    
    UCB1CTLW0 |= UCSWRST;                   // Put eUSCI_B1 into software reset

    UCB1CTLW0 |= UCSSEL_3;                  // Use SMCLK (1MHz)
    UCB1BRW = 10;                           // 100kHz I2C clock

    UCB1CTLW0 |= UCMODE_3 | UCMST;          // I2C mode, Master mode

    UCB1CTLW1 |= UCASTP_2;                  // Auto STOP after byte count
    UCB1TBCNT = 1;                          // One byte to transmit or receive

    P4SEL1 &= ~(BIT6 | BIT7);               // SDA/SCL select
    P4SEL0 |=  (BIT6 | BIT7);               // SDA=P4.6, SCL=P4.7

    PM5CTL0 &= ~LOCKLPM5;                   // Activate port settings

    UCB1CTLW0 &= ~UCSWRST;                  // Take eUSCI_B1 out of reset

    UCB1IE |= UCTXIE0 | UCRXIE0;            // Enable TX and RX interrupts

    __enable_interrupt();                   // Enable global interrupts

    // master_i2c_send(0, ds3231)
}

//----------------------------------------------------------------------
// Master I2C Send (1-byte)
//----------------------------------------------------------------------
void master_i2c_send(char input, int address)
{
    UCB1I2CSA = address;      // Set slave address
    packet = input;           // Set data to send
    UCB1CTLW0 |= UCTR;        // Transmit mode
    UCB1CTLW0 |= UCTXSTT;     // Generate START
    while (UCB1CTLW0 & UCTXSTT);  // Wait for START complete
    UCB1IFG &= ~UCSTPIFG;     // Clear STOP flag
}

//----------------------------------------------------------------------
// Master I2C Receive (1-byte)
//----------------------------------------------------------------------
void master_i2c_receive(int address, int reg)
{
    UCB1I2CSA = address;
    UCB1TBCNT = 1;
    packet = reg;
    i2c_done = false;

    // Step 1: Write register address
    UCB1CTLW0 |= UCTR;        // TX mode
    UCB1CTLW0 |= UCTXSTT;     // START condition

    while (!i2c_done);        // Wait for TXIFG0 (reg sent)
    i2c_done = false;

    //  Wait for STOP condition to complete before issuing repeated START
    while (UCB1CTLW0 & UCTXSTT);        // Wait until START is sent
        UCB1IFG &= ~UCSTPIFG;           // Clear STOP flag
    
    // Recieve Data from Rx
    UCB1CTLW0 &= ~UCTR;                 // Put into Rx mode
    UCB1CTLW0 |= UCTXSTT;               // Generate START condition
    
    while (UCB1CTLW0 & UCTXSTT);        // Wait until START is sent
        UCB1IFG &= ~UCSTPIFG;           // Clear STOP flag

        while (!i2c_done);                // Wait for RX
}


int return_time(void)                // Returns time (seconds or minutes) to decimal
{
    return ((time_in >> 4) * 10) + (time_in & 0x0F);
}


//----------------------------------------------------------------------
// I2C ISR (Handles TX and RX)
//----------------------------------------------------------------------
#pragma vector=EUSCI_B1_VECTOR
__interrupt void EUSCI_B1_I2C_ISR(void)
{
    switch (UCB1IV)
    {
        case 0x16:                      // RXIFG0
            // if (address = ds3231)
                // if (UCB1RXBUF = 6)
                    // time_in = 0;
                    // master_i2c_send(0, ds3231);
                // else
                    time_in = UCB1RXBUF;
            // if address = lm92
                // temp_in = UCB1RXBUF;
            i2c_done = true;
            break;

        case 0x18:                      // TXIFG0
            UCB1TXBUF = packet;        // Send register address
            i2c_done = true;
            break;

        case 0x22:                      // STOP condition received
            break;

        case 0x02:                      // NACK received
            UCB1CTLW0 |= UCTXSTP;      // Send STOP
            break;

        default:
            break;
    }
}

