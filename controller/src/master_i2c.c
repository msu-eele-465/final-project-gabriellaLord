#include <msp430.h>
#include <stdbool.h>
#include <stdio.h>
#include "intrinsics.h"
#include "master_i2c.h"

char packet;                  // Data to be sent using I2C
int time_in = 0;              // Data received via I2C
volatile bool i2c_done = false; // Flag indicating I2C transmission/reception is done

//------------------------------------------------------------------------------
// Master I2C Initialization (TX + RX)
//------------------------------------------------------------------------------
/**
 * @brief Initialize eUSCI_B1 for I2C master communication at 100 kHz
 */
void master_i2c_init(void)
{
    WDTCTL = WDTPW | WDTHOLD;               // Stop watchdog timer

    UCB1CTLW0 |= UCSWRST;                   // Put eUSCI_B1 into software reset
    UCB1CTLW0 |= UCSSEL_3;                  // Use SMCLK (1MHz)
    UCB1BRW = 10;                           // Set baud rate for ~100kHz
    UCB1CTLW0 |= UCMODE_3 | UCMST;          // I2C mode, master mode
    UCB1CTLW1 |= UCASTP_2;                  // Auto STOP after byte count
    UCB1TBCNT = 1;                          // One byte to transmit or receive

    P4SEL1 &= ~(BIT6 | BIT7);               // Configure SDA/SCL function
    P4SEL0 |=  (BIT6 | BIT7);               // SDA=P4.6, SCL=P4.7

    PM5CTL0 &= ~LOCKLPM5;                   // Enable GPIO functionality
    UCB1CTLW0 &= ~UCSWRST;                  // Take eUSCI_B1 out of reset

    UCB1IE |= UCTXIE0 | UCRXIE0;            // Enable TX and RX interrupts
    __enable_interrupt();                   // Enable global interrupts
}

//------------------------------------------------------------------------------
// Master I2C Send (1-byte)
//------------------------------------------------------------------------------
/**
 * @brief Send a single byte to a slave device via I2C
 * @param input Byte to send
 * @param address 7-bit I2C address of the slave
 */
void master_i2c_send(char input, int address)
{
    UCB1I2CSA = address;         // Set slave address
    packet = input;              // Load data to send
    UCB1CTLW0 |= UCTR;           // Set to transmit mode
    UCB1CTLW0 |= UCTXSTT;        // Generate START condition
    while (UCB1CTLW0 & UCTXSTT); // Wait for START to complete
    UCB1IFG &= ~UCSTPIFG;        // Clear STOP flag
}

//------------------------------------------------------------------------------
// Master I2C Receive (1-byte)
//------------------------------------------------------------------------------
/**
 * @brief Receive a single byte from a slave register via I2C
 * @param address 7-bit I2C address of the slave
 * @param reg Register address to read from
 */
void master_i2c_receive(int address, int reg)
{
    UCB1I2CSA = address;         // Set slave address
    UCB1TBCNT = 1;               // Only one byte
    packet = reg;                // Register to access
    i2c_done = false;

    // Step 1: Write register address
    UCB1CTLW0 |= UCTR;           // Transmit mode
    UCB1CTLW0 |= UCTXSTT;        // Generate START
    while (!i2c_done);           // Wait for TX complete
    i2c_done = false;

    while (UCB1CTLW0 & UCTXSTT); // Wait until START sent
    UCB1IFG &= ~UCSTPIFG;        // Clear STOP flag

    // Step 2: Switch to read mode and read value
    UCB1CTLW0 &= ~UCTR;          // Receive mode
    UCB1CTLW0 |= UCTXSTT;        // Generate START
    while (UCB1CTLW0 & UCTXSTT); // Wait for START
    UCB1IFG &= ~UCSTPIFG;        // Clear STOP flag
    while (!i2c_done);           // Wait for RX complete
}

//------------------------------------------------------------------------------
// Time Conversion
//------------------------------------------------------------------------------
/**
 * @brief Convert BCD time value to decimal
 * @return Time in decimal format
 */
int return_time(void)
{
    return ((time_in >> 4) * 10) + (time_in & 0x0F);
}

//------------------------------------------------------------------------------
// I2C ISR (Handles TX and RX)
//------------------------------------------------------------------------------
/**
 * @brief ISR for handling I2C TX, RX, STOP, and NACK interrupts
 */
#pragma vector=EUSCI_B1_VECTOR
__interrupt void EUSCI_B1_I2C_ISR(void)
{
    switch (UCB1IV)
    {
        case 0x16: // RXIFG0
            time_in = UCB1RXBUF;    // Read received byte
            i2c_done = true;
            break;

        case 0x18: // TXIFG0
            UCB1TXBUF = packet;     // Send packet
            i2c_done = true;
            break;

        case 0x22: // STOP condition
            break;

        case 0x02: // NACK received
            UCB1CTLW0 |= UCTXSTP;   // Send STOP
            break;

        default:
            break;
    }
}
