/*
 * EELE 465, Final Project
 * Gabby Lord
 *
 * Target device: MSP430FR2310 Slave
 */

//----------------------------------------------------------------------
// Headers
//----------------------------------------------------------------------
#include "intrinsics.h"
#include <msp430.h>
#include <stdbool.h>
//--End Headers---------------------------------------------------------

//----------------------------------------------------------------------
// Definitions
//----------------------------------------------------------------------
// Port 2
#define RS BIT0     // P2.0 - Register Select
#define EN BIT6     // P2.6 - Enable

// Port 1
#define D4  BIT4    // P1.4 - LCD Data 4
#define D5 BIT5     // P1.5 - LCD Data 5
#define D6 BIT6     // P1.6 - LCD Data 6
#define D7 BIT7     // P1.7 - LCD Data 7
#define SLAVE_ADDR  0x38   // Slave I2C Address
//--End Definitions-----------------------------------------------------

//----------------------------------------------------------------------
// Variables
//----------------------------------------------------------------------
volatile unsigned char receivedData = 0;    // Received data from I2C
char key_unlocked;
char mode = '\0';
char prev_mode = '\0';
char new_window_size = '\0';
char pattern_cur = '\0';
int length_time = 0;
int length_adc = 0;
bool in_time_mode = false;
bool in_temp_adc_mode = false;

// Custom characters for LCD
// Arrow pointing North (↑)
unsigned char arrow_north[8] = {
    0b00000,
    0b00000,
    0b00100,
    0b01110,
    0b10101,
    0b00100,
    0b00100,
    0b00000
};

// Arrow pointing Northeast (↗)
unsigned char arrow_ne[8] = {
    0b00000,
    0b00000,
    0b00111,
    0b00011,
    0b00101,
    0b01000,
    0b10000,
    0b00000
};

// Arrow pointing Southeast (↘)
unsigned char arrow_se[8] = {
    0b00000,
    0b00000,
    0b10000,
    0b01000,
    0b00101,
    0b00011,
    0b00111,
    0b00000
};

// Arrow pointing South (↓)
unsigned char arrow_south[8] = {
    0b00000,
    0b00000,
    0b00100,
    0b00100,
    0b10101,
    0b01110,
    0b00100,
    0b00000
};

// Arrow pointing Southwest (↙)
unsigned char arrow_sw[8] = {
    0b00000,
    0b00000,
    0b00001,
    0b00010,
    0b10100,
    0b11000,
    0b11100,
    0b00000
};

// Arrow pointing Northwest (↖)
unsigned char arrow_nw[8] = {
    0b00000,
    0b00000,
    0b11100,
    0b11000,
    0b10100,
    0b00010,
    0b00001,
    0b00000
};

// Angle symbol for LCD
unsigned char angle[8] = {
    0b00000,
    0b00000,
    0b00001,
    0b00010,
    0b00100,
    0b01000,
    0b11111,
    0b00000
};
//--End Variables-------------------------------------------------------

//----------------------------------------------------------------------
// Begin I2C Init
//----------------------------------------------------------------------
void I2C_Slave_Init(void)
{
    WDTCTL = WDTPW | WDTHOLD;  // Stop Watchdog Timer

    // Configure P1.2 as SDA and P1.3 as SCL
    P1SEL1 &= ~(BIT2 | BIT3);
    P1SEL0 |= BIT2 | BIT3;

    // Configure USCI_B0 for I2C Slave mode
    UCB0CTLW0 |= UCSWRST;               // Put eUSCI_B0 into software reset
    UCB0CTLW0 |= UCMODE_3;              // I2C mode, slave
    UCB0I2COA0 = SLAVE_ADDR + UCOAEN;   // Own address enable
    UCB0CTLW0 |= UCTXACK;               // Automatically send ACK

    PM5CTL0 &= ~LOCKLPM5;               // Disable GPIO high-impedance mode

    UCB0CTLW0 &= ~UCSWRST;              // Release eUSCI_B0 from reset
    UCB0IE |= UCSTTIE + UCRXIE;         // Enable Start and RX interrupts

    __enable_interrupt();               // Enable global interrupts
}
//--End I2C Init---------------------------------------------------------

//----------------------------------------------------------------------
// Begin Send Commands
//----------------------------------------------------------------------
void pulseEnable() {
    P2OUT |= EN;             // Set Enable high
    __delay_cycles(1000);    // Small delay
    P2OUT &= ~EN;            // Set Enable low
    __delay_cycles(1000);    // Small delay
}

void sendNibble(unsigned char nibble) {
    P1OUT &= ~(D4 | D5 | D6 | D7);  // Clear data lines
    P1OUT |= ((nibble & 0x0F) << 4);  // Output nibble
    pulseEnable();  // Pulse Enable to latch data
}

void send_data(unsigned char data) {
    P2OUT |= RS;    // Data mode
    sendNibble(data >> 4);  // Send high nibble
    sendNibble(data & 0x0F);  // Send low nibble
    __delay_cycles(4000); // Delay for data write
}

void send_command(unsigned char cmd) {
    P2OUT &= ~RS;   // Command mode
    sendNibble(cmd >> 4);  // Send high nibble
    sendNibble(cmd);  // Send low nibble
    __delay_cycles(4000); // Delay for command processing
}

void lcdSetCursor(unsigned char position) {
    send_command(0x80 | position);  // Set cursor to DDRAM address
}

void lcd_print(const char* str, unsigned char startPos) {
    lcdSetCursor(startPos);
    while (*str) {
        send_data(*str++);
        startPos++;
        if (startPos == 0x10) startPos = 0x40;  // Jump to second line after end of first
    }
}

void lcd_create_char(unsigned char location, unsigned char charmap[]) {
    location &= 0x07; // Only locations 0–7 are valid
    send_command(0x40 | (location << 3)); // Set CGRAM address
    int i;
    for (i = 0; i < 8; i++) {
        send_data(charmap[i]);
    }
}
//--End Send Commands---------------------------------------------------

//----------------------------------------------------------------------
// Begin LCD Init
//----------------------------------------------------------------------
void lcdInit() {
    // Set pins as output
    P1DIR |= D4 | D5 | D6 | D7;
    P2DIR |= RS | EN;

    // Clear outputs
    P1OUT &= ~(D4 | D5 | D6 | D7);
    P2OUT &= ~(RS | EN);
    __delay_cycles(50000);  // Initial delay
    sendNibble(0x03);  // LCD initialization sequence
    __delay_cycles(5000);
    sendNibble(0x03);
    __delay_cycles(200);
    sendNibble(0x03);
    sendNibble(0x02);  // Set 4-bit mode

    send_command(0x28);  // 4-bit, 2-line, 5x8 font
    send_command(0x0C);  // Display ON, cursor OFF
    send_command(0x06);  // Auto-increment cursor
    send_command(0x01);  // Clear display
    __delay_cycles(2000); // Wait for clear

    // Load custom characters
    lcd_create_char(0, arrow_north);
    lcd_create_char(1, arrow_ne);
    lcd_create_char(2, arrow_se);
    lcd_create_char(3, arrow_south);
    lcd_create_char(4, arrow_sw);
    lcd_create_char(5, arrow_nw);
    lcd_create_char(6, angle);

    // Initial display text
    lcd_print(" xxx.x", 0x00);      // Angle in degrees
    lcd_print("TDOA:xxxx s", 0x40);  // +/- time distance of arrival

    // Place angle and degree symbols
    lcdSetCursor(0x00);
    send_data(0x06);        // Custom angle symbol
    lcdSetCursor(0x06);
    send_data(0xDF);        // Built-in degree symbol
    lcdSetCursor(0x49);
    send_data(0xE4);        // Built-in mu (μ) symbol
}
//--End LCD Init--------------------------------------------------------

void display_tdoa(char input)
{
    char string[2];
    string[0] = input;
    string[1] = '\0';
    switch (length_adc) {
        case 0:
            length_adc++;
            break;
        case 1:
            lcd_print(string, 0x45);
            length_adc++;
            break;
        case 2:
            lcd_print(string, 0x46);
            length_adc++;
            break;
        case 3:
            lcd_print(string, 0x47);
            length_adc++;
            break;
        case 4:
            lcd_print(string, 0x48);
            in_temp_adc_mode = false;
            mode = '\0';
            length_adc = 0;
            break;
        default:
            in_temp_adc_mode = false;
            length_adc = 0;
            break;
    }
}

void display_angle(char input)
{
    char string[2];
    string[0] = input;
    string[1] = '\0';
    switch (length_adc) {
        case 0:
            length_adc++;
            break;
        case 1:
            lcd_print(string, 0x01);
            length_adc++;
            break;
        case 2:
            lcd_print(string, 0x02);
            length_adc++;
            break;
        case 3:
            lcd_print(string, 0x03);
            length_adc++;
            break;
        case 4:
            lcd_print(string, 0x05);
            in_temp_adc_mode = false;
            mode = '\0';
            length_adc = 0;
            break;
        default:
            in_temp_adc_mode = false;
            length_adc = 0;
            break;
    }
}

//----------------------------------------------------------------------
// Begin Print Commands
//----------------------------------------------------------------------
void display_output(char input)
{
    switch (input)
    {
        case '1':           // North
            lcdSetCursor(0x4F);
            send_data(0x00);
            break;
        case '2':           // Northeast
            lcdSetCursor(0x4F);
            send_data(0x01);
            break;
        case '3':           // East
            lcdSetCursor(0x4F);
            send_data(0x7E); // Built-in right arrow
            break;
        case '4':           // Southeast
            lcdSetCursor(0x4F);
            send_data(0x02);
            break;
        case '5':           // South
            lcdSetCursor(0x4F);
            send_data(0x03);
            break;
        case '6':           // Southwest
            lcdSetCursor(0x4F);
            send_data(0x04);
            break;
        case '7':           // West
            lcdSetCursor(0x4F);
            send_data(0x7F); // Built-in left arrow
            break;
        case '8':           // Northwest
            lcdSetCursor(0x4F);
            send_data(0x05);
            break;
        case 'A':
            mode = 'A';
            break;
        case 'B':
            mode = 'B';
            break;
        default:
            break;
    }

    if (mode == 'A')
        display_angle(input);
    if (mode == 'B')
        display_tdoa(input);
}
//--End Print Commands--------------------------------------------------

//----------------------------------------------------------------------
// Begin Main
//----------------------------------------------------------------------
int main(void) {
    WDTCTL = WDTPW | WDTHOLD;  // Stop watchdog timer
    PM5CTL0 &= ~LOCKLPM5;      // Disable GPIO high-impedance mode
    lcdInit();                 // Initialize LCD
    I2C_Slave_Init();           // Initialize I2C slave
    __bis_SR_register(LPM0_bits + GIE); // Enter Low Power Mode 0 with interrupts enabled
    return 0;
}
//--End Main------------------------------------------------------------

//----------------------------------------------------------------------
// Begin Interrupt Service Routines
//----------------------------------------------------------------------
// I2C ISR
#pragma vector = USCI_B0_VECTOR
__interrupt void USCI_B0_ISR(void)
{
    switch (__even_in_range(UCB0IV, USCI_I2C_UCTXIFG0))
    {
        case USCI_I2C_UCRXIFG0:         // Receive interrupt
            display_output(UCB0RXBUF);  // Display received data
            break;
        default: 
            break;
    }
}
//-- End Interrupt Service Routines --------------------------------------------
