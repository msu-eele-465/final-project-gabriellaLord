/*
 * EELE 465, Project 5
 * Gabby and Iker
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
// Puerto 2
#define RS BIT0     // P2.0
#define EN BIT6     // P2.6

// Puerto 1
#define D4  BIT4     // P1.4
#define D5 BIT5     // P1.5
#define D6 BIT6     // P1.6
#define D7 BIT7     // P1.7
#define SLAVE_ADDR  0x38                    // Slave I2C Address
//--End Definitions-----------------------------------------------------

//----------------------------------------------------------------------
// Variables
//----------------------------------------------------------------------
volatile unsigned char receivedData = 0;    // Recieved data
char key_unlocked;
char mode = '\0';
char prev_mode = '\0';
char new_window_size = '\0';
char pattern_cur = '\0';
int length_time = 0;
int length_adc = 0;
bool in_time_mode = false;
bool in_temp_adc_mode = false;
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

    // Configure USCI_B0 as I2C Slave
    UCB0CTLW0 |= UCSWRST;               // Put eUSCI_B0 into software reset
    UCB0CTLW0 |= UCMODE_3;              // Select I2C slave mode
    UCB0I2COA0 = SLAVE_ADDR + UCOAEN;   // Set and enable first own address
    UCB0CTLW0 |= UCTXACK;               // Send ACKs

    PM5CTL0 &= ~LOCKLPM5;               // Disable low-power inhibit mode

    UCB0CTLW0 &= ~UCSWRST;              // Pull eUSCI_B0 out of software reset
    UCB0IE |= UCSTTIE + UCRXIE;         // Enable Start and RX interrupts

    __enable_interrupt();               // Enable Maskable IRQs
}
//--End I2C Init--------------------------------------------------------

//----------------------------------------------------------------------
// Begin Send Commands
//----------------------------------------------------------------------
void pulseEnable() {
    P2OUT |= EN;             // Establecer Enable en 1
    __delay_cycles(1000);    // Retardo
    P2OUT &= ~EN;            // Establecer Enable en 0
    __delay_cycles(1000);    // Retardo
}

void sendNibble(unsigned char nibble) {
    P1OUT &= ~(D4 | D5 | D6 | D7);  // Limpiar los bits de datos
    P1OUT |= ((nibble & 0x0F) << 4);  // Cargar el nibble en los bits correspondientes (P1.4 a P1.7)
    pulseEnable();  // Pulsar Enable para enviar datos
}

void send_data(unsigned char data) {
    P2OUT |= RS;    // Modo datos
    sendNibble(data >> 4);  // Enviar los 4 bits más significativos
    sendNibble(data & 0x0F);  // Enviar los 4 bits menos significativos (corregido)
    __delay_cycles(4000); // Retardo para procesar los datos
}

void send_command(unsigned char cmd) {
    P2OUT &= ~RS;   // Modo comando
    sendNibble(cmd >> 4);  // Enviar los 4 bits más significativos
    sendNibble(cmd);  // Enviar los 4 bits menos significativos
    __delay_cycles(4000); // Retardo para asegurarse de que el comando se procese
}

void lcdSetCursor(unsigned char position) {
    send_command(0x80 | position);  // Establecer la dirección del cursor en la DDRAM
}
//--End Send Commands---------------------------------------------------

//----------------------------------------------------------------------
// Begine LCD Init
//----------------------------------------------------------------------
void lcdInit() {
    // Configurar pines como salida
    P1DIR |= D4 | D5 | D6 | D7;
    P2DIR |= RS | EN;

    // Limpiar salidas
    P1OUT &= ~(D4 | D5 | D6 | D7);
    P2OUT &= ~(RS | EN);
    __delay_cycles(50000);  // Retardo de inicio
    sendNibble(0x03);  // Inicialización del LCD
    __delay_cycles(5000);  // Retardo
    sendNibble(0x03);  // Repetir la inicialización
    __delay_cycles(200);  // Retardo
    sendNibble(0x03);  // Repetir la inicialización
    sendNibble(0x02);  // Establecer modo de 4 bits

    send_command(0x28);  // Configurar LCD: 4 bits, 2 líneas, 5x8
    send_command(0x0C);  // Encender display, apagar cursor
    send_command(0x06);  // Modo de escritura automática
    send_command(0x01);  // Limpiar la pantalla
    __delay_cycles(2000); // Esperar para limpiar la pantalla
}
//--End LCD Init--------------------------------------------------------

//----------------------------------------------------------------------
// Begin Print Commands
//----------------------------------------------------------------------
void lcd_print(const char* str, unsigned char startPos) {
    lcdSetCursor(startPos);
    while (*str) {
        send_data(*str++);
        startPos++;
        if (startPos == 0x10) startPos = 0x40;  // Salto automático a segunda línea
    }
}

void display_time(char input)
{
    char string[2];
    string[0] = input;
    string[1] = '\0';
    switch (length_time) {
        case 0:
            length_time++;
            break;
        case 1:
            lcd_print(string, 0x42);
            length_time++;
            break;
        case 2:
            lcd_print(string, 0x43);
            length_time++;
            break;
        case 3:
            lcd_print(string, 0x44);
            in_time_mode = false;
            mode = 'A';
            length_time = 0;
            break;
        default:
            in_time_mode = false;
            length_time = 0;
            break;
    }
}

void display_temp_adc(char input)
{
    char string[2];
    string[0] = input;
    string[1] = '\0';
    switch (length_adc) {
        case 0:
            length_adc++;
            break;
        case 1:
            lcd_print(string, 0x0A);
            length_adc++;
            break;
        case 2:
            lcd_print(string, 0x0B);
            length_adc++;
            break;
        case 3:
            lcd_print(string, 0x0D);
            in_temp_adc_mode = false;
            mode = 'A';
            length_adc = 0;
            break;
        default:
            in_temp_adc_mode = false;
            length_adc = 0;
            break;
    }
}

void display_output(char input)
{
    switch (input)
    {
        case 'A':           // heat mode
            lcd_print("HEAT ", 0x00);
            mode = 'A';
            break;
        case 'B':           // cool mode
            lcd_print("COOL ", 0x00);
            mode = 'B';
            break;
        case 'C':           // match mode
            lcd_print("MATCH", 0x00);
            mode = 'C';
            break;
        case 'D':           // off
            send_command(0x01);
            lcd_print("OFF  ", 0x00);
            break;
        case 'S':           // display seconds
            in_time_mode = true;
            //prev_mode = mode;
            mode = 'S';
            length_time = 0;
            break;
        case 'X':           // display I2C temperature
            break;
        case 'Y':           // display AD2 temperature
            in_temp_adc_mode = true;
            //prev_mode = mode;
            mode = 'Y';
            length_adc = 0; // reset position for temperature digits
            break;
        case 'Z':           // just unlocked
            send_command(0x01);
            __delay_cycles(2000);
            lcd_print("OFF     A:xx.x", 0x00);
            lcdSetCursor(0x0E);             // Move to where the degree symbol goes
            send_data(0xDF);                // Send the built-in degree symbol
            lcd_print("C", 0x0F);           // Continue with 'C'
            lcd_print("3 xxxs  P:xx.x", 0x40);
            lcdSetCursor(0x4E);             // Move to where the degree symbol goes
            send_data(0xDF);                // Send the built-in degree symbol
            lcd_print("C", 0x4F);           // Continue with 'C'
            mode = 'A';
            break;
    }

    if (mode == 'S')
    {
        display_time(input);
    }

    
    if (mode == 'Y')
    {
        display_temp_adc(input);
    }
}
//--End Print Commands--------------------------------------------------

//----------------------------------------------------------------------
// Begin Main
//----------------------------------------------------------------------
int main(void) {
    WDTCTL = WDTPW | WDTHOLD;  // Detener el watchdog
    PM5CTL0 &= ~LOCKLPM5;
    lcdInit();  // Inicializar el LCD
    I2C_Slave_Init();                   // Initialize the slave for I2C
    __bis_SR_register(LPM0_bits + GIE); // Enter LPM0, enable interrupts
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
        case USCI_I2C_UCRXIFG0:         // Receive Interrupt
            display_output(UCB0RXBUF);
            break;
        default: 
            break;
    }
}
//-- End Interrupt Service Routines --------------------------------------------
