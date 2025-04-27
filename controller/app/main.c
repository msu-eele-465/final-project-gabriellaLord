/*
 * EELE 465, Project 6
 * Gabby and Iker
 *
 * Target device: MSP430FR2355 Master
 */

//----------------------------------------------------------------------
// Headers
//----------------------------------------------------------------------
#include <msp430.h>
#include <stdbool.h>
#include <stdio.h>
#include "intrinsics.h"
#include "..\src\master_i2c.h"
#include "..\src\rgb_led.h"
#include "..\src\heartbeat.h"
//--End Headers---------------------------------------------------------

//----------------------------------------------------------------------
// Definitions
//----------------------------------------------------------------------
#define PROWDIR     P6DIR  // FORMERLY P1
#define PROWREN     P6REN
#define PROWIN      P6IN
#define PROWOUT     P6OUT
#define PCOLDIR     P5DIR   // FORMERLY P5
#define PCOLOUT     P5OUT
#define RS BIT2  // P1.2 -> RS (Register Select)
#define EN BIT3  // P1.3 -> Enable
#define D4 BIT4  // P1.4 -> Data bit 4
#define D5 BIT5  // P1.5 -> Data bit 5
#define D6 BIT6  // P1.6 -> Data bit 6
#define D7 BIT7  // P1.7 -> Data bit 7
#define COL 4
#define ROW 4
#define TABLE_SIZE 4
#define LCD_ADDR    0x38
#define LM92_ADDR   0x48
#define LED_ADDR    0x58
#define DS3231_ADDR 0x68
//--End Definitions-----------------------------------------------------

//----------------------------------------------------------------------
// Variables
//----------------------------------------------------------------------
char real_code[] = {'3','9','4','D'};
char keypad[ROW][COL] = {
    {'1', '2', '3', 'A'},
    {'4', '5', '6', 'B'},
    {'7', '8', '9', 'C'},
    {'*', '0', '#', 'D'}
};
int lockState = 3;
bool bool_unlocked = false;
volatile bool adc_ready = false;
volatile unsigned int window_size = 3;
volatile unsigned int adc_result = 0;
volatile unsigned int samples[9] = {0};
volatile unsigned int sample_index = 0;
volatile unsigned int temp_C = 0;
volatile unsigned int seconds_total = 0;
//--End Variables-------------------------------------------------------

//----------------------------------------------------------------------
// Begin Debounce
//----------------------------------------------------------------------
void debounce() {
    volatile unsigned int i;
    for (i = 20000; i > 0; i--) {}
}
//--End Debounce--------------------------------------------------------

//----------------------------------------------------------------------
// Begin Initializing Keypad Ports
//----------------------------------------------------------------------
void keypad_init(void) 
{
    // Set rows as inputs (with pull-up)
    PROWDIR &= ~0x0F;   // P1.0, P1.1, P1.2, P1.3 as inputs
    PROWREN |= 0x0F;    // Activate pull-up
    PROWOUT |= 0x0F;    // Activar pull-up in rows
    
    // Set columns as outputs
    PCOLDIR |= BIT0 | BIT1 | BIT2 | BIT3; // Set P5.0, P5.1, P5.2 y P5.3 as outputs:
    PCOLOUT &= ~(BIT0 | BIT1 | BIT2 | BIT3);  // Set down the pins P5.0, P5.1, P5.2 y P5.3:
}
//--End Initialize Keypad-----------------------------------------------

//----------------------------------------------------------------------
// Begin Initializing ADC
//----------------------------------------------------------------------
void adc_init(void)
{
    // Configure GPIO
    P1SEL0 |= BIT3;             // P1.3 = A3 (analog)
    P1DIR &= ~BIT3;             // Input

    // Configure ADC
    ADCCTL0 = ADCSHT_2 | ADCON;
    ADCCTL1 = ADCSHP | ADCSSEL_1;    // Use sampling timer, ACLK
    ADCCTL2 = ADCRES_2;              // 12-bit
    ADCMCTL0 = ADCINCH_3;            // Channel A3

    // Timer setup (ACLK = 32768 Hz → 0.5s = 16384)
    TB1CTL = TBSSEL__ACLK | MC__UP | TBCLR;
    TB1CCR0 = 16384;
    TB1CCTL0 = CCIE;
}
//--End Initialize ADC--------------------------------------------------

void adc_off(void)
{
    TB1CCTL0 &= ~CCIE;     // Disable Timer1_B0 interrupt
    TB1CTL = 0;            // Stop Timer1
    ADCCTL0 &= ~ADCENC;    // Disable ADC conversion
    ADCCTL0 &= ~ADCON;     // Turn off ADC
}

void send_int_3_digit(char mode, int input)
{
    char buffer[4];
    int j;

    buffer[0] = (input / 100) % 10 + '0';
    buffer[1] = (input / 10) % 10 + '0';
    buffer[2] = input % 10 + '0';
    buffer[3] = '\0';

    master_i2c_send(mode, LCD_ADDR);
    for (j = 0; buffer[j] != '\0'; j++) {
        master_i2c_send(buffer[j], LCD_ADDR);
    }
}


//----------------------------------------------------------------------
// Begin Moving Average
//----------------------------------------------------------------------
void adc_moving_average(void) {
    unsigned long sum = 0;
    int i;
    for (i = 0; i < window_size; i++) {
        sum += samples[i];
    }
    unsigned int avg_adc = sum / window_size;

    // Convert ADC to voltage
    float voltage = (avg_adc * 3.3f) / 4095.0f;

    // LM19 sensor voltage to temperature conversion
    // Vtemp = -11.69 mV/°C * T + 1.8639 V → Solve for T
    temp_C = (unsigned int)((1.8639f - voltage) / 0.001169f);
    send_int_3_digit('Y', temp_C);
}
//--End Movind Average--------------------------------------------------

void system_lock(void)
{
    rgb_led_continue(3);            // Set LED to red when 'D' is pressed
    master_i2c_send('D', LED_ADDR);    // led slave
    master_i2c_send('D', LCD_ADDR);    // lcd slave
    bool_unlocked = false;
    adc_off();
    P3OUT &= ~(BIT0 | BIT1); // Force both P3.0 and P3.1 low initially
}

//----------------------------------------------------------------------
// Begin Unlocking Routine
//----------------------------------------------------------------------
char keypad_unlocking(void)
{
    int row, col;

    // Go through 4 columns
    for (col = 0; col < 4; col++) {
        // Put column down (active)
        PCOLOUT &= ~(1 << col);   // P2.0, P2.1, P2.2, P2.4
         __delay_cycles(1000);  // Little stop to stabilize the signal

        // Go through 4 rows
        for (row = 0; row < 4; row++) {
            if ((PROWIN & (1 << row)) == 0) {  // We detect that the row is low
                debounce();  // Wait to filter the bouncing
                if ((PROWIN & (1 << row)) == 0) {  // Confirm that the key is pressed
                    rgb_led_continue(0);                // Set LED to yellow
                    char key = keypad[row][col];
                    // Wait until the key is released to avoid multiple readings 
                    while ((PROWIN & (1 << row)) == 0);
                    // Deactivate the column before returning
                    PCOLOUT |= (1 << col);                    
                    return key;
                }
            }
        }
        // Put the column high (deactivated)
        PCOLOUT |= (1 << col);
    }

    return 0; // No key pressed
}
//--End Unlocking-------------------------------------------------------

//----------------------------------------------------------------------
// Begin Unlocked Routine
//----------------------------------------------------------------------
char keypad_unlocked(void)
{
    char key_unlocked = '\0';

    // Continuously poll until 'D' is pressed
    while (key_unlocked != 'D') {
        int row, col;

        for (col = 0; col < 4; col++) {
            // Activate column
            PCOLOUT &= ~(1 << col);
            __delay_cycles(1000);

            // Check all rows
            for (row = 0; row < 4; row++) {
                if ((PROWIN & (1 << row)) == 0) {
                    debounce();
                    if ((PROWIN & (1 << row)) == 0) {
                        key_unlocked = keypad[row][col];
                        if (key_unlocked != 'D') {
                            switch(key_unlocked)
                            {
                                case 'A':
                                    P3OUT &= ~BIT1;  // P3.1 = Low
                                    debounce();
                                    P3OUT |= BIT0;   // P3.0 = High
                                    break;

                                case 'B':
                                    P3OUT &= ~BIT0;  // P3.0 = Low
                                    debounce();
                                    P3OUT |= BIT1;   // P3.1 = High
                                    break;
                            }
                            master_i2c_send(key_unlocked, LED_ADDR);   // led slave
                            master_i2c_send(key_unlocked, LCD_ADDR);   // lcd slave
                        }
                        // Wait for key release
                        while ((PROWIN & (1 << row)) == 0);
                        PCOLOUT |= (1 << col);
                        
                        // switch(key_unlocked)
                            // case 'A' match

                        if (key_unlocked == 'D') { // or time = 300
                            system_lock();
                            return key_unlocked;
                        }
                    }
                }
            }
            // Deactivate column
            PCOLOUT |= (1 << col);
        }
        if (bool_unlocked)
        {
            //master_i2c_receive(LM, int reg)
            master_i2c_receive(DS3231_ADDR, 0x00);             // Seconds
            seconds_total = return_time();
            master_i2c_receive(DS3231_ADDR, 0x01);             // Minutes
            seconds_total = return_time() * 60 + seconds_total;
            send_int_3_digit('S', seconds_total);
            if (adc_ready)
            {
                adc_ready = false;
                adc_moving_average();                   // This does the I2C and temperature calc
            }
;        }

    }
    return key_unlocked;
}
//----------------------------------------------------------------------
// Begin Main
//----------------------------------------------------------------------
void main(void)
{   
    int counter, i, equal;
    char introduced_password[TABLE_SIZE], key; 
    P3DIR |= BIT0 | BIT1;    // Set as output
    P3OUT &= ~(BIT0 | BIT1); // Force both P3.0 and P3.1 low initially


    keypad_init();
    heartbeat_init();
    rgb_led_init();
    master_i2c_init();
      
    while(true)
    {
        counter = 0;
        key = 0;

        while (counter < TABLE_SIZE)
        {
            key = keypad_unlocking();
            if(key!=0)
            {
                introduced_password [counter] = key;
                counter++;
            }        
        }
        //Compare the introduced code with the real code   
        equal = 1;   
        for (i = 0; i < TABLE_SIZE; i++) {
            if (introduced_password[i] != real_code[i]) 
            {
                equal = 0;
                break;
            }
        }
        // Verify the code
        if (equal==1) 
        {
            printf("Correct Code!\n");
            adc_init();
            counter = 0;
            rgb_led_continue(1);            // Set LED to blue
            for (i = 0; i < TABLE_SIZE; i++) 
            {
                introduced_password[i] = 0;        
            }
            bool_unlocked = true;
            master_i2c_send('Z', LCD_ADDR);            // lcd slave
            keypad_unlocked();  // This now handles polling until 'D' is pressed
        } 
        else 
        {
            printf("Incorrect code. Try again.\n");
            counter = 0;  // Reinitiate counter to try again
            rgb_led_continue(3);            // Set LED to red
            master_i2c_send('\0', LED_ADDR);
            master_i2c_send('\0', LCD_ADDR);
            //led_patterns('\0');
            for (i = 0; i < TABLE_SIZE; i++) 
            {
                introduced_password[i] = 0;        
            }
        }    
    }
}
//--End Main------------------------------------------------------------

//----------------------------------------------------------------------
// Begin Interrupt Service Routines
//----------------------------------------------------------------------
#pragma vector = TIMER1_B0_VECTOR
__interrupt void ISR_TB1_CCR0(void)
{
    ADCCTL0 |= ADCENC | ADCSC;             // Start ADC conversion
    while (ADCCTL1 & ADCBUSY);             // Wait for result
    adc_result = ADCMEM0;

    samples[sample_index++] = adc_result;
    if (sample_index >= window_size)
    {
        sample_index = 0;
        adc_ready = true;   // Tell main loop to process this
    }

    TB1CCTL0 &= ~CCIFG;                    // Clear interrupt flag
}
//-- End Interrupt Service Routines ------------------------------------
