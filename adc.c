
#include <p32xxxx.h>
#include <plib.h>

#pragma config FNOSC = FRCPLL       // Internal Fast RC oscillator (8 MHz) w/ PLL
#pragma config FPLLIDIV = DIV_2     // Divide FRC before PLL (now 4 MHz)
#pragma config FPLLMUL = MUL_20     // PLL Multiply (now 80 MHz)
#pragma config FPLLODIV = DIV_2     // Divide After PLL (now 40 MHz)
#pragma config FWDTEN = OFF         // Watchdog Timer Disabled
#pragma config ICESEL = ICS_PGx1    // ICE/ICD Comm Channel Select
#pragma config JTAGEN = OFF         // Disable JTAG
#pragma config FSOSCEN = OFF        // Disable Secondary Oscillator

#define SYSCLK 40000000L

void setupStatusLED()
{
    TRISAbits.TRISA0 = 0;
    TRISAbits.TRISA1 = 0;
}

void enableStatusLED()
{
    LATAbits.LATA0 = 1;
    LATAbits.LATA1 = 0;
}

void disableStatusLED()
{
    LATAbits.LATA0 = 0;
    LATAbits.LATA1 = 1;
}

void delay(int t)
{
    while( t--){
        TMR1 = 0;                   // reset timer 1
        while( TMR1 < SYSCLK/1000); // delay 1ms
    }
}

void adcConfigureManual(){
    AD1CON1CLR = 0x8000;    // disable ADC before configuration

    AD1CON1 = 0x00E0;       // internal counter ends sampling and starts conversion (auto-convert), manual sample
    AD1CON2 = 0;            // AD1CON2<15:13> set voltage reference to pins AVSS/AVDD
    AD1CON3 = 0x0f01;       // TAD = 4*TPB, acquisition time = 15*TAD
}

int analogRead(char analogPIN){
    AD1CHS = analogPIN << 16;       // AD1CHS<16:19> controls which analog pin goes to the ADC

    AD1CON1bits.SAMP = 1;           // Begin sampling
    while( AD1CON1bits.SAMP );      // wait until acquisition is done
    while( ! AD1CON1bits.DONE );    // wait until conversion done

    return ADC1BUF0;                // result stored in ADC1BUF0
}

void main(void)
{
    int clock = SYSTEMConfig(SYSCLK, SYS_CFG_ALL);

    setupStatusLED();

    ANSELBbits.ANSB2 = 0;
    adcConfigureManual();
    AD1CON1SET = 0x8000;

    T1CON = 0x8000;                 // TMR1 on, prescaler 1:1
    PR1 = 0xffff;                   // Set timer 1 period to its max (2^16-1)

    while( 1)
    {
        int n = analogRead(4);

        enableStatusLED();
        delay(n);

        disableStatusLED();
        delay(n);
    }
}