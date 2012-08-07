
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
    //TRISAbits.TRISA1 = 0;
}

void enableStatusLED()
{
    LATAbits.LATA0 = 1;
    //LATAbits.LATA1 = 0;
}

void disableStatusLED()
{
    LATAbits.LATA0 = 0;
    //LATAbits.LATA1 = 1;
}

void delay(int t)
{
    while( t--){
        TMR1 = 0;                   // reset timer 1
        while( TMR1 < SYSCLK/1000); // delay 1ms
    }
}

void main(void)
{
    int clock = SYSTEMConfig(SYSCLK, SYS_CFG_ALL);

    setupStatusLED();

    T1CON = 0x8000;                 // TMR1 on, prescaler 1:1
    PR1 = 0xffff;                   // Set timer 1 period to its max (2^16-1)

    while( 1)
    {
        enableStatusLED();
        delay(100);
        
        disableStatusLED();
        delay(100);
    }
}