
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

void statusLEDSetup()
{
    PORTSetPinsDigitalOut(IOPORT_A, BIT_4);
    mPORTAClearBits(BIT_4);
}

void statusLEDEnable()
{
    mPORTASetBits(BIT_4);
}

void statusLEDDisable()
{
    mPORTAClearBits(BIT_4);
}

void statusLEDToggle()
{
    mPORTAToggleBits(BIT_4);
}

void timerSetup()
{
    T1CON = 0x8000;                 // TMR1 on, prescaler 1:1
    PR1 = 0xffff;                   // Set timer 1 period to its max (2^16-1)
}

void timerDelayMillis(int t)
{
    while(t--) {
        TMR1 = 0;                   // reset timer 1
        while( TMR1 < SYSCLK/1000); // delay 1ms
    }
}

void serialSetup(int clock)
{
    RPA0R = 1;
    OpenUART1(UART_EN, UART_TX_ENABLE | UART_RX_ENABLE, mUARTBRG(clock, 38400));
}

void __ISR(_EXTERNAL_0_VECTOR, ipl7) INT0Interrupt()
{
    statusLEDToggle();
    mINT0ClearIntFlag();
} 

void main(void)
{
    int clock = SYSTEMConfig(SYSCLK, SYS_CFG_ALL);

    timerSetup();
    timerDelayMillis(1000);

    statusLEDSetup();
    serialSetup(clock);

    INTEnableSystemMultiVectoredInt();
    ConfigINT0(EXT_INT_PRI_7 | RISING_EDGE_INT | EXT_INT_ENABLE);

    putsUART1("\r\nExternal Interrupt test. If INT0 (B7) fires, we toggle the LED on A4\r\n");

    while(1) {
        // Nothing to do as everything happens from the interrupt handler.
    }
}
