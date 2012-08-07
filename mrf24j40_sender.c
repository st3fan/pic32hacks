
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

#define RXMCR     0x00
#define PANIDL    0x01
#define PANIDH    0x02
#define SADRL     0x03
#define SADRH     0x04
#define EADR0     0x05
#define EADR1     0x06
#define EADR2     0x07
#define EADR3     0x08
#define EADR4     0x09
#define EADR5     0x0a
#define EADR6     0x0b
#define EADR7     0x0c
#define RXFLUSH   0x0d
#define TXNMTRIG  0x1b
#define TXSR      0x24
#define ISRSTS    0x31
#define INTMSK    0x32
#define GPIO      0x33
#define TRISGPIO  0x34
#define RFCTL     0x36
#define RSSITHCCA 0x3F

#define BBREG0      0x38
#define BBREG1      0x39
#define BBREG2      0x3A
#define BBREG3      0x3B
#define BBREG4      0x3C
#define BBREG6      0x3E

#define INTSTAT     0x31
#define INTCON      0x32

#define RFCTRL0   0x200
#define RFCTRL2   0x202
#define RFCTRL3   0x203
#define RFCTRL6   0x206
#define RFCTRL7   0x207
#define RFCTRL8   0x208
#define CLKINTCR  0x211
#define SLPCON0   0x211
#define CLCCTRL   0x220

// CS    RB4
// SCK   RB14 / SCK1
// SDI   RB8
// SDO   RB5
// RESET RB9

void radioSetup()
{
    SDI1Rbits.SDI1R = 1;        // SET SDI1 to RPB5
    RPB8Rbits.RPB8R = 3;        // SET RPB8R to SDO1
    TRISBbits.TRISB4 = 0;       // RB4 Slave Select
    TRISBbits.TRISB9 = 0;       // RB9 Reset

    SPI1CON = 0x8120;
    SPI1BRG = 0x0001;

    LATBbits.LATB4 = 1;         // Set SS to High
    LATBbits.LATB9 = 1;         // Set Reset to High
}

unsigned char radioSPITransfer(unsigned char data)
{
    SPI1BUF = data;                     // write to buffer for TX
    while( !SPI1STATbits.SPIRBF);       // wait transfer complete
    return SPI1BUF;                     // read the received value
}

unsigned char radioReadShort(unsigned char address)
{
    unsigned char result = 0;

    mPORTBClearBits(BIT_4);
    {
        (void) radioSPITransfer( (address<<1) & 0x7E );
        result = radioSPITransfer(0xff);
    }
    mPORTBSetBits(BIT_4);

    return result;
}

unsigned char radioReadLong(unsigned short address)
{
    unsigned char value;

    mPORTBClearBits(BIT_4);
    {
        (void) radioSPITransfer((address>>3) | 0x80);
        (void) radioSPITransfer((address<<5) & 0xE0);
        value = radioSPITransfer(0xFF);
    }
    mPORTBSetBits(BIT_4);

    return value;
}

void radioWriteShort(unsigned char address, unsigned char data)
{
    mPORTBClearBits(BIT_4);
    {
        (void) radioSPITransfer(((address<<1) & 0x7E) | 0x01);
        (void) radioSPITransfer(data);
    }
    mPORTBSetBits(BIT_4);
}

void radioWriteLong(unsigned short address, unsigned char data)
{
    mPORTBClearBits(BIT_4);
    {
        (void) radioSPITransfer((address>>3) | 0x80);
        (void) radioSPITransfer(((address<<5) & 0xE0) | 0x10);
        (void) radioSPITransfer(data);
    }
    mPORTBSetBits(BIT_4);
}

void radioReset()
{
    // Reset the chip

    LATBbits.LATB9 = 0;
    timerDelayMillis(1);
    LATBbits.LATB9 = 1;
    timerDelayMillis(1);

    // Reset the RF module
    radioWriteShort(RFCTL, 0x04);
    radioWriteShort(RFCTL, 0x00);
    radioWriteShort(RFCTL, 0x00);

    // Setup PAN and short address
    radioWriteShort(PANIDL, 0xAA);
    radioWriteShort(PANIDH, 0xAA);
    radioWriteShort(SADRL, 0xAA);
    radioWriteShort(SADRH, 0xAA);

    // TODO Write MAC addresses here. We don't care ???

    

    radioWriteLong(RFCTRL2, 0x80);  // Enable RF PLL.

    radioWriteLong(RFCTRL3, 0x00);  // Full power.
    radioWriteLong(RFCTRL6, 0x80);  // Enable TX filter (recommended)
    radioWriteLong(RFCTRL8, 0x10);  // Enhanced VCO (recommended)

    radioWriteShort(BBREG2,0x78);   // Clear Channel Assesment use carrier sense.
    radioWriteShort(BBREG6,0x40);   // Calculate RSSI for Rx packet.
    radioWriteShort(RSSITHCCA,0x00);// RSSI threshold for CCA.

    radioWriteLong(RFCTRL0, 0x00);  // Channel 11.

    radioWriteShort(RXMCR, 0x01); // Don't check address upon reception.

    // Configure interrupts
    radioWriteShort(INTCON, 0b11110111); // Enable RX interrupts

    // Reset RF module with new settings.
    radioWriteShort(RFCTL, 0x04);
    radioWriteShort(RFCTL, 0x00);

    timerDelayMillis(1);
}

void radioDebugPrintRegister(const char* name, unsigned char address)
{
    char tmp[80];
    sprintf(tmp,  "  0x%.2X %12s = 0x%.2X\r\n", address, name, radioReadShort(address));
    putsUART1(tmp);
}

void radioDebugRegisters()
{
   putsUART1("\r\nMRF24J40 Registers:\r\n\r\n");
   radioDebugPrintRegister("RXMCR", RXMCR);
   radioDebugPrintRegister("PANIDL", PANIDL);
   radioDebugPrintRegister("PANIDH", PANIDH);
   radioDebugPrintRegister("SADRL", SADRL);
   radioDebugPrintRegister("SADRH", SADRH);
   radioDebugPrintRegister("EADR0", EADR0);
   radioDebugPrintRegister("EADR1", EADR1);
   radioDebugPrintRegister("EADR2", EADR2);
   radioDebugPrintRegister("EADR3", EADR3);
   radioDebugPrintRegister("EADR4", EADR4);
   radioDebugPrintRegister("EADR5", EADR5);
   radioDebugPrintRegister("EADR6", EADR6);
   radioDebugPrintRegister("EADR7", EADR7);
   radioDebugPrintRegister("RXFLUSH", RXFLUSH);
   radioDebugPrintRegister("TXNMTRIG", TXNMTRIG);
   radioDebugPrintRegister("TXSR", TXSR);
   radioDebugPrintRegister("ISRSTS", ISRSTS);
   radioDebugPrintRegister("INTMSK", INTMSK);
   radioDebugPrintRegister("GPIO", GPIO);
   radioDebugPrintRegister("TRISGPIO", TRISGPIO);
   radioDebugPrintRegister("RFCTL", RFCTL);
   radioDebugPrintRegister("BBREG2", BBREG2);
   radioDebugPrintRegister("BBREG6", BBREG6);
   radioDebugPrintRegister("RSSITHCCA", RSSITHCCA);
}

void radioTransmit(const unsigned char* data, int length)
{
  unsigned char i;

  radioWriteLong(0x000, 0);
  radioWriteLong(0x001, length);
  for(i=0; i<length; i++) {
    radioWriteLong(0x002+i, data[i]);
  }

  radioWriteShort(TXNMTRIG, 0x01);
}

int radioReceive(unsigned char* buffer, int bufferSize)
{
    unsigned char i, length;

    length = radioReadLong(0x300);
    if (length > bufferSize) {
        length = bufferSize;
    }

    for (i = 0; i < length; i++) {
        *buffer++ = radioReadLong(0x301 + i);
    }

    return length;
}

void radioDebugPacket(const unsigned char* buffer, int length)
{
    int i;
    for(i = 0; i < length; i++)
    {
        char tmp[32];
        sprintf(tmp, "  %2d 0x%.2x\r\n", i, buffer[i]);
        putsUART1(tmp);
    }
}

void __ISR(_EXTERNAL_0_VECTOR, ipl7) radioInterruptHandler()
{
    // Disable interrupts, disable receiving packets
    radioWriteShort(INTCON, 0b11111111); // Disable all interrupts
    radioWriteShort(BBREG1,0x04);

    {
        unsigned char mask = radioReadShort(INTSTAT);
        if (mask & 0b00001000)
        {
            statusLEDToggle();

            WriteCoreTimer(0);

            // Read the packet
            unsigned char buffer[128];
            int length = radioReceive(buffer, 128);

            unsigned int time = ReadCoreTimer();

            if (length != 0) {
                char tmp[128];
                sprintf(tmp, "\r\nReceived a packet in %u ticks\r\n\r\n", time);
                putsUART1(tmp);
                radioDebugPacket(buffer, length);
            } else {
                putsUART1("\r\nReceived an empty packet (false alarm?)\r\n\r\n");
            }
        }
    }

    // Enable interrupts and packet reception
    radioWriteShort(INTCON, 0b11110111); // Enable RX interrupts
    radioWriteShort(BBREG1, 0x00);

    mINT0ClearIntFlag();
}

void main(void)
{
    int clock = SYSTEMConfig(SYSCLK, SYS_CFG_ALL);

    timerSetup();
    timerDelayMillis(2500);

    statusLEDSetup();
    statusLEDDisable();
    serialSetup(clock);

    radioSetup();
    radioReset();
    radioDebugRegisters();

#if RECEIVER
    INTEnableSystemMultiVectoredInt();
    ConfigINT0(EXT_INT_PRI_7 | FALLING_EDGE_INT | EXT_INT_ENABLE);

    while (1) {
        // Do nothing
    }
#endif
}
