// PIC32MZ_14.c - Using SQI to read and write from an external PSRAM
// by Aidan Mocke 2018-12-11

// PIC32MZ2048EFH144 Configuration Bit Settings

// DEVCFG3
// USERID = No Setting
#pragma config FMIIEN = ON              // Ethernet RMII/MII Enable (MII Enabled)
#pragma config FETHIO = ON              // Ethernet I/O Pin Select (Default Ethernet I/O)
#pragma config PGL1WAY = OFF            // Permission Group Lock One Way Configuration (Allow multiple reconfigurations)
#pragma config PMDL1WAY = OFF           // Peripheral Module Disable Configuration (Allow multiple reconfigurations)
#pragma config IOL1WAY = OFF            // Peripheral Pin Select Configuration (Allow multiple reconfigurations)
#pragma config FUSBIDIO = OFF           // USB USBID Selection (Controlled by Port Function)

// DEVCFG2
#pragma config FPLLIDIV = DIV_2         // System PLL Input Divider (1x Divider)
#pragma config FPLLRNG = RANGE_5_10_MHZ // System PLL Input Range (5-10 MHz Input)
#pragma config FPLLICLK = PLL_FRC       // System PLL Input Clock Selection (POSC is input to the System PLL)
#pragma config FPLLMULT = MUL_50        // System PLL Multiplier (PLL Multiply by 50)
#pragma config FPLLODIV = DIV_2         // System PLL Output Clock Divider (2x Divider)
#pragma config UPLLFSEL = FREQ_24MHZ    // USB PLL Input Frequency Selection (USB PLL input is 24 MHz)

// DEVCFG1
#pragma config FNOSC = SPLL             // Oscillator Selection Bits (System PLL)
#pragma config DMTINTV = WIN_127_128    // DMT Count Window Interval (Window/Interval value is 127/128 counter value)
#pragma config FSOSCEN = OFF            // Secondary Oscillator Enable (Disable SOSC)
#pragma config IESO = OFF               // Internal/External Switch Over (Disabled)
#pragma config POSCMOD = OFF            // Primary Oscillator Configuration (Disabled)
#pragma config OSCIOFNC = OFF           // CLKO Output Signal Active on the OSCO Pin (Disabled)
#pragma config FCKSM = CSECME           // Clock Switching and Monitor Selection (Clock Switch Enabled, FSCM Enabled)
#pragma config WDTPS = PS1048576        // Watchdog Timer Postscaler (1:1048576)
#pragma config WDTSPGM = STOP           // Watchdog Timer Stop During Flash Programming (WDT stops during Flash programming)
#pragma config WINDIS = NORMAL          // Watchdog Timer Window Mode (Watchdog Timer is in non-Window mode)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable (WDT Disabled)
#pragma config FWDTWINSZ = WINSZ_25     // Watchdog Timer Window Size (Window size is 25%)
#pragma config DMTCNT = DMT31           // Deadman Timer Count Selection (2^31 (2147483648))
#pragma config FDMTEN = OFF             // Deadman Timer Enable (Deadman Timer is disabled)

// DEVCFG0
#pragma config DEBUG = OFF              // Background Debugger Enable (Debugger is disabled)
#pragma config JTAGEN = OFF             // JTAG Enable (JTAG Disabled)
#pragma config ICESEL = ICS_PGx1        // ICE/ICD Comm Channel Select (Communicate on PGEC1/PGED1)
#pragma config TRCEN = OFF              // Trace Enable (Trace features in the CPU are disabled)
#pragma config BOOTISA = MIPS32         // Boot ISA Selection (Boot code and Exception code is MIPS32)
#pragma config FECCCON = OFF_UNLOCKED   // Dynamic Flash ECC Configuration (ECC and Dynamic ECC are disabled (ECCCON bits are writable))
#pragma config FSLEEP = OFF             // Flash Sleep Mode (Flash is powered down when the device is in Sleep mode)
#pragma config DBGPER = PG_ALL          // Debug Mode CPU Access Permission (Allow CPU access to all permission regions)
#pragma config SMCLR = MCLR_NORM        // Soft Master Clear Enable bit (MCLR pin generates a normal system Reset)
#pragma config SOSCGAIN = GAIN_2X       // Secondary Oscillator Gain Control bits (2x gain setting)
#pragma config SOSCBOOST = ON           // Secondary Oscillator Boost Kick Start Enable bit (Boost the kick start of the oscillator)
#pragma config POSCGAIN = GAIN_2X       // Primary Oscillator Gain Control bits (2x gain setting)
#pragma config POSCBOOST = ON           // Primary Oscillator Boost Kick Start Enable bit (Boost the kick start of the oscillator)
#pragma config EJTAGBEN = NORMAL        // EJTAG Boot (Normal EJTAG functionality)
#pragma config CP = OFF                 // Code Protect (Protection Disabled)


#include <xc.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#define SYS_FREQ 200000000              // Running at 200MHz

unsigned int QUAD_MODE_SETTING = 0;		// Starting off in single channel mode
unsigned char *TXDATA = (unsigned char *)&SQI1TXDATA;	// Address to write to for 8-bit data
unsigned char *RXDATA = (unsigned char *)&SQI1RXDATA;	// Address to read from for 8-bit data

void delay_us(unsigned int us)
{
    // Convert microseconds us into how many clock ticks it will take
    us *= (SYS_FREQ / 1000000) / 2; // Core Timer updates every 2 ticks
                      
    _CP0_SET_COUNT(0); // Set Core Timer count to 0

    while (us > _CP0_GET_COUNT()); // Wait until Core Timer count reaches the number we calculated earlier
}

void delay_ms(int ms)
{
    delay_us(ms * 1000);
}

void set_performance_mode()
{   
	unsigned int cp0;
	
    // Unlock Sequence
    asm volatile("di"); // Disable all interrupts
    SYSKEY = 0xAA996655;
    SYSKEY = 0x556699AA;  

    // PB1DIV
    // Peripheral Bus 1 cannot be turned off, so there's no need to turn it on
    PB1DIVbits.PBDIV = 1; // Peripheral Bus 1 Clock Divisor Control (PBCLK1 is SYSCLK divided by 2)

    // PB2DIV
    PB2DIVbits.ON = 1; // Peripheral Bus 2 Output Clock Enable (Output clock is enabled)
    PB2DIVbits.PBDIV = 1; // Peripheral Bus 2 Clock Divisor Control (PBCLK2 is SYSCLK divided by 2)

    // PB3DIV
    PB3DIVbits.ON = 1; // Peripheral Bus 2 Output Clock Enable (Output clock is enabled)
    PB3DIVbits.PBDIV = 1; // Peripheral Bus 3 Clock Divisor Control (PBCLK3 is SYSCLK divided by 2)

    // PB4DIV
    PB4DIVbits.ON = 1; // Peripheral Bus 4 Output Clock Enable (Output clock is enabled)
    while (!PB4DIVbits.PBDIVRDY); // Wait until it is ready to write to
    PB4DIVbits.PBDIV = 0; // Peripheral Bus 4 Clock Divisor Control (PBCLK4 is SYSCLK divided by 1)

    // PB5DIV
    PB5DIVbits.ON = 1; // Peripheral Bus 5 Output Clock Enable (Output clock is enabled)
    PB5DIVbits.PBDIV = 1; // Peripheral Bus 5 Clock Divisor Control (PBCLK5 is SYSCLK divided by 2)

    // PB7DIV
    PB7DIVbits.ON = 1; // Peripheral Bus 7 Output Clock Enable (Output clock is enabled)
    PB7DIVbits.PBDIV = 0; // Peripheral Bus 7 Clock Divisor Control (PBCLK7 is SYSCLK divided by 1)

    // PB8DIV
    PB8DIVbits.ON = 1; // Peripheral Bus 8 Output Clock Enable (Output clock is enabled)
    PB8DIVbits.PBDIV = 1; // Peripheral Bus 8 Clock Divisor Control (PBCLK8 is SYSCLK divided by 2)

    // PRECON - Set up prefetch
    PRECONbits.PFMSECEN = 0; // Flash SEC Interrupt Enable (Do not generate an interrupt when the PFMSEC bit is set)
    PRECONbits.PREFEN = 0b11; // Predictive Prefetch Enable (Enable predictive prefetch for any address)
    PRECONbits.PFMWS = 0b010; // PFM Access Time Defined in Terms of SYSCLK Wait States (Two wait states)

    // Set up caching
    cp0 = _mfc0(16, 0);
    cp0 &= ~0x07;
    cp0 |= 0b011; // K0 = Cacheable, non-coherent, write-back, write allocate
    _mtc0(16, 0, cp0);  

    // Lock Sequence
    SYSKEY = 0x33333333;
    asm volatile("ei"); // Enable all interrupts
}

void SQI_init()
{
// Turn on internal pull-ups for G12, G13, G14 and A7 (SQID0 ~ SQID3)
    CNPUGbits.CNPUG12 = 1;
    CNPUGbits.CNPUG13 = 1;
    CNPUGbits.CNPUG14 = 1;
    CNPUAbits.CNPUA7 = 1;
    
// Make G12, G13, G14 and A7 inputs (SQID0 ~ SQID3)
    TRISGbits.TRISG12 = 1;
    TRISGbits.TRISG13 = 1;
    TRISGbits.TRISG14 = 1;
    TRISAbits.TRISA7 = 1;

// Disable trace outputs, they are shared with SQI
    CFGCONbits.TROEN = 0;
    
// Turn on Reference Clock 2 Output
    if (!REFO2CONbits.ACTIVE)
    {
        REFO2CONbits.RODIV = 1;
        REFO2CONbits.ROSEL = 1;
        REFO2CONbits.ON = 1;
        while (REFO2CONbits.DIVSWEN);
        REFO2CONbits.OE = 1;
    }
    
// Turn off the SQI clock divider
    SQI1CLKCONbits.CLKDIV = 0;
// Enable the SQI peripheral
    SQI1CLKCONbits.EN = 1;
// Wait until the SQI clock has stabilised
    while (!SQI1CLKCONbits.STABLE);    
    
   
// Reset the SQI peripheral by setting RESET bit. It will be cleared automatically.
    SQI1CFGbits.RESET = 1;
// Set the SPI mode to 0 (SQI clock is held low when idle)
    SQI1CFGbits.CPOL = 0;
    SQI1CFGbits.CPHA = 0;
// Set the SQI transfer mode to Programmed I/O (PIO) mode
    SQI1CFGbits.MODE = 1;
// Switch on Burst Transfer mode (this must always be set to 1)
    SQI1CFGbits.BURSTEN = 1;
// Initially enable only SQID0 and SQID1
    SQI1CFGbits.DATAEN = 0b10;

// Set the control buffer threshold to 1 byte    
    SQI1THR = 0x001;
// Set the command buffer threshold to 1 byte for transfer and 1 byte for receive
    SQI1CMDTHR = 0x101;
}

// SRAM_select sets the CS pin of the SRAM to either 0 (selected) or 1 (de-selected)
void SRAM_select(char select)
{
    LATJbits.LATJ1 = select;
}

void SRAM_send_cmd(unsigned char cmd)
{
    // Setup SQI1CON to write 1 byte
    SQI1CON = 0x00510001 | QUAD_MODE_SETTING;
    
	// Write the **8-bit** data to the FIFO buffer
    *TXDATA = cmd;

	// Wait until the transmit buffer is empty
    while (SQI1STAT1bits.TXBUFFREE < 32);
}

unsigned int endian_change(unsigned int data)
{
	unsigned char change[4];
	
	// Get separate bytes
	change[0] = data >> 24;
	change[1] = (data & 0x00FF0000) >> 16;
	change[2] = (data & 0x0000FF00) >> 8;
	change[3] = (data & 0xFF);
	
	// Combine them in reverse order
	return ((change[3] << 24) | (change[2] << 16) | (change[1] << 8) |(change[0]));
}

void SRAM_start_write(int address)
{
    // Setup SQI1CON to write 4 bytes
    SQI1CON = 0x00510004  | QUAD_MODE_SETTING;
    
	// Single Channel Mode Write is command 0x02 and Quad Channel Mode Write is 0x38
    if (!QUAD_MODE_SETTING)
        SQI1TXDATA = endian_change((0x02 << 24) | address);
    else
        SQI1TXDATA = endian_change((0x03 << 38) | address);

	// Wait until the transmit buffer is empty
    while (SQI1STAT1bits.TXBUFFREE < 32);
}

void SRAM_start_read(int address)
{    
    // Setup SQI1CON to write 4 bytes
    SQI1CON = 0x00510004 | QUAD_MODE_SETTING;
    
	// Single Channel Mode Read is command 0x03 and Quad Channel Mode Read is 0xEB
    if (!QUAD_MODE_SETTING)
        SQI1TXDATA = endian_change((0x03 << 24) | address);
    else
        SQI1TXDATA = endian_change((0xEB << 24) | address);

	// Wait until the transmit buffer is empty
    while (SQI1STAT1bits.TXBUFFREE < 32);
}

unsigned char SRAM_read_byte()
{
    // Setup SQI1CON to read 1 byte
    SQI1CON = 0x00520001 | QUAD_MODE_SETTING;
    
	// Wait until 1 byte has been received
    while ((SQI1STAT1 & 0xFF) != 1);

	// Read the **8-bit** data from the FIFO buffer and return it
    return *RXDATA;
}

void SRAM_get_EID(unsigned char *buffer)
{    
    int cnt;
    
    // Select the SRAM chip by setting Chip Select to 0
    SRAM_select(0);
    
    // Setup SQI1CON to write 4 bytes
    SQI1CON = 0x00510004 | QUAD_MODE_SETTING;    
    // Command 0x9F followed by 3 address bytes (all 0) is the Read EID command
    SQI1TXDATA = 0x0000009F;
    // Wait until all bytes have been transmitted
    while (SQI1STAT1bits.TXBUFFREE < 32);
        
    // Setup SQI1CON to read 8 bytes
    SQI1CON = 0x00520008 | QUAD_MODE_SETTING;
    // Wait until 8 bytes have been received
    while ((SQI1STAT1 & 0xFF) != 0x08);
    
    // Put the 8 bytes into the destination array
    for (cnt = 0; cnt < 8; cnt++)
    {
        *buffer++ = *RXDATA;
    }
    
    // De-select the SRAM chip by setting Chip Select to 1
    SRAM_select(1);
}

void SRAM_go_SQI()
{
    // Select the SRAM chip by setting Chip Select to 0
    SRAM_select(0);

    // Setup SQI1CON to write 4 bytes
    SQI1CON = 0x00510001 | QUAD_MODE_SETTING;
    
    // Command 0x35 is the Enter Quad Mode command
    *TXDATA = 0x35;
    
    while (SQI1STAT1bits.TXBUFFREE < 32);
    
    // De-select the SRAM chip by setting Chip Select to 1
    SRAM_select(1);
         
    // Remember that we are now in quad mode
    QUAD_MODE_SETTING = 0x00080000;
}

void setup_ports()
{
    // Turn off all analog inputs, make everything digital
    ANSELA = 0;
    ANSELB = 0;
    ANSELC = 0;
    ANSELD = 0;
    ANSELE = 0;
    ANSELF = 0;
    ANSELG = 0;
    ANSELH = 0;
    ANSELJ = 0;
    
    // Make all ports outputs initially
    TRISA = 0;
    TRISB = 0;
    TRISC = 0;
    TRISD = 0;
    TRISE = 0;
    TRISF = 0;
    TRISG = 0;
    TRISH = 0;
    TRISJ = 0;        
    
    // Set all port values low initially
    LATA = 0;
    LATB = 0;
    LATC = 0;
    LATD = 0;
    LATE = 0;
    LATF = 0;
    LATG = 0;
    LATH = 0;
    LATJ = 0;
}

void SRAM_write_buffer(int address, unsigned char *buffer, int num_bytes)
{
	int cnt;
	
    // Select the SRAM chip by setting Chip Select to 0
	SRAM_select(0);
	
    // Tell the chip we want to start writing to the address contained in [address]
    SRAM_start_write(address);
	
	// Write the data one byte at a time
    for (cnt = 0; cnt < num_bytes; cnt++)
    {
        SRAM_send_cmd(buffer[cnt]);
    }
	
    // De-select the SRAM chip by setting Chip Select to 1
    SRAM_select(1);
}

void SRAM_read_buffer(int address, unsigned char *buffer, int num_bytes)
{
	int cnt;
	
    // Select the SRAM chip by setting Chip Select to 0
	SRAM_select(0);
	
    // Tell the chip we want to start reading from the address contained in [address]
	SRAM_start_read(address);
	
    // This particular chip needs a few cycles to get ready before a Fast Read command, so I do this by reading and discarding three bytes
    *buffer = SRAM_read_byte();
    *buffer = SRAM_read_byte();
    *buffer = SRAM_read_byte();    
	
    // Read the data from the SRAM and put it into [buffer]
	for (cnt = 0; cnt < num_bytes; cnt++)
    {
        buffer[cnt] = SRAM_read_byte();
    }
	
    // De-select the SRAM chip by setting Chip Select to 1
    SRAM_select(1);   
}

int main()
{
    int cnt;
    unsigned char buffer[256];
    
    set_performance_mode();

    setup_ports();
    
    // De-select the SRAM chip by setting Chip Select to 1
	SRAM_select(1);
    
    // Setup SQI peripheral in single channel mode
    SQI_init();
    
    // Get EID data
    SRAM_get_EID(buffer);
    
    // Fill the buffer with values 0 to 255
    for (cnt = 0; cnt < 256; cnt++)
        buffer[cnt] = cnt;
    
    // Switch SRAM to quad lane mode
    SRAM_go_SQI();
    
    // Write data in buffer to SRAM
    SRAM_write_buffer(0, buffer, 256);
	    
    // Read the data back into buffer from SRAM
    SRAM_read_buffer(0, buffer, 256);
}