// PIC32MZ_13.c - I2C and the MPU-9250
// by Aidan Mocke 2018-11-27
//

// PIC32MZ2048EFH144 Configuration Bit Settings

// 'C' source line config statements

// DEVCFG3
// USERID = No Setting
#pragma config FMIIEN = ON              // Ethernet RMII/MII Enable (MII Enabled)
#pragma config FETHIO = ON              // Ethernet I/O Pin Select (Default Ethernet I/O)
#pragma config PGL1WAY = OFF            // Permission Group Lock One Way Configuration (Allow multiple reconfigurations)
#pragma config PMDL1WAY = OFF           // Peripheral Module Disable Configuration (Allow multiple reconfigurations)
#pragma config IOL1WAY = OFF            // Peripheral Pin Select Configuration (Allow multiple reconfigurations)
#pragma config FUSBIDIO = OFF           // USB USBID Selection (Controlled by Port Function)

// DEVCFG2
#pragma config FPLLIDIV = DIV_1         // System PLL Input Divider (3x Divider)
#pragma config FPLLRNG = RANGE_5_10_MHZ // System PLL Input Range (5-10 MHz Input)
#pragma config FPLLICLK = PLL_FRC      // System PLL Input Clock Selection (POSC is input to the System PLL)
#pragma config FPLLMULT = MUL_50        // System PLL Multiplier (PLL Multiply by 50)
#pragma config FPLLODIV = DIV_2         // System PLL Output Clock Divider (2x Divider)
#pragma config UPLLFSEL = FREQ_24MHZ    // USB PLL Input Frequency Selection (USB PLL input is 24 MHz)

// DEVCFG1
#pragma config FNOSC = SPLL             // Oscillator Selection Bits (System PLL)
#pragma config DMTINTV = WIN_127_128    // DMT Count Window Interval (Window/Interval value is 127/128 counter value)
#pragma config FSOSCEN = OFF            // Secondary Oscillator Enable (Disable SOSC)
#pragma config IESO = OFF               // Internal/External Switch Over (Disabled)
#pragma config POSCMOD = OFF             // Primary Oscillator Configuration (External clock mode)
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

#define SYS_FREQ 200000000
#define MPU9250_ADDRESS 0x68            // The address of MPU9250 when the AD0 pin is connected to ground
#define MPU9250_WHOAMI  0x75            // Will return a set value based on device, in my case 0x73

long set_performance_mode()
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

void delay_us(unsigned int us)
{
    
    // Convert microseconds us into how many clock ticks it will take
    us *= SYS_FREQ / 1000000 / 2; // Core Timer updates every 2 ticks

    _CP0_SET_COUNT(0); // Set Core Timer count to 0

    while (us > _CP0_GET_COUNT()); // Wait until Core Timer count reaches the number we calculated earlier
}

void delay_ms(unsigned int ms)
{
    delay_us(ms * 1000);
}

// I2C_wait_for_idle() waits until the I2C peripheral is no longer doing anything  
void I2C_wait_for_idle(void)
{
    while(I2C1CON & 0x1F); // Acknowledge sequence not in progress
                                // Receive sequence not in progress
                                // Stop condition not in progress
                                // Repeated Start condition not in progress
                                // Start condition not in progress
    while(I2C1STATbits.TRSTAT); // Bit = 0 ? Master transmit is not in progress
}

// I2C_start() sends a start condition  
void I2C_start()
{
    I2C_wait_for_idle();
    I2C1CONbits.SEN = 1;
    while (I2C1CONbits.SEN == 1);
}

// I2C_stop() sends a stop condition  
void I2C_stop()
{
    I2C_wait_for_idle();
    I2C1CONbits.PEN = 1;
}

// I2C_restart() sends a repeated start/restart condition
void I2C_restart()
{
    I2C_wait_for_idle();
    I2C1CONbits.RSEN = 1;
    while (I2C1CONbits.RSEN == 1);
}

// I2C_ack() sends an ACK condition
void I2C_ack(void)
{
    I2C_wait_for_idle();
    I2C1CONbits.ACKDT = 0; // Set hardware to send ACK bit
    I2C1CONbits.ACKEN = 1; // Send ACK bit, will be automatically cleared by hardware when sent  
    while(I2C1CONbits.ACKEN); // Wait until ACKEN bit is cleared, meaning ACK bit has been sent
}

// I2C_nack() sends a NACK condition
void I2C_nack(void) // Acknowledge Data bit
{
    I2C_wait_for_idle();
    I2C1CONbits.ACKDT = 1; // Set hardware to send NACK bit
    I2C1CONbits.ACKEN = 1; // Send NACK bit, will be automatically cleared by hardware when sent  
    while(I2C1CONbits.ACKEN); // Wait until ACKEN bit is cleared, meaning NACK bit has been sent
}

// address is I2C slave address, set wait_ack to 1 to wait for ACK bit or anything else to skip ACK checking  
void I2C_write(unsigned char address, char wait_ack)
{
    I2C1TRN = address | 0;				// Send slave address with Read/Write bit cleared
    while (I2C1STATbits.TBF == 1);		// Wait until transmit buffer is empty
    I2C_wait_for_idle();				// Wait until I2C bus is idle
    if (wait_ack) while (I2C1STATbits.ACKSTAT == 1); // Wait until ACK is received  
}

// value is the value of the data we want to send, set ack_nack to 0 to send an ACK or anything else to send a NACK  
void I2C_read(unsigned char *value, char ack_nack)
{
    I2C1CONbits.RCEN = 1;				// Receive enable
    while (I2C1CONbits.RCEN);			// Wait until RCEN is cleared (automatic)  
    while (!I2C1STATbits.RBF);    		// Wait until Receive Buffer is Full (RBF flag)  
    *value = I2C1RCV;    				// Retrieve value from I2C1RCV
    
    if (!ack_nack)						// Do we need to send an ACK or a NACK?  
        I2C_ack();						// Send ACK  
    else
        I2C_nack();						// Send NACK  
}

// Write byte value to register at reg_address
void MPU9250_write(unsigned char reg_address, unsigned char value)
{
    I2C_start();						/* Send start condition */  
    I2C_write(MPU9250_ADDRESS << 1, 1); /* Send MPU9250's address, read/write bit not set (AD + R) */  
    I2C_write(reg_address, 1);			/* Send the register address (RA) */  
    I2C_write(value, 1);				/* Send the value to set it to */  
    I2C_stop();    						/* Send stop condition */  
}

// Read a byte from register at reg_address and return in *value
void MPU9250_read(unsigned char reg_address, unsigned char *value)
{
    I2C_start();						/* Send start condition */  
    I2C_write(MPU9250_ADDRESS << 1, 1);	/* Send MPU9250's address, read/write bit not set (AD + R) */  
    I2C_write(reg_address, 1);			/* Send the register address (RA) */  
    I2C_restart();						/* Send repeated start condition */  
    I2C_write(MPU9250_ADDRESS << 1 | 1, 1);	/* Send MPU9250's address, read/write bit set (AD + W) */  
    I2C_read(value, 1);					/* Read value from the I2C bus */  
    I2C_stop();    						/* Send stop condition */  
}

// I2C_init() initialises I2C1 at at frequency of [frequency]Hz
void I2C_init(double frequency)
{
    double BRG;
    
    I2C1CON = 0;			// Turn off I2C1 module
    I2C1CONbits.DISSLW = 1; // Disable slew rate for 100kHz
    
    BRG = (1 / (2 * frequency)) - 0.000000104;
    BRG *= (SYS_FREQ / 2) - 2;    
    
    I2C1BRG = (int)BRG;		// Set baud rate
    I2C1CONbits.ON = 1;		// Turn on I2C1 module
}

unsigned char main()
{
    unsigned char value;
    
    // Set performance to ultra rad
    set_performance_mode();
    
    // Moved all the ANSEL, TRIS and LAT settings to their own function
    setup_ports();        
	
    // Enable multi-vectored interrupts mode
    INTCONbits.MVEC = 1;
    
    // No need to set up PPS, I2C hardware is fixed to certain pins. SCL1 = RA14, SDA1 = RA15

    // Initialise I2C1 at 100kHz
    I2C_init(100000);
    
    while (1)
	{
		/* Read the value at register 0x75, the MPU9250's WHOAMI register. Should return 0x68, 0x71 or 0x73 depending on version. */  
		MPU9250_read(MPU9250_WHOAMI, &value);
		
		/* Wait 10ms before trying again so as not to overwhelm the MPU9250 or the PIC32MZ's I2C peripheral */  
		delay_ms(10);
	}
}