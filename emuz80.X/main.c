/*
  PIC18F47Q43 ROM RAM and UART emulation firmware
  This single source file contains all code

  Target: EMUZ80 - The computer with only Z80 and PIC18F47Q43
  Compiler: MPLAB XC8 v2.36
  Written by Tetsuya Suzuki

  for COSMAC CDP1802 by kanpapa
*/

// CONFIG1
#pragma config FEXTOSC = OFF    // External Oscillator Selection (Oscillator not enabled)
#pragma config RSTOSC = HFINTOSC_64MHZ// Reset Oscillator Selection (HFINTOSC with HFFRQ = 64 MHz and CDIV = 1:1)

// CONFIG2
#pragma config CLKOUTEN = OFF   // Clock out Enable bit (CLKOUT function is disabled)
#pragma config PR1WAY = ON      // PRLOCKED One-Way Set Enable bit (PRLOCKED bit can be cleared and set only once)
#pragma config CSWEN = ON       // Clock Switch Enable bit (Writing to NOSC and NDIV is allowed)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor enabled)

// CONFIG3
#pragma config MCLRE = EXTMCLR  // MCLR Enable bit (If LVP = 0, MCLR pin is MCLR; If LVP = 1, RE3 pin function is MCLR )
#pragma config PWRTS = PWRT_OFF // Power-up timer selection bits (PWRT is disabled)
#pragma config MVECEN = ON      // Multi-vector enable bit (Multi-vector enabled, Vector table used for interrupts)
#pragma config IVT1WAY = ON     // IVTLOCK bit One-way set enable bit (IVTLOCKED bit can be cleared and set only once)
#pragma config LPBOREN = OFF    // Low Power BOR Enable bit (Low-Power BOR disabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled , SBOREN bit is ignored)

// CONFIG4
#pragma config BORV = VBOR_1P9  // Brown-out Reset Voltage Selection bits (Brown-out Reset Voltage (VBOR) set to 1.9V)
#pragma config ZCD = OFF        // ZCD Disable bit (ZCD module is disabled. ZCD can be enabled by setting the ZCDSEN bit of ZCDCON)
#pragma config PPS1WAY = OFF    // PPSLOCK bit One-Way Set Enable bit (PPSLOCKED bit can be set and cleared repeatedly (subject to the unlock sequence))
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = ON         // Low Voltage Programming Enable bit (Low voltage programming enabled. MCLR/VPP pin function is MCLR. MCLRE configuration bit is ignored)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Extended Instruction Set and Indexed Addressing Mode disabled)

// CONFIG5
#pragma config WDTCPS = WDTCPS_31// WDT Period selection bits (Divider ratio 1:65536; software control of WDTPS)
#pragma config WDTE = OFF       // WDT operating mode (WDT Disabled; SWDTEN is ignored)

// CONFIG6
#pragma config WDTCWS = WDTCWS_7// WDT Window Select bits (window always open (100%); software control; keyed access not required)
#pragma config WDTCCS = SC      // WDT input clock selector (Software Control)

// CONFIG7
#pragma config BBSIZE = BBSIZE_512// Boot Block Size selection bits (Boot Block size is 512 words)
#pragma config BBEN = OFF       // Boot Block enable bit (Boot block disabled)
#pragma config SAFEN = OFF      // Storage Area Flash enable bit (SAF disabled)
#pragma config DEBUG = OFF      // Background Debugger (Background Debugger disabled)

// CONFIG8
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block not Write protected)
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers not Write protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not Write protected)
#pragma config WRTSAF = OFF     // SAF Write protection bit (SAF not Write Protected)
#pragma config WRTAPP = OFF     // Application Block write protection bit (Application Block not write protected)

// CONFIG10
#pragma config CP = OFF         // PFM and Data EEPROM Code Protection bit (PFM and Data EEPROM code protection disabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdio.h>

//#define Z80_CLK 2500000UL // Z80 clock frequency(Max 16MHz)

//#define ROM_SIZE 0x4000 // ROM size 16K bytes
//#define RAM_SIZE 0x1000 // RAM size 4K bytes
//#define RAM_TOP 0x8000 // RAM start address

// CDP1802
//#define Z80_CLK 1800000UL // CDP1802 clock frequency(1.8MHz)
#define Z80_CLK 180UL // CDP1802 clock frequency(0.18MHz) slowdown

#define ROM_SIZE 0x80 // ROM size 128 bytes
#define RAM_SIZE 0x80 // RAM size 128 bytes
#define RAM_TOP 0x80 // RAM start address

#define UART_DREG 0xE000 // UART data register address
#define UART_CREG 0xE001 // UART control register address

#define _XTAL_FREQ 64000000UL

extern const unsigned char rom[]; // Equivalent to ROM, see end of this file
unsigned char ram[RAM_SIZE]; // Equivalent to RAM
static union {
    unsigned int w; // 16 bits Address
    struct {
        unsigned char l; // Address low 8 bits
        unsigned char h; // Address high 8 bits
    };
} address;
    

/*
// UART3 Transmit
void putch(char c) {
    while(!U3TXIF); // Wait or Tx interrupt flag set
    U3TXB = c; // Write data
}

// UART3 Recive
char getch(void) {
    while(!U3RXIF); // Wait for Rx interrupt flag set
    return U3RXB; // Read data
}
*/

#define dff_reset() {G3POL = 1; G3POL = 0;}
#define db_setin() (TRISC = 0xff)
#define db_setout() (TRISC = 0x00)

// Never called, logically
void __interrupt(irq(default),base(8)) Default_ISR(){}

// Called at Z80 MREQ falling edge (PIC18F47Q43 issues WAIT)
// Called at COSMAC TPB falling edge (PIC18F47Q43 issues WAIT)
void __interrupt(irq(CLC1),base(8)) CLC_ISR(){
    CLC1IF = 0; // Clear interrupt flag
    
    //address.h = PORTD; // Read address high
    address.h = 0; // Read address high (256byte only)
    address.l = PORTB; // Read address low
    
    if(!RA5) { // COSMAC memory read cycle (MRD active)
        db_setout(); // Set data bus as output
        if(address.w < ROM_SIZE){ // ROM area
            LATC = rom[address.w]; // Out ROM data
        } else if((address.w >= RAM_TOP) && (address.w < (RAM_TOP + RAM_SIZE))){ // RAM area
            LATC = ram[address.w - RAM_TOP]; // RAM data
        //} else if(address.w == UART_CREG){ // UART control register
        //    LATC = PIR9; // U3 flag
        //} else if(address.w == UART_DREG){ // UART data register
        //    LATC = U3RXB; // U3 RX buffer
        } else { // Out of memory
            LATC = 0xff; // Invalid data
        }
    } else { // COSMAC memory write cycle (MRD inactive)
        //if(RE0) while(RE0);
            if((address.w >= RAM_TOP) && (address.w < (RAM_TOP + RAM_SIZE))){ // RAM area
                ram[address.w - RAM_TOP] = PORTC; // Write into RAM
                //} else if(address.w == UART_DREG) { // UART data register
                //    U3TXB = PORTC; // Write into U3 TX buffer
            }
        //}
    }
    dff_reset(); // WAIT inactive
}

//  Called at COSMAC TPB rising edge
void __interrupt(irq(INT0),base(8)) INT0_ISR(){
    INT0IF = 0; // Clear interrupt flag
    db_setin(); // Set data bus as input
}

// main routine
void main(void) {
    // System initialize
    OSCFRQ = 0x08; // 64MHz internal OSC

    // Address bus A15-A8 pin (RD0-RD7) --- Unused
    ANSELD = 0x00; // Disable analog function
    WPUD = 0xff; // Week pull up
    TRISD = 0xff; // Set as input

    // Address bus A7-A0 pin (RB0-RB7)
    ANSELB = 0x00; // Disable analog function
    WPUB = 0xff; // Week pull up
    TRISB = 0xff; // Set as input

    // Data bus D7-D0 pin (RC0-RC7)
    ANSELC = 0x00; // Disable analog function
    WPUC = 0xff; // Week pull up
    TRISC = 0xff; // Set as input(default)

    // MWR input pin (RE0)
    ANSELE0 = 0; // Disable analog function
    WPUE0 = 1; // Week pull up
    TRISE0 = 1; // Set as input

    // RESET output pin (RE1)
    ANSELE1 = 0; // Disable analog function
    LATE1 = 0; // Reset
    TRISE1 = 0; // Set as output

    // INT output pin (RE2)) --- Unused
    ANSELE2 = 0; // Disable analog function
    LATE2 = 1; // No interrupt request
    TRISE2 = 0; // Set as output

    // N0(RA0) input pin --- Unused
    ANSELA0 = 0; // Disable analog function
    WPUA0 = 1; // Week pull up
    TRISA0 = 1; // Set as input

    // MRD(RA5)  input pin
    ANSELA5 = 0; // Disable analog function
    WPUA5 = 1; // Week pull up
    TRISA5 = 1; // Set as intput

    // clock(RA3) by NCO FDC mode
    RA3PPS = 0x3f; // RA3 asign NCO1
    ANSELA3 = 0; // Disable analog function
    TRISA3 = 0; // PWM output pin
    NCO1INCU = (unsigned char)((Z80_CLK*2/61/65536) & 0xff);
    NCO1INCH = (unsigned char)((Z80_CLK*2/61/256) & 0xff);
    NCO1INCL = (unsigned char)((Z80_CLK*2/61) & 0xff);
    NCO1CLK = 0x00; // Clock source Fosc
    NCO1PFM = 0;  // FDC mode
    NCO1OUT = 1;  // NCO output enable
    NCO1EN = 1;   // NCO enable

    // UART3 initialize
    //U3BRG = 416; // 9600bps @ 64MHz
    //U3RXEN = 1; // Receiver enable
    //U3TXEN = 1; // Transmitter enable

    // UART3 Receiver
    //ANSELA7 = 0; // Disable analog function
    //TRISA7 = 1; // RX set as input
    //U3RXPPS = 0x07; //RA7->UART3:RX3;

    // UART3 Transmitter
    //ANSELA6 = 0; // Disable analog function
    //LATA6 = 1; // Default level
    //TRISA6 = 0; // TX set as output
    //RA6PPS = 0x26;  //RA6->UART3:TX3;

    //U3ON = 1; // Serial port enable
    
    // CLK(RA1) CLC input pin --- FF-CLK
    ANSELA1 = 0; // Disable analog function
    WPUA1 = 1; // Week pull up
    TRISA1 = 1; // Set as input
    CLCIN0PPS = 0x1; //RA1->CLC1:CLCIN0;

    // TPB(RA2) CLC input pin --- FF-D
    ANSELA2 = 0; // Disable analog function
    WPUA2 = 1; // Week pull up
    TRISA2 = 1; // Set as input
    CLCIN1PPS = 0x2; //RA2->CLC1:CLCIN1;

    // WAIT(RA4) CLC output pin --- Unused
    ANSELA4 = 0; // Disable analog function
    LATA4 = 1; // Default level
    TRISA4 = 0; // Set as output
    RA4PPS = 0x01;  //RA4->CLC1:CLC1;

    //C logic configuration
    CLCSELECT = 0x0; // Use only 1 CLC instance  
    CLCnPOL = 0x82; // LCG2POL inverted, LCPOL inverted

    // CLC data inputs select
    CLCnSEL0 = 0; // D-FF CLK assign CLCIN0PPS(RA1)
    CLCnSEL1 = 1; // D-FF D assign CLCIN1PPS(RA2) 
    CLCnSEL2 = 127; //127; // D-FF S assign none
    CLCnSEL3 = 127; // D-FF R assign none

    // CLCn gates logic select
    CLCnGLS0 = 0x1; // 0x1 Connect LCG1D1N 
    CLCnGLS1 = 0x4; // 0x4 Connect LCG2D2N
    CLCnGLS2 = 0x0; // Connect none
    CLCnGLS3 = 0x0; // Connect none

    CLCDATA = 0x0; // Clear all CLC outs
    CLCnCON = 0x8c; // Select D-FF, falling edge inturrupt

    // Vectored Interrupt Controller setting sequence
    INTCON0bits.IPEN = 1; // Interrupt priority enable

    IVTLOCK = 0x55;
    IVTLOCK = 0xAA;
    IVTLOCKbits.IVTLOCKED = 0x00; // Unlock IVT

    IVTBASEU = 0;
    IVTBASEH = 0;
    IVTBASEL = 8; // Default VIT base address

    IVTLOCK = 0x55;
    IVTLOCK = 0xAA;
    IVTLOCKbits.IVTLOCKED = 0x01; // Lock IVT

    // CLC VI enable
    CLC1IF = 0; // Clear the CLC interrupt flag
    CLC1IE = 1; // Enabling CLC1 interrupt

    // INT0 VI enable
    INT0PPS = 0x1; //RA1->INTERRUPT MANAGER:INT0;
    INT0EDG = 1; // Rising edge
    INT0IE = 1;

    // COSMAC start
    GIE = 1; // Global interrupt enable
    LATE1 = 1; // Release reset

    while(1); // All things come to those who wait
}

// blink program 1
//00  7B               1->Q
//01  7A               0->Q
//02  30 00            00->R(P).0

// Counter program 2
//0000 F8 81             8 (   2) START   LDI     #IOR    ;D <- #IOR
//0002 A3                9 (   2)         PLO     3       ;R(3).0 <- D
//0003 E3               10 (   2)         SEX     3       ;X <- 3
//0004 84               11 (   2) LOOP1   GLO     4       ;D <- R(4).1
//0005 53               12 (   2)         STR     3       ;M(R(3)) <- D
//0006 61               13 (   2)         OUT     1       ;BUS <- M(R(3)); R(3)++
//0007 23               14 (   2)         DEC     3       ;R(3)--
//0008 14               15 (   2)         INC     4       ;R(4)++
//0009 30 04            16 (   2)         BR      LOOP1   ;Branch to LOOP1
//000B                  17        *
//0081 00               18        IOR     .DB     0       ;IO Register


const unsigned char rom[ROM_SIZE] = {
    // OSC ($0000-$007F)

	// Program 1: Blink
    //0x7b, 0x7a, 0x30, 0x00, 0xff, 0xff, 0xff, 0xff, // 0000

    // Program 2: Counter
    0xf8, 0x81, 0xa3, 0xe3, 0x84, 0x53, 0x61, 0x23, // 0000
	0x14, 0x30, 0x04, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, // 0010
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, // 0020
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, // 0030
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, // 0040
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, // 0050
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, // 0060
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, // 0070
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff
};
