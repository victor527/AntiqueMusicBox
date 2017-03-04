/* 
 * File:   AMBmain.c
 * Author: Yixing Liu, Zhiyi Xiang, Ran An, Kejian Lin
 * Description: This code is for the Antique Music Box that utilize Reading SD 
 * card through SPI module and connect with Smartphone in Bluetooth through UART
 * module
 *
 * Created on November 22, 2015, 3:52 PM
 */

#include "p33fxxxx.h"

#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <stdbool.h>
#include <xc.h>
#include <math.h>
#include <dsp.h>

#include "configuration_pins.h"
#include "VFDdisplayMessage.h"
#include "VFDbar.h"
#include "SD-SPI.h"
#include "FSIO.h"
#include "vs1053.h"
#include "ADC.h"

// External Oscillator

_FOSCSEL(FNOSC_FRC);		
_FOSC(FCKSM_CSECMD & OSCIOFNC_OFF  & POSCMD_HS);  
								// Clock Switching is enabled and Fail Safe Clock Monitor is disabled
								// OSC2 Pin Function: OSC2 is Clock Output
								// Primary Oscillator Mode: XT Crystal


_FWDT(FWDTEN_OFF);              // Watchdog Timer Enabled/disabled by user software
								// (LPRC can be disabled by clearing SWDTEN bit in RCON register
#define True 1
#define False 0

#define MAXBUFFERSIZE 32
   
SearchRec file;
unsigned char attributes = ATTR_HIDDEN | ATTR_SYSTEM | ATTR_READ_ONLY | ATTR_VOLUME| ATTR_ARCHIVE;
char extention[] = "*.mp3";
int findfile = 0;   
FSFILE * pointer; 
unsigned char section[MAXBUFFERSIZE];
unsigned int timer_counter = 0;

unsigned short int currentVolume, scibass=0, memtest=0, clock=0, scimode=0, scistatus=0, hdat0=0, hdat1=0, samplerate=0;

void InitClock(void);
void InitSPI(void);
void Init_Timer1( void );
void Init_Timer2 (void);
void InitUART1(void);
void ws2812_send(ws2812_ptr);


volatile int timer1_counter = 0;
volatile int timer2_counter = 0;
volatile int tick = 0;
int second=0;
int minute=0;
int hour=0;
int song_is_over = 0;
int SD = 0;
int BT = 0;
char RECEIVE = 0;
int play = 0;
int stop = 0;
int next = 0;
int first = 0;

int index = 50000;
char str[4] = {0};
long data[10] = {1,2,3,4,5,6,7,7,6,5};

unsigned long int bitflip(unsigned char b);
typedef struct {
    unsigned char   r;
    unsigned char   b;
    unsigned char   g;
} ws2812_ptr;
ws2812_ptr led;



#define FCY 9000000
#define BAUDRATE 115200
#define BRGVAL FCY/BAUDRATE/4 - 1


void __attribute__((__interrupt__, no_auto_psv)) _T1Interrupt(void)
{
    /* Interrupt Service Routine code goes here */
    IFS0bits.T1IF = 0;  //Clear Timer1 interrupt flag
    T1CONbits.TON = 0;  // disable timer

    timer1_counter ++;
    if(timer1_counter == 1000)
    {
        LATAbits.LATA13 = !LATAbits.LATA13;
        second = second + 1;
        if(second == 60)
        {
            minute = minute + 1;
            second = 0;
        }
        if(minute == 60)
        {
            hour = hour + 1;
            minute = 0;
        }
        if(hour == 12 && minute==60)
        {
            hour = 0;
            minute = 0;
            second = 0;
        }
        timer1_counter = 0;
    }
       
    TMR1 = 0;   
    T1CONbits.TON = 1;
}
    

void __attribute__((__interrupt__, no_auto_psv)) _T2Interrupt(void)
{
/* Interrupt Service Routine code goes here */
    IFS0bits.T2IF = 0;     //Clear Timer1 interrupt flag
    T2CONbits.TON = 0;  // disable timer

    if(play == 1)
    {
        if(index < 25000)
            VFDbar(data,1);
        else
            VFDbar(data,1);
        index--;
        if(index == 0)
            index = 50000;
    }

    if(song_is_over == 1 || next == 1)
    {
       if(index < 25000)
          VFDbar(data,0);
       else
          VFDbar(data,0);
       index--;
       if(index == 0)
           index = 50000;
    }
    
    if(BT == 1)
    {
         if(index < 25000)
            VFDbar(data,2);
        else
            VFDbar(data,2);
        index--;
        if(index == 0)
            index = 50000;
    }
    
    TMR2 = 0;   
    T2CONbits.TON = 1;
}

void __attribute__ ((interrupt, no_auto_psv)) _U1RXInterrupt(void)
{
    if(U1STAbits.OERR == 1)
    {
        U1STAbits.OERR = 0;
    }
    RECEIVE = U1RXREG;

    {
        if(RECEIVE == 'c')
        {
            
            VFDdisplayMessage(file.filename);
            if(play == 0)
            {
                play = 1;
            }
            else
            {
                play = 0;
            }
        }

        if(RECEIVE == 'd')
        {
            next = 1;
        }

        if( RECEIVE == 'a')
        {
            SD = 1;
            BT = 0;
            mux1 = 0;
            mux2 = 0;   // mux1 = mux2 = 0 = SD card
            VFDdisplayMessage("SD mode");

        }
        
        if( RECEIVE == '1')
        {
           BT = 1; 
           SD = 0;
           play = 0;
           next = 0;
           mux1 = 1;
           mux2 = 1;   // mux1 = mux2 = 1 = BT
           
        }
        if( RECEIVE == '3')
        {
           BT = 1; 
           SD = 0;
           play = 0;
           next = 0;
           mux1 = 1;
           mux2 = 1;   // mux1 = mux2 = 1 = BT
           VFDdisplayMessage("Youtube");
        }
        
        if( RECEIVE == '5')
        {
           BT = 1; 
           SD = 0;
           play = 0;
           next = 0;
           mux1 = 1;
           mux2 = 1;   // mux1 = mux2 = 1 = BT
           VFDdisplayMessage(" Morning  ");
        }
        if( RECEIVE == '6')
        {
           BT = 1; 
           SD = 0;
           play = 0;
           next = 0;
           mux1 = 1;
           mux2 = 1;   // mux1 = mux2 = 1 = BT
           VFDdisplayMessage("Afternoon ");
        }
        
        if( RECEIVE == '7')
        {
           BT = 1; 
           SD = 0;
           play = 0;
           next = 0;
           mux1 = 1;
           mux2 = 1;   // mux1 = mux2 = 1 = BT
           VFDdisplayMessage("Evening");
        }
        
        
    }
    IFS0bits.U1RXIF = 0;
}


int main(void)
{   
    Configure_Pins(); 
    InitClock();
    VFDdisplayMessage("Welcome  ");

//    led.r = 255;
//    led.g = 255;
//    led.b = 255;
//    
//    int try = 0;
//    
//    while(1) 
//    {
//        // timing on the WS2812B is very precise. turn off interrupts before talking
//        //GIE = 0; while (GIE);
//        //for(try = 0;try<9;try++)
//        //{
//        LED_VFD = 0;
//            ws2812_send(&led);
//            LED_VFD = 0;
//            try = 0;
//        //}
//        //delay(100);
//        //GIE = 1; while (!GIE);
//        //delay(10);
//      }
    

    InitUART1();
    Init_Timer1();
    Init_Timer2();
    
    // Peripheral Initialisation
//   	initAdc1();             	// Initialize the A/D converter to convert Channel 5
//	initDma0();					// Initialise the DMA controller to buffer ADC data in conversion order
//	initTmr3();					// Initialise the Timer to generate sampling event to ADC @ 8Khz rate
    
    InitSPI(); 
    vs1053Config();										// Open SPI2 and Set Pin
    vs1053Reset();										// Reset MP3 Player
    vs1053Init();										// initializes the vs1053 mp3 player to "New" modes
    scimode = vs1053SCIRead(SCI_MODE);
    scistatus = vs1053SCIRead(SCI_STATUS);	
    vs1053SCIWrite(SCI_BASS, 0x44);
    scibass = vs1053SCIRead(SCI_BASS);
    clock = vs1053SCIRead(SCI_CLOCKF);
    vs1053SCIWrite(SCI_AUDATA, 0xAC45);
    samplerate = vs1053SCIRead(SCI_AUDATA);

    while(!MDD_MediaDetect())
    {
        VFDdisplayMessage("No media  ");
    }

    FSInit();
    findfile = FindFirst(extention,attributes,&file);
    if ( findfile == 0)
    {
        first = 1;
        pointer = FSfopen(file.filename,"r");  // open current file
    }

    delay(100);
    
    VFDdisplayMessage(" PLAY Song");

    while(1)
    {
        if(play == 1)
        {
            if(first == 1)
            {
                VFDdisplayMessage(file.filename);
                first = 0;
            }
            while(1)
            {
                if(next == 1 || song_is_over == 1)
                {
                    findfile = FindNext(&file);
                    if(findfile != 0)
                    {
                        findfile = FindFirst(extention,attributes,&file);
                    }
                }
                if(findfile == 0)   // file find success
                {
                    if(song_is_over == 1||next == 1)
                    {
                        VFDdisplayMessage(file.filename);
                        pointer = FSfopen(file.filename,"r");  // open current file
                        next = 0;
                        song_is_over = 0;
                    }
                    if(pointer == NULL)
                        while(1);
                    while(FSfeof(pointer) == 0)
                    {
                        FSfread(section,1,MAXBUFFERSIZE,pointer);
                        while(X_DREQ != 1);
                        vs1053SendMusic(section,MAXBUFFERSIZE);
                        if(play == 0 || next == 1)
                            break;
                    }
                    if(FSfeof(pointer) != 0)
                    {    
                        song_is_over = 1;
                    }
                    FSfclose(pointer);
                    SendZerosToVS1053();
                }
                else
                    VFDdisplayMessage("Insert SD");
                if( play == 0 )
                    break;
            }
        }
        if(play == 0)
        {
         
            
//                char time[10]={0};
//                time[0] = hour;
//                time[1] = minute;
//                time[2] =':';
//                time[3] = second;
//                time[4] = '\0';    
//                VFDdisplayMessage(time);
//            if(play == 1)
//                break;
//            
            next = 0;
//            char time[10] = {0};
//            time[4] = 2;
//            time[5] = 3;
//            time[6] = '\0';
//            
         //  VFDdisplayMessage(time);
        }
    }
}

void InitSPI(void) 
{
    SPI1CON1bits.DISSCK = 0; //SCK pin is controlled by SPI module
    SPI1CON1bits.DISSDO = 0; //SDO pin is controlled by SPI module
    SPI1CON1bits.MODE16 = 0; //8-bit mode selected
    SPI1CON1bits.SMP = 0; //sample data in midle of data output time
    SPI1CON1bits.CKE = 1; //positive clock edge selected
    SPI1CON1bits.SSEN = 0; //master mode, SS pin is not used
    SPI1CON1bits.CKP = 0; //clock active high
    SPI1CON1bits.MSTEN = 1; //select master mode
    SPI1CON1bits.SPRE = 0b101; //secondary scale 1:4
    SPI1CON1bits.PPRE = 0b11; //primary scale 1:1
    SPI1STATbits.SPIROV = 0; //clear overflow flag
    SPI1STATbits.SPIEN = 1; //enable SPI module; config SDI, SDO, SCK pins
    SPI1STATbits.SPISIDL = 0; //halt the module in idle state
    SPI1STATbits.SPITBF = 0; //clear TX buffer full flag
    SPI1STATbits.SPIRBF = 0; //clear RX buffer full flag
    
    SPI2CON1bits.DISSCK = 0; //SCK pin is controlled by SPI module
    SPI2CON1bits.DISSDO = 0; //SDO pin is controlled by SPI module
    SPI2CON1bits.MODE16 = 0; //8-bit mode selected
    SPI2CON1bits.SMP = 0; //sample data in midle of data output time
    SPI2CON1bits.CKE = 1; //positive clock edge selected
    SPI2CON1bits.SSEN = 0; //master mode, SS pin is not used
    SPI2CON1bits.CKP = 0; //clock active high
    SPI2CON1bits.MSTEN = 1; //select master mode
    SPI2CON1bits.SPRE = 0b101; //secondary scale 1:4
    SPI2CON1bits.PPRE = 0b11; //primary scale 1:1
    SPI2STATbits.SPIROV = 0; //clear overflow flag
    SPI2STATbits.SPIEN = 1; //enable SPI module; config SDI, SDO, SCK pins
    SPI2STATbits.SPISIDL = 0; //halt the module in idle state
    SPI2STATbits.SPITBF = 0; //clear TX buffer full flag
    SPI2STATbits.SPIRBF = 0; //clear RX buffer full flag
}


      
void InitClock()
{
    PLLFBD = 4;  //
    CLKDIVbits.PLLPOST = 0;    // N1 = 2
    CLKDIVbits.PLLPRE = 0;    // N2 = 2
    OSCTUN = 0;
    RCONbits.SWDTEN = 0;
    // Clock switch to incorporate PLL    NOSC = 0b010
    __builtin_write_OSCCONH(0b011);
    __builtin_write_OSCCONL(0b001);    // Start clock switching
    while (OSCCONbits.COSC != 0b011);    // Wait for Clock switch to occur    
    while(OSCCONbits.LOCK != 1) {};
}

void Init_Timer1(void)
{
    T1CONbits.TON = 0;     // Disable Timer
    T1CONbits.TCS = 0;     // Select internal instruction cycle clock
    T1CONbits.TGATE = 0;   // Disable Gated Timer mode
    T1CONbits.TCKPS = 0b10;// Select 1:256 Prescaler

    TMR1 = 0x00;           // Clear timer register
    PR1 = 164;            

    IPC0bits.T1IP = 0x01;  // Set Timer1 Interrupt Priority Level
    IFS0bits.T1IF = 0;     // Clear Timer1 Interrupt Flag
    IEC0bits.T1IE = 1;     // Enable Timer1 interrupt
    T1CONbits.TON = 1;
}

void Init_Timer2(void)
{
    T2CONbits.TON = 0;     // Disable Timer
    T2CONbits.TCS = 0;     // Select internal instruction cycle clock
    T2CONbits.TGATE = 0;   // Disable Gated Timer mode
    T2CONbits.TCKPS = 0b11;// Select 1:256 Prescaler

    TMR2 = 0x00;           // Clear timer register
    PR2 = 2000;            

    IPC1bits.T2IP = 0x05;  // Set Timer2 Interrupt Priority Level
    IFS0bits.T2IF = 0;     // Clear Timer2 Interrupt Flag
    IEC0bits.T2IE = 1;     // Enable Timer2 interrupt
    T2CONbits.TON = 1;
}


void InitUART1(void)
{
    // This is an EXAMPLE, so brutal typing goes into explaining all bit sets
    // The HPC16 board has a DB9 connector wired to UART1, so we will
    // be configuring this port only
    PMD1bits.U1MD = 0;
    // configure U1MODE
    U1MODEbits.UARTEN = 0;    // Bit15 TX, RX DISABLED, ENABLE at end of func
    //U1MODEbits.notimplemented;    // Bit14
    U1MODEbits.USIDL = 0;    // Bit13 Continue in Idle
    U1MODEbits.IREN = 0;    // Bit12 No IR translation
    U1MODEbits.RTSMD = 0;    // Bit11 Simplex Mode
    //U1MODEbits.notimplemented;    // Bit10
    U1MODEbits.UEN = 0;    // Bits8,9 TX,RX enabled, CTS,RTS not
    U1MODEbits.WAKE = 0;    // Bit7 No Wake up (since we don't sleep here)
    U1MODEbits.LPBACK = 0;    // Bit6 No Loop Back
    U1MODEbits.ABAUD = 0;    // Bit5 No Autobaud (would require sending '55')
    U1MODEbits.URXINV = 0;    // Bit4 IdleState = 1  (for dsPIC)
    U1MODEbits.BRGH = 1;    // Bit3 16 clocks per bit period
    U1MODEbits.PDSEL = 0;    // Bits1,2 8bit, No Parity
    U1MODEbits.STSEL = 0;    // Bit0 One Stop Bit

    U1BRG = BRGVAL;    

    // Load all values in for U1STA SFR
    U1STAbits.UTXISEL1 = 0;    //Bit15 Int when Char is transferred (1/2 config!)
    U1STAbits.UTXINV = 0;    //Bit14 N/A, IRDA config
    U1STAbits.UTXISEL0 = 0;    //Bit13 Other half of Bit15
    //U2STAbits.notimplemented = 0;    //Bit12
    U1STAbits.UTXBRK = 0;    //Bit11 Disabled
    U1STAbits.UTXEN = 0;    //Bit10 TX pins controlled by periph
    U1STAbits.UTXBF = 0;    //Bit9 *Read Only Bit*
    U1STAbits.TRMT = 0;    //Bit8 *Read Only bit*
    U1STAbits.URXISEL = 0;    //Bits6,7 Int. on character recieved
    U1STAbits.ADDEN = 0;    //Bit5 Address Detect Disabled
    U1STAbits.RIDLE = 0;    //Bit4 *Read Only Bit*
    U1STAbits.PERR = 0;    //Bit3 *Read Only Bit*
    U1STAbits.FERR = 0;    //Bit2 *Read Only Bit*
    U1STAbits.OERR = 0;    //Bit1 *Read Only Bit*
    U1STAbits.URXDA = 0;    //Bit0 *Read Only Bit*

    IPC7 = 0x4400;    // Mid Range Interrupt Priority level, no urgent reason
    IFS0bits.U1TXIF = 0;    // Clear the Transmit Interrupt Flag
    IEC0bits.U1TXIE = 0;    // Enable Transmit Interrupts
    IFS0bits.U1RXIF = 0;    // Clear the Receive Interrupt Flag
    IEC0bits.U1RXIE = 1;    // Enable Receive Interrupts
    U1MODEbits.UARTEN = 1;    // And turn the peripheral on
    U1STAbits.UTXEN = 1;
}

// transmit the ws2812 led
void ws2812_send(ws2812_ptr* led) {
    int j;
    int i;
    long int val;
    // the WS2812 wants bits in the order of:
    // GGGGGGGGRRRRRRRRBBBBBBBB
    // but I want to work in the opposite order. so i'm going to flip
    // the bits around and do some shifting so my order is
    // BBBBBBBBRRRRRRRRGGGGGGGG
    // with the most significant bit on the far right. so the RGB value
    // of 128 64 32, which normally would be:
    // R : 0b10000000
    // G : 0b01000000
    // B : 0b00100000
    // will become:
    // BBBBBBBBRRRRRRRRGGGGGGGG
    // 000001000000000100000010
    val = (bitflip(led->b) << 16) + (bitflip(led->r) << 8) + (bitflip(led->g));

    // now begin shifting them over one at a time
    for(i=0;i<8;i++)
    {
        for(j = 0; j < 24; j++) {
            // depending on if the currently viewed bit is 1 or 0
            // the pin will stay high for different times

            if (val & 1 == 1) {
                // if it is a 1, let it stay higher a bit longer
                //LED_BAR = 1;
                LED_VFD = 1;
                i = 0;
                //LED_BAR = 0;
                LED_VFD = 0;
            } else {
                // but a 0 should go high and then low as fast as possible
                //LED_BAR = 1;
                //LED_BAR = 0;
                LED_VFD = 1;
                LED_VFD = 0;
                //i = 0;
            }
        
        // and then right shift to get the next bit
        val = val >> (unsigned char)1;
        }
    }
}

// reverse the bits in a char
unsigned long int bitflip(unsigned char b) {
   b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
   b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
   b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
   return (unsigned long int)b;
}