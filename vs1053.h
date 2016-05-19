/* 
 * File:   vs1053.h
 * Author: ECE477 Team8 Antique Music Box
 *
 * Created on November 6, 2015, 12:16 PM
 */


#ifndef VS1053_H
#define	VS1053_H

#ifdef	__cplusplus
extern "C" {
#endif

/* *********************************************************
               Function declarations
  ********************************************************* */
void vs1053Init(void);		
unsigned short int vs1053SCIRead(unsigned char);
void vs1053SCIWrite(unsigned char, unsigned short int);
void vs1053SendMusic(unsigned char*, int);
void vs1053SineTest(unsigned char);
void vs1053Config(void);
void vs1053Finish(void);
void vs1053Mute(void);
void vs1053SetVolume(char setting);
void vs1053Reset(void);
void vs1053MemoryTest(void);
void ScreenSetPlayTime(void);
void SendZerosToVS1053(void);

/* *********************************************************
              MACRO definitions
  ********************************************************* */
#define TRIS_SCI 	TRISBbits.TRISB4
#define X_SCI   	LATBbits.LATB4

#define TRIS_SDI	TRISBbits.TRISB3
#define X_SDI		LATBbits.LATB3

#define TRIS_XRST	TRISBbits.TRISB5
#define X_RST		LATBbits.LATB5

#define TRIS_XDREQ  TRISBbits.TRISB0
#define X_DREQ	    PORTBbits.RB0

#define READ_COMMAND 0x03
#define WRITE_COMMAND 0x02

/* *********************************************************
              General definitions
  ********************************************************* */
#define EXTERNAL_FREQ 13000000

/* *********************************************************
              vs1053 Registers
  ********************************************************* */
#define SCI_MODE	0x00
#define SCI_STATUS	0x01
#define SCI_BASS	0x02
#define SCI_CLOCKF	0x03
#define SCI_DECODE_TIME	0x04
#define SCI_AUDATA	0x05
#define SCI_WRAM	0x06
#define SCI_WRAMADDR	0x07
#define SCI_HDAT0	0x08
#define SCI_HDAT1	0x09
#define SCI_AIADDR	0x0A
#define SCI_VOL		0x0B
#define SCI_AICTRL0	0x0C
#define SCI_AICTRL1	0x0D
#define SCI_AICTRL2	0x0E
#define SCI_AICTRL3	0x0F

/* *********************************************************
              vs1053 Bit Masks
  ********************************************************* */
//SCI_MODE Bits	
 #define SM_DIFF		0x0001
 #define SM_SETTOZERO	0x0002
 #define SM_RESET		0x0004
 #define SM_OUTOFWAV	0x0008
 #define SM_PDOWN		0x0010
 #define SM_TESTS		0x0020
 #define SM_STREAM		0x0040
 #define SM_PLUSV		0x0080
 #define SM_DACT		0x0100
 #define SM_SDIORD		0x0200
 #define SM_SDISHARE	0x0400
 #define SM_SDINEW		0x0800
 #define SM_ADPCM		0x1000
 #define SM_ADPCM_HP	0x2000

 //SCI_STATUS Bits
 #define SS_VER			0x0070		//Use this as a mask for the Version when reading the Status Register
 //The rest of the Status bits are to be used by the vs1053 FW only
 
 //SCI_CLOCKF Bits
 #define SC_DOUBLECLK	0x8000
 #define SC_INCLK		EXTERNAL_FREQ/2000
 
 //SCI_VOL Settings
 #define 	INCREASE		'1'
 #define 	DECREASE		'0'
 #define 	SV_MUTE			0xFFFF
 #define	SV_MAX_VOLUME	0x0000


#ifdef	__cplusplus
}
#endif

#endif	/* VS1053_H */

