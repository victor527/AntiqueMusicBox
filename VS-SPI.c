#include "FSDefs.h"
#include "Compiler.h"
#include "HardwareProfile.h"
#include "VS-SPI.h"

// Configuration of SD card input and output
#define SD_CS_TRIS  TRISCbits.TRISC1
#define SD_CS   PORTCbits.RC1
#define SD_CD_TRIS  TRISCbits.TRISC2
#define SD_CD   PORTCbits.RC2
#define SD_WE_TRIS  TRISCbits.TRISC3
#define SD_WE   PORTCbits.RC3

#define SPISTAT_RBF SPI1STATbits.SPIRBF
#define SPISTAT_TBF SPI1STATbits.SPITBF
#define SPIBUF  SPI1BUF
#define SPISTAT SPI1STAT
#define SPICON1 SPI1CON1

#define SPIIN TRISFbits.TRISF7
#define SPIOUT TRISFbits.TRISF8

#define SPIENABLE   SPI1STATbits.SPIEN
#define SPICLOCK    SPI1CON1bits.DISSCK


/******************************************************************************
 * Prototypes
 *****************************************************************************/
BYTE MDD_SDSPI_ReadMedia1(void);

unsigned char WriteSPIM1( unsigned char data_out );

/*****************************************************************************
  Function:
    unsigned char WriteSPIM (unsigned char data_out)
  Summary:
    Writes data to the SD card.
  Conditions:
    None.
  Input:
    data_out - The data to write.
  Return:
    0.
  Side Effects:
    None.
  Description:
    The WriteSPIM function will write a byte of data from the microcontroller to the
    SD card.
  Remarks:
    None.
  ***************************************************************************************/
unsigned char WriteSPIM1( unsigned char data_out )
{
    BYTE   clear;
    SPIBUF = data_out;          // write byte to SSP1BUF register
    while(SPISTAT_TBF == 1);
    while(SPISTAT_RBF == 0);      // wait until bus cycle complete
    clear = SPIBUF;
    return ( 0 );               // return non-negative#
}


/*****************************************************************************
  Function:
    BYTE MDD_SDSPI_ReadMedia (void)
  Summary:
    Reads a byte of data from the SD card.
  Conditions:
    None.
  Input:
    None.
  Return:
    The byte read.
  Side Effects:
    None.
  Description:
    The MDD_SDSPI_ReadMedia function will read one byte from the SPI port.
  Remarks:
    This function replaces ReadSPI, since some implementations of that function
    will initialize SSPBUF/SPIBUF to 0x00 when reading.  The card expects 0xFF.
  ***************************************************************************************/
BYTE MDD_SDSPI_ReadMedia1(void)
{
    SPIBUF = 0xFF;                              //Data Out - Logic ones
    while(SPISTAT_RBF == 0);                     //Wait until cycle complete
    return(SPIBUF);                             //Return with byte read
}
