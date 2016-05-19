#include <p33fxxxx.h>		
#include "vs1053.h"	
#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <stdbool.h>
#include <xc.h>

#include "VS-SPI.h"

#define SD_CS   PORTCbits.RC1
#define SPISTAT SPI1STAT

/**********************************************************
                       Functions
**********************************************************/
unsigned char displayTitle[5];

void vs1053Init(void)
{
Delayms(1);
	vs1053SCIWrite(SCI_MODE, SM_SDINEW | SM_TESTS); 
Delayms(1);
	vs1053SCIWrite(SCI_CLOCKF, (SC_DOUBLECLK | SC_INCLK));	
}

//Usage: vs1053Config()
//Inputs: None
//Outputs: None
//Description: Configures the PIC32 for communication with the mp3 player and sdcard
void vs1053Config(void)
{

   // SPI2BRG = SPICalutateBRG(GetPeripheralClock(), 400000); // Fpb/(2*(99+1))= 80/200 = 0,4 MHz
   // SPI2CON = 0x8120; // ON, CKE=1; CKP=0, sample middle
   //OpenSPI2(SPI_MODE8_ON|FRAME_ENABLE_OFF|SPI_SMP_OFF|SPI_CKE_OFF|SLAVE_ENABLE_OFF|CLK_POL_ACTIVE_HIGH|MASTER_ENABLE_ON|SEC_PRESCAL_2_1|PRI_PRESCAL_64_1,SPI_ENABLE);
	TRIS_SCI=0;
	TRIS_SDI=0;
	TRIS_XRST=0;	
	TRIS_XDREQ=1;

	X_SCI=1;
	X_SDI=1;
	X_RST=0;
    SD_CS = 1;
}

//Usage: vs1053Finish();
//Inputs: None
//Outputs: None
//Description: Relinquishes control of the SPI lines to the MP3 player
void vs1053Finish(void)
{
	SPISTAT &= 0x7FFF;		// Relinquish SPI pin connections
}

//Usage: register_value = vs1053SCIRead(SCI_MODE);
//Inputs: unsigned char address - Address of the register to be read
//Ouputs: None
//Description: Returns the value of the vs1053 register defined by "address"
unsigned short int vs1053SCIRead(unsigned char address)
{
	unsigned short int temp;
    X_SCI=0;

	WriteSPIM1(READ_COMMAND);
	WriteSPIM1(address);
    temp = MDD_SDSPI_ReadMedia1();
   	temp <<= 8;
   	temp |= MDD_SDSPI_ReadMedia1();
	
    X_SCI=1;
    Delayms(1);

	return temp;
}

//Usage: vs1053SCIWrite(SCI_MODE, SM_SDINEW);
//Inputs: unsigned char address - Adress of the register to be written to
//		  unsigned short int data - Data to write to the register
//Outputs: None
//Description: Writes "data" to the register defined in "address"
void vs1053SCIWrite(unsigned char address, unsigned short int data)
{	
    X_SCI=0;

	WriteSPIM1(WRITE_COMMAND);
	WriteSPIM1(address);
	WriteSPIM1(data >> 8);						// Send High Byte of data
	WriteSPIM1(data & 0x00ff);					// Send Low Byte of data

    X_SCI=1;
    Delayms(1);
}

//Usage: vs1053SineTest(126);
//Inputs: unsigned char pitch - pitch of the sine wave to be produced
//Outputs: None
//Description: Runs the Sine Test defined in the vs1053 datasheet
void vs1053SineTest(unsigned char pitch)
{
    X_SDI=0;
    
	WriteSPIM1(0x53);
	WriteSPIM1(0xEF);
	WriteSPIM1(0x6E);
   	WriteSPIM1(pitch);						//Send the Pitch	
	WriteSPIM1(0);
	WriteSPIM1(0);
	WriteSPIM1(0);
	WriteSPIM1(0);

    X_SDI=1;
}

void vs1053MemoryTest(void)
{ 
    X_SDI=0;

	WriteSPIM1(0x4d);
	WriteSPIM1(0xea);
   	WriteSPIM1(0x6D);							
	WriteSPIM1(0x54);
	WriteSPIM1(0);
	WriteSPIM1(0);
	WriteSPIM1(0);
	WriteSPIM1(0);

    X_SDI=1;
    Delayms(150);
}

//Usage: vs1053Mute();
//Inputs: None
//Outputs: None
//Description: Mutes the output of the vs1053 MP3 player
void vs1053Mute(void)
{
    vs1053SCIWrite(SCI_VOL, SV_MUTE);
}

//Usage: vs1053SetVolume(INCREASE);
//Inputs: char setting - either INCREASE(1) or DECREASE(0)
//Outputs: None
//Description: Increases or decreases the volume of the MP3 player based on the value of "setting"
void vs1053SetVolume(char setting)
{
	unsigned short int currentVolume=0;
	currentVolume = vs1053SCIRead(SCI_VOL);
	if(setting == INCREASE)
	{
		if(currentVolume == SV_MAX_VOLUME);	//Don't Change the current volume
		else currentVolume -= 0x0808;		//Increment both channels equally
	}
	else{
		if(currentVolume == SV_MUTE);			//Don't change the current volume
		else currentVolume += 0x0808;
	}
	vs1053SCIWrite(SCI_VOL, currentVolume);
}

//Usage: vs1053SendMusic(bufferedSongData, BUFFERSIZE);
//Inputs: unsigned char* songData - pointer to MP3 data that is to be sent to mp3 player
//		  int buffer_size - size of song data in bytes
//Outputs: None
//Description: Sends the data in the songData buffer to the MP3 player
void vs1053SendMusic(unsigned char* songData, int buffer_size)
{	
    int i;
    X_SDI=0;
    for (i=0; i<buffer_size; i++)
    {
        WriteSPIM1(*songData++);			//Send the buffered byte of data, then increment the buffer position
    }
    X_SDI=1;
}

//Usage: vs1053Reset();
//Inputs: None
//Outputs: None
//Description: Resets the mp3 player
void vs1053Reset(void)
{
X_RST=0; //Reset the vs1053
Delayms(10);//Hold Reset
X_RST=1; //Bring vs1053 out of reset
}

void ScreenSetPlayTime(void){
	unsigned int playTime;
	unsigned char minutes, seconds;
	

    playTime = vs1053SCIRead(SCI_DECODE_TIME);
    minutes = playTime/60;
    seconds = playTime%60;
    displayTitle[0]=('0'+minutes/10);
    displayTitle[1]=('0'+minutes%10);
    displayTitle[2]=(':');
    displayTitle[3]=('0'+seconds/10);
    displayTitle[4]=('0'+seconds%10);
}

void SendZerosToVS1053(void){
    int temp;
    X_SDI=0; 
    for (temp=0; temp<1048; temp++)
	{ 
        while(X_DREQ!=1);
        WriteSPIM1(0);
    }
    X_SDI=1;
}


