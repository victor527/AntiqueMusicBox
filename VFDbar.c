/*
 * File:   VFDdisplayMessage.c
 * Author: ECE477 Team8 Antique Music Box
 * This file display the message on VFD for the Antique Music Box
 * Created on October 31, 2015, 12:00 AM
 */

#include <xc.h>
#include "p33Fxxxx.h"
#include "configuration_pins.h"
#include "VFDbar.h"
#include <math.h>

void BARshiftout(int,int);
void BARdelay(int);


/* How to use the function
 * long data[10][2] = {leftdata of bar,rightdata of bar};
 * use the data FFT create but truncate the data from 16 into 8
 * use a for loop to input the data from FFT into this data type.
 * convert 2 into 1, for high freq should be zero for most of the time
 */
void VFDbar(long * data, int status)
{
    int i;
    VFD_BLANK = 0;    // set blank as low
    long data1[10] = {rand() % 7,rand() % 7,rand() % 7,rand() % 7,rand() % 7,rand() % 7,rand() % 7,rand() % 7,7- rand() % 3,7- rand() % 2};
    long data0[10] = {7,7,7,7,7,7,7,7,7,7};
    long data2[10] = {0,0,0,0,0,0,0,0,0,0};
    if(status == 1)
    {
    for(i = 0;i < 10; i++)
    {
        BARshiftout(data1[i],data1[i+1]);
        BAR_LATCH = 1;
        BAR_LATCH = 0;
        i++;
    }
    }
    if(status == 2)
    {
            for(i = 0;i < 10; i++)
    {
        BARshiftout(data2[i],data2[i+1]);
        BAR_LATCH = 1;
        BAR_LATCH = 0;
        i++;
    }
        
    }
    if(status == 0)
    {
            for(i = 0;i < 10; i++)
    {
        BARshiftout(data0[i],data0[i+1]);
        BAR_LATCH = 1;
        BAR_LATCH = 0;
        i++;
    }
       
    }
}

void BARdelay(int n)
{
    int i;
    int index;
    for(i=0;i<20000;i++)
    {
        for(index=0;index<n;index++);
    }
}

void BARshiftout(int left_data,int right_data)
{
    long barleft[8][7]={
        {1,1,1,1,1,1,1}, //full on
        {1,1,1,1,1,1,0}, //1 light out
        {1,1,1,1,1,0,0}, //2 light out
        {1,1,1,1,0,0,0},
        {1,1,1,0,0,0,0},
        {1,1,0,0,0,0,0},
        {1,0,0,0,0,0,0},
        {0,0,0,0,0,0,0}, //blank
    };
    
    long barright[8][7]={
        {1,1,1,1,1,1,1},
        {1,1,1,1,1,1,0},
        {1,1,1,1,1,0,0},
        {1,1,1,1,0,0,0},
        {1,1,1,0,0,0,0},
        {1,1,0,0,0,0,0},
        {1,0,0,0,0,0,0},
        {0,0,0,0,0,0,0},
    };
    
    long mid[1][6]={
        {0,0,0,0,0,0},
    };

    int i;
    int var = 0;

    for (i=6; i>=0; i--)  {
        var = barright[right_data][i];
        BAR_DATA = var;
        BAR_CLOCK = 0;
        BAR_CLOCK = 1;
        BAR_DATA = 0;
    }
    
    for (i=5; i>=0; i--)  {
        var = mid[0][i];
        BAR_DATA = var;
        BAR_CLOCK = 0;
        BAR_CLOCK = 1;
        BAR_DATA = 0;
    }
    
    for (i=6; i>=0; i--)  {
        var = barleft[left_data][i];
        BAR_DATA = var;
        BAR_CLOCK = 0;
        BAR_CLOCK = 1;
        BAR_DATA = 0;
    }
}