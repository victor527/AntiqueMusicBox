# AntiqueMusicBox

The code is running under MPLAB environment with dspic33 of Microchip. 
You can find a homepage and a demo here: https://engineering.purdue.edu/ece477/Archive/2015/Fall/477grp8/index.html

![IMG_5098](/Users/Victor/Desktop/IMG_5098.jpg)

##### Description of the function

The AMBmain.c is the file that construct the main function and an interrupt function that aims for multithread on one single chip. The interrupt function is used to continue streaming music while monitor the operations of smartphone as well as showing song information on the VFD. In the main function, there is a while loop that use to continue playing music in the SD card and monitoring the Bluetooth module.

The configuration_pins.c/.h are the files that we use to configure all the pins on the PIC33. In this case, we can make the code easier to read.

The VFDdisplayMessage.c/.h are the files that configure the Vacuum Florescent Display and we map the ASCII into the pins that the VFD has, which is 22 pins. The VFDbar.c/.h are also doing the same job to configure the VFD bar display. In the AMDmain, the function send message directly to the VFD or the VFD bar.

The SD-SPI.c/.h, FSIO.c/.h, FSDefs.h are used to transmitted data from SD card. The AMBmain would call these functions to retrieve song names and the whole song information.



##### The Antique Music Box Functionalities

1.Pair with any Android device with Bluetooth and play music 
An android application will be developed and installed in the Android device. Thus, the android device will be able to connect with the Bluetooth module RN-52 built inside the music box and transmit music files over the air to the microcontroller. Bluetooth module uses the UART interface to communicate. 

2.Display the audio spectrum bar graph on IV-25 VFD tubes and magnitude tubes.
The microcontroller will display the bar graph on IV-25 tubes based on different frequencies as well as the magnitude on magnitude tubes.

3.Display song name on the VFD tubes.
When the SD card is plugged in and streaming songs through SD card. Song name will be sent to the VFD driver chip and characters will be shifted and displayed by a clock signal along with song information.

4.Play music directly from a SD card.
Our device includes a SD card slot that will enable users to play music even without an Android device. The user can choose between SD card mode and Bluetooth mode from the Android cellphone.



This was a group project that completed by four of us.

Yixing Liu - Team Leader and Embedded Software Engineer

Zhiyi Xiang - Embedded Software Engineer

Ran An - Android Software Engineer

Kejian Lin - Hardware Engineer