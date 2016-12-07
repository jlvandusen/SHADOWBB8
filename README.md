# SHADOWBB8
James Bruton's BB8 controller code converted to SHADOW code for use with PS3 Navigation controllers (pair).
This code is a work in progress, although all functions have been implemented.  

Further work will entail:
- Add Disco Droid control mappings to button combinations for sounds
- Add Ps3 sixaxis controller configuration
- Add PS4 sixaxis controller configuration
- add mp3trigger controls
- Add Dome Automation routines

NOTE: This is my personal code for S.H.A.D.O.W. (Small Handheld Arduino Droid Operating Wand) that I use to control my BB8 droid.

Note: You will need a Arduino Mega 1280/2560 to run this sketch,
as a normal Arduino (Uno, Duemilanove etc.) doesn't have enough SRAM and FLASH

The arduino mega 2560 adk is the preferred method as it will involve less hardware

This is written to be a UNIVERSAL Sketch - supporting multiple controller options
for the James Bruton BB8 build version 3.. in replacement of his custom controller
see: www.youtube.com/user/jamesbruton and www.xrobots.co.uk
learn more at http://jimmyzsbb8.blogspot.com or http://jimmyzsr2.blogspot.com

- Single PS3 Move Navigation
- Pair of PS3 Move Navigation controllers (recommended)
- Bluetooth connected Phone (Limited Controls)
- Disco Droid Support (Audio sound support)
//      - JLV: Please do not use Digital pin 7 as input or output because is used in the comunication with MAX3421E
//      - JLV: on the arduino mega ADK model. 
//      - JLV: Digital: 7 (RST), 50 (MISO), 51 (MOSI), 52 (SCK).
//      - JLV: https://www.arduino.cc/en/Main/ArduinoBoardMegaADK
