# STM32F103_NEC_DECODER
Project for decoding NEC IR protocol with interrupt to avoid blocking code.

## Material list

1 blue pill STM32F103C8 or STM32F1 family microcontroller  
1 IR receiver  
1 8x8 LED Matrix Drived with MAX7219 (optional)  

## MCU Pinout 

![Schematics](/Pictures/CubeMX_Pinout.PNG)

RCC_OSC_IN & RCC_OSC_OUT 	= 8MHz Oscillator  
LED 				= Led for indicator  
SPI1\* and CS pins 		= MAX7219  
SYS_JT\*			= debug  

## Little guide of usage 

For display debug (to see timing between rinsing/falling edges) in the Virtual port com (USB)  
uncomment '#define DEBUG' in the main.c file.

For display the result of decoded NEC data in MAX7219 please connect the MAX7219 matrix (see schematics below) and  
uncomment '#define MAXIMDISP' int the main.c file.

For display the result of decoded NEC data in the Virtual port com (USB)  
uncomment '#define VPCOM' in the main.c file.

## Future features and bug to resolve

- [ ] Add pseudo-CRC control (but some remotes don't use the real NEC protocol)
- [ ] Delete data when time out to repeat
- [ ] IR receiver VS1838B has a bug when it's don't detect any IR code for long time see note at end (coming soon)

## Libararies and programs used to init project

SMT32CUBEMX was used to initialise the project code, please open the .ioc file to find my microcontroller configuration.  
All libraries used in this project are from STM32CUBEMX from STMicroelectronics.
