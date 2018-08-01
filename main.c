/* 

    0x4002 2800 - 0x4002 2BFF GPIOK
    0x4002 2400 - 0x4002 27FF GPIOJ
    0x4002 2000 - 0x4002 23FF GPIOI
    0x4002 1C00 - 0x4002 1FFF GPIOH
    0x4002 1800 - 0x4002 1BFF GPIOG
    0x4002 1400 - 0x4002 17FF GPIOF
    0x4002 1000 - 0x4002 13FF GPIOE
    0x4002 0C00 - 0x4002 0FFF GPIOD
    0x4002 0800 - 0x4002 0BFF GPIOC
    0x4002 0400 - 0x4002 07FF GPIOB
    0x4002 0000 - 0x4002 03FF GPIOA

 * led's for the stem32f469NI
 * NB: They are active low
 *  ------------------------
 *  LED     Port    Color
 *  ________________________
 *  LD1     PG6     Green
 *  LD2     PD4     Orange
 *  LD3     PD5     Red
 *  LD4     PK3     Blue
 *  ------------------------
 *
 *  Base Addresses
 *  ------------------------------------
 *  Port    Address Ranges
 *  ____________________________________
 *  PG     0x4002_1800 --> 0x4002_1BFF
 *  PD     0x4002_0C00 --> 0x4002_0FFF
 *  PK     0x4002_2800 --> 0x4002_2BFF
 *  ------------------------------------
 */
#include<stdint.h>

uint32_t counter = 1;
	
	
	
int main(void) {
	
	uint32_t *myIntPtr = &counter;
	
	while(*myIntPtr < 8) {
	  ++(*myIntPtr);
 
	
	uint32_t *beefPtr = (uint32_t *)0x20000028;
		*beefPtr = 0x1234BEEF;
	}
}

