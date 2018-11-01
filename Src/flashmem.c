#include "stm32f4xx.h"

//#pragma location=0x800C000  
//volatile const uint8_t flash_data[2048];

const uint8_t MyFlashPage[2048] __attribute__((section(".ARM.__at_0x0800C000")));

