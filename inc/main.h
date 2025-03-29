#ifndef _MAIN_H_
#define _MAIN_H_

#ifdef __cplusplus
extern "C" {
#endif

/////////////////// INCLUDES /////////////////////
#include <stdio.h>
#include "pico/stdlib.h"
#include "lvgl.h"
#include "lcd/bsp_lcd.h"

#define auto 1
#define manual 0
#define AUTO_MANUAL auto


#if (AUTO_MANUAL == auto) 
    #include <stdio.h>
    #include "pico/stdlib.h"
#else
    #include <stdio.h>
    #include "pico/stdlib.h"
#endif


#ifdef __cplusplus
}
#endif

#endif  // _MAIN_H_
