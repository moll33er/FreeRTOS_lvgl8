#include "stdio.h"

#pragma import(__use_no_semihosting_swi)
#pragma import(__use_no_semihosting) 

void _sys_exit(int x) { 
    x = x; 
} 

struct __FILE  { 
    int handle; 
    /* Whatever you require here. If the only file you are using is */ 
    /* standard output using printf() for debugging, no file handling */ 
    /* is required. */ 
}; 
/* FILE is typedef¡¯ d in stdio.h. */ 
FILE __stdout;