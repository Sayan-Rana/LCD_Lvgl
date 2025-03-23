#include "main.h"


int main()
{
    stdio_init_all();
    lv_init();

    while (true) {
        printf("Hello, world!\n");
        sleep_ms(1000);
    }
}
