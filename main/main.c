#include "stm32f411xe.h"
#include "gpio.h"
#include <stdint.h>


void sys_clk_init(void);

int main() {

    sys_clk_init();
    gpio_init();

    while (1) {

    }
}

void sys_clk_init(void) {

}

// Defined here and used in startup code
void SystemInit(void) {

}
