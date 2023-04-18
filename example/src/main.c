/**
 * @file main.c
 * @author Piotr UjemnyGH Plombon
 * @brief Simple example of usage stm32l4xx.h library (that`s what I have and can test program on), to start new project copy linker_conf.ld and init.c, also you can use my mk file, it`s simple yet effective while testing.
 * @version 0.1
 * @date 2023-04-18
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#define STM32L4
#include "../stm32/stm32.h"

void delay(word_t delay) {
    while(delay--) asm("nop");
}

int main() {
    rcc->ahb2_periph_clk_enable |= rcc_enable_clk(RCC_AHB2_GPIO_C);
    set_gpio_pin_mode(gpio_c, 3, GPIO_OUTPUT);
    set_gpio_pin_mode(gpio_c, 0, GPIO_OUTPUT);
    set_gpio_pin_mode(gpio_c, 1, GPIO_INPUT);
    set_gpio_pull_up_pull_down(gpio_c, 1, GPIO_PULL_DOWN);

    while (1) {
        if(get_gpio_input_value(gpio_c, 1)) {
            set_gpio_output_value(gpio_c, 3, HIGH);
            set_gpio_output_value(gpio_c, 0, LOW);
        }
        else {
            set_gpio_output_value(gpio_c, 3, LOW);
            set_gpio_output_value(gpio_c, 0, HIGH);
        }
    }
}