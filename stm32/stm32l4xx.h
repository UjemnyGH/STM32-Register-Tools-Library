/**
 * @file stm32l4xx.h
 * @author Piotr UjemnyGH Plombon
 * @brief STM32L4xx library.
 * @version 0.1
 * @date 2023-04-11
 * 
 * @copyright Copyright (c) 2023
 * 
 * Library created from joining stuff from what I found in reference manual 0432.
 * Library shouldn`t have much bugs(I mean entire library contains values and adresess, maybe some 
 * functions for convinience), but still be careful.
 * 
 * Have fun programming STM32L4xx
 * 
 * Peter UjemnyGH Plombon
 */

#ifndef __STM32L4xx_
#define __STM32L4xx_

typedef unsigned int                word_t;
typedef unsigned short              hword_t;
typedef unsigned char               byte_t;

#define REG32(base)                 (*((volatile word_t*)(base)))
#define REG16(base)                 (*((volatile hword_t*)(base)))
#define REG8(base)                  (*((volatile byte_t*)(base)))

#define LOW                         0UL                               /* Output state */
#define HIGH                        1UL                               /* Output state */

#define PERIPH_BASE                 ((word_t)0x40000000UL)          /* Peripheral base adress */

/**
 * 
 * APB1 peripherals
 * 
 */
#define APB1PERIPH_BASE             PERIPH_BASE                     /* APB1 bus peripheral */

#define RTC_BASE                    (APB1PERIPH_BASE + 0x2800UL)


/**
 * 
 * APB2 peripherals
 * 
 */
#define APB2PERIPH_BASE             (PERIPH_BASE + 0x00010000UL)      /* APB2 bus peripheral */

#define DFSDM_BASE                  (APB2PERIPH_BASE + 0x6000UL)

#define SAI1_BASE                   (APB2PERIPH_BASE + 0x5400UL)

#define SAI2_BASE                   (APB2PERIPH_BASE + 0x5800UL)

#define TIM15_BASE                  (APB2PERIPH_BASE + 0x4000UL)

#define TIM16_BASE                  (APB2PERIPH_BASE + 0x4400UL)

#define TIM17_BASE                  (APB2PERIPH_BASE + 0x4800UL)

/**
 * 
 * AHB1 peripherals
 * 
 */
#define AHB1PERIPH_BASE             (PERIPH_BASE + 0x00020000)      /* AHB1 bus peripheral */

#define TSC_BASE                    (AHB1PERIPH_BASE + 0x4000UL)
#define CRC_BASE                    (AHB1PERIPH_BASE + 0x3000UL)
#define FLASH_BASE                  (AHB1PERIPH_BASE + 0x2000UL)
#define RCC_BASE                    (AHB1PERIPH_BASE + 0x1000UL)

#define DMA1_BASE                   AHB1PERIPH_BASE
#define DMA1_CH1_BASE               (DMA1_BASE + 0x0008UL)
#define DMA1_CH2_BASE               (DMA1_BASE + 0x001cUL)
#define DMA1_CH3_BASE               (DMA1_BASE + 0x0030UL)
#define DMA1_CH4_BASE               (DMA1_BASE + 0x0044UL)
#define DMA1_CH5_BASE               (DMA1_BASE + 0x0058UL)
#define DMA1_CH6_BASE               (DMA1_BASE + 0x006cUL)
#define DMA1_CH7_BASE               (DMA1_BASE + 0x0080UL)
#define DMA1_CSELR_BASE             (DMA1_BASE + 0x00a8UL)

#define DMA2_BASE                   (AHB1PERIPH_BASE + 0x400UL)
#define DMA2_CH1_BASE               (DMA2_BASE + 0x0008UL)
#define DMA2_CH2_BASE               (DMA2_BASE + 0x001cUL)
#define DMA2_CH3_BASE               (DMA2_BASE + 0x0030UL)
#define DMA2_CH4_BASE               (DMA2_BASE + 0x0044UL)
#define DMA2_CH5_BASE               (DMA2_BASE + 0x0058UL)
#define DMA2_CH6_BASE               (DMA2_BASE + 0x006cUL)
#define DMA2_CH7_BASE               (DMA2_BASE + 0x0080UL)
#define DMA2_CSELR_BASE             (DMA2_BASE + 0x00a8UL)

/**
 * 
 * AHB2 peripherals
 * 
 */
#define AHB2PERIPH_BASE             (PERIPH_BASE + 0x08000000)      /* AHB2 bus peripheral */

#define GPIOA_BASE                  (AHB2PERIPH_BASE + 0x0000UL)    /* General Purpose I/O A Base */
#define GPIOB_BASE                  (AHB2PERIPH_BASE + 0x0400UL)    /* General Purpose I/O B Base */
#define GPIOC_BASE                  (AHB2PERIPH_BASE + 0x0800UL)    /* General Purpose I/O C Base */
#define GPIOD_BASE                  (AHB2PERIPH_BASE + 0x0c00UL)    /* General Purpose I/O D Base */
#define GPIOE_BASE                  (AHB2PERIPH_BASE + 0x1000UL)    /* General Purpose I/O E Base */
#define GPIOF_BASE                  (AHB2PERIPH_BASE + 0x1400UL)    /* General Purpose I/O F Base */
#define GPIOG_BASE                  (AHB2PERIPH_BASE + 0x1800UL)    /* General Purpose I/O G Base */
#define GPIOH_BASE                  (AHB2PERIPH_BASE + 0x1c00UL)    /* General Purpose I/O H Base */

#define USBOTG_BASE                 (AHB2PERIPH_BASE + 0x8000000UL)
#define ADC1_BASE                   (AHB2PERIPH_BASE + 0x8040000UL)
#define ADC2_BASE                   (AHB2PERIPH_BASE + 0x8040100UL)
#define ADC3_BASE                   (AHB2PERIPH_BASE + 0x8040200UL)
#define ADC123_COMMON_BASE          (AHB2PERIPH_BASE + 0x8040300UL)

#define RNG_BASE                    (AHB2PERIPH_BASE + 0x8060800UL) /* Random number generation base */

/**
 * 
 * General Purpose Input/Output
 * 
 */

typedef struct {
    volatile word_t moder;
    volatile word_t output_type;
    volatile word_t output_speed;
    volatile word_t pull_up_down;
    volatile word_t input_data;
    volatile word_t output_data;
    volatile word_t bit_set_reset;
    volatile word_t config_lock;
    volatile word_t alternate_func_l;
    volatile word_t alternate_func_h;
    volatile word_t bit_reset;
    volatile word_t analog_switch_ctrl;
} gpio_t;

#define gpio(base)                  ((gpio_t*)base)                 /* GPIO base pointer macro */

#define gpio_a                      (gpio(GPIOA_BASE))              /* Predefined GPIO A macro*/
#define gpio_b                      (gpio(GPIOB_BASE))              /* Predefined GPIO B macro*/
#define gpio_c                      (gpio(GPIOC_BASE))              /* Predefined GPIO C macro*/
#define gpio_d                      (gpio(GPIOD_BASE))              /* Predefined GPIO D macro*/
#define gpio_e                      (gpio(GPIOE_BASE))              /* Predefined GPIO E macro*/
#define gpio_f                      (gpio(GPIOF_BASE))              /* Predefined GPIO F macro*/
#define gpio_g                      (gpio(GPIOG_BASE))              /* Predefined GPIO G macro*/
#define gpio_h                      (gpio(GPIOH_BASE))              /* Predefined GPIO H macro*/

/* GPIO moder */
#define gpio_moder_input_mode       0UL                             /* Set GPIO to input mode, pin value captured in input_data every bus clock */
#define gpio_moder_gp_out_mode(pin) (1UL << (pin * 2))              /* Set GPIO to output mode, write value to output_data, read input_data to determine pin state, read output_data for last written value */
#define gpio_moder_alt_fn_mode(pin) (2UL << (pin * 2))              /* Select alternate function via alternate_func11/2 mix/reg */
#define gpio_moder_analog_mode(pin) (3UL << (pin * 2))              /* Disable output buffer, input Schmitt trigger, pull resistors */
#define gpio_moder_mask(pin)        (3UL << (pin * 2))              /* Set GPIO moder mask */

#define GPIO_A_MODER_RESET          0xabffffff                      /* GPIO A reset value */
#define GPIO_B_MODER_RESET          0xfffffebf                      /* GPIO B reset value */
#define GPIO_C_G_MODER_RESET        0xffffffff                      /* GPIO C-G reset value */
#define GPIO_H_MODER_RESET          0x0000000f                      /* GPIO H reset value */

#define GPIO_INPUT                  0UL                             /* Set GPIO pin to input mode */
#define GPIO_OUTPUT                 1UL                             /* Set GPIO pin to output mode */
#define GPIO_ALT_FUNCTION           2UL                             /* Set GPIO pin to alternate function mode */
#define GPIO_ANALOG_IO              3UL                             /* Set GPIO pin to analog input output mode */

void set_gpio_pin_mode(gpio_t* io, int pin, int mode) {
    if(mode == GPIO_OUTPUT) {
        io->moder |= (1UL << (pin * 2));
        io->moder &= ~(2UL << (pin * 2));
    }
    else if(mode == GPIO_ALT_FUNCTION) {
        io->moder &= ~(1UL << (pin * 2));
        io->moder |= (2UL << (pin * 2));
    }
    else if(mode == GPIO_INPUT) {
        io->moder &= ~(1UL << (pin * 2));
        io->moder &= ~(2UL << (pin * 2));
    }
    else if(mode == GPIO_ANALOG_IO) {
        io->moder |= (1UL << (pin * 2));
        io->moder |= (2UL << (pin * 2));
    }
}

/* GPIO output type */
#define GPIO_OUTPUT_TYPE_PUSH_PULL           0UL                    /* N-MOS and P-MOD enabled */
#define GPIO_OUTPUT_TYPE_OPEN_DRAIN          1UL                    /* P-MOD disabled */
#define GPIO_OUTPUT_TYPE_RESET               0x0                    /* GPIO output type reset value */

void set_gpio_type(gpio_t* io, int pin, int open) {
    if(open) {
        io->output_type |= (1UL << pin);
    }
    else {
        io->output_type &= ~(1UL << pin);
    }
}

/* GPIO output speed */
#define GPIO_SPEED_LOW              0UL
#define GPIO_SPEED_MEDIUM           1UL
#define GPIO_SPEED_HIGH             2UL
#define GPIO_SPEED_VERY_HIGH        3UL

#define GPIO_A_SPEED_RESET          0x0c000000                      /* GPIO A speed reset value */
#define GPIO_SPEED_RESET            0x00000000                      /* GPIO !A speed reset value */

void set_gpio_speed(gpio_t* io, int pin, int speed) {
    if(speed == GPIO_SPEED_LOW) {
        io->output_speed &= ~(1UL << (pin * 2));
        io->output_speed &= ~(2UL << (pin * 2));
    }
    else if(speed == GPIO_SPEED_MEDIUM) {
        io->output_speed |= (1UL << (pin * 2));
        io->output_speed &= ~(2UL << (pin * 2));
    }
    else if(speed == GPIO_SPEED_HIGH) {
        io->output_speed &= ~(1UL << (pin * 2));
        io->output_speed |= (2UL << (pin * 2));
    }
    else if(speed == GPIO_SPEED_VERY_HIGH) {
        io->output_speed |= (1UL << (pin * 2));
        io->output_speed |= (2UL << (pin * 2));
    }
}

/* GPIO pull up/down */
#define GPIO_PUSH_PULL              0UL                             /* Disable pull down or pull up resistor */
#define GPIO_PULL_UP                1UL                             /* Enable pull up resistor (high state when no value) */
#define GPIO_PULL_DOWN              2UL                             /* Enable pull down resistor (low state when no value) */

#define GPIO_A_PUPD_RESET           0x64000000                      /* GPIO A pull up/pull down reset value */
#define GPIO_B_PUPD_RESET           0x00000100                      /* GPIO B pull up/pull down reset value */
#define GPIO_PUPD_RESET             0x00000000                      /* rest GPIO pull up/pull down reset value */

void set_gpio_pull_up_pull_down(gpio_t* io, int pin, int pupd) {
    if(pupd == GPIO_PUSH_PULL) {
        io->pull_up_down &= ~(1UL << (pin * 2));
        io->pull_up_down &= ~(2UL << (pin * 2));
    }
    else if(pupd == GPIO_PULL_UP) {
        io->pull_up_down |= (1UL << (pin * 2));
        io->pull_up_down &= ~(2UL << (pin * 2));
    }
    else if(pupd == GPIO_PULL_DOWN) {
        io->pull_up_down &= ~(1UL << (pin * 2));
        io->pull_up_down |= (2UL << (pin * 2));
    }
}

/* GPIO input data */
int get_gpio_input_value(gpio_t* io, int pin) {
    return io->input_data & (1UL << pin);
}

/* GPIO output data */
#define gpio_output_data(pin)       (1UL << pin)
#define GPIO_OUTPUT_DATA_RESET      0x00000000                      /* GPIO output data reset value */

void set_gpio_output_value(gpio_t* io, int pin, int state) {
    if(state == LOW) {
        io->output_data &= ~gpio_output_data(pin);
    }
    else {
        io->output_data |= gpio_output_data(pin);
    }
}

/* GPIO bit set/reset */
#define gpio_bit_set_reset_set(bit) (1UL << bit)                    /* GPIO Set desired bit */
#define gpio_bit_set_reset_reset_shift(bit) ((1UL << bit) << 16UL)  /* GPIO Set desired reset */

#define GPIO_BIT_RESET              0UL
#define GPIO_BIT_SET                1UL

void set_gpio_bit_set_reset(gpio_t* io, int pin, int state) {
    if(state == GPIO_BIT_RESET) {
        io->bit_set_reset |= ((1UL << pin) << 16UL);
        io->bit_set_reset &= ~(1UL << pin);
    }
    else if(state == GPIO_BIT_SET) {
        io->bit_set_reset |= (1UL << pin);
        io->bit_set_reset &= ~((1UL << pin) << 16UL);
    }
}

/* GPIO configuration lock */
#define gpio_lock_config(pin)       (1UL << pin)

#define GPIO_LOCK_RESET             0x0

void gpio_lock_config_port(gpio_t* io, int pin, int locked) {
    if(locked) {
        io->config_lock |= (1UL << pin);
    }
    else {
        io->config_lock &= ~(1UL << pin);
    }
}

/* GPIO alternate function low/high */
#define gpio_alt_func_sel1(pin)     (1UL << (pin * 4))
#define gpio_alt_func_sel2(pin)     (2UL << (pin * 4))
#define gpio_alt_func_sel3(pin)     (4UL << (pin * 4))
#define gpio_alt_func_sel4(pin)     (8UL << (pin * 4))
#define gpio_alt_func_mask(pin)     (0xFUL << (pin * 4))

#define GPIO_ALT_FUNC_RESET         0x0

void set_gpio_alternate_function(gpio_t* io, int pin, int alt_func) {
    if(pin < 8) {
        io->alternate_func_l |= alt_func;
    }
    else {
        io->alternate_func_h |= alt_func;
    }
}

/* GPIO bit reset */
#define gpio_bit_reset(pin)         (1UL << pin)

void set_gpio_bit_reset(gpio_t* io, int pin, int state) {
    if(state) {
        io->bit_reset |= (1UL << pin);
    }
    else {
        io->bit_reset &= ~(1UL << pin);
    }
}

/* GPIO analog switch control */
#define gpio_analog_sw_ctrl(pin)    (1UL << pin)

/**
 * 
 * Random Number Generator
 * 
 */

typedef struct {
    volatile word_t cr;
    volatile word_t sr;
    volatile word_t dr;
} rng_t;

#define rng                         ((rng_t*)RNG_BASE)

/**
 * 
 * Reset and Clock Control
 * 
*/

typedef struct {
    volatile word_t clock_ctrl;
    volatile word_t internal_clock_sources_calib;
    volatile word_t clock_config;
    volatile word_t sys_pll_config;
    volatile word_t pll_sai1_config;
    volatile word_t pll_sai2_config;
    volatile word_t clk_interrupt_enable;
    volatile word_t clk_interrupt_flag;
    volatile word_t clk_interrupt_clear;
    word_t reserved_0;
    volatile word_t ahb1_periph_reset;
    volatile word_t ahb2_periph_reset;
    volatile word_t ahb3_periph_reset;
    word_t reserved_1;
    volatile word_t apb1_periph_reset_reg1;
    volatile word_t apb1_periph_reset_reg2;
    volatile word_t apb2_periph_reset;
    word_t reserved_2;
    volatile word_t ahb1_periph_clk_enable;
    volatile word_t ahb2_periph_clk_enable;
    volatile word_t ahb3_periph_clk_enable;
    word_t reserved_3;
    volatile word_t apb1_periph_clk_enable_reg1;
    volatile word_t apb1_periph_clk_enable_reg2;
    volatile word_t apb2_periph_clk_enable;
    word_t reserved_4;
    volatile word_t ahb1_periph_clk_enable_sleep_stop;
    volatile word_t ahb2_periph_clk_enable_sleep_stop;
    volatile word_t ahb3_periph_clk_enable_sleep_stop;
    word_t reserved_5;
    volatile word_t apb1_periph_clk_enable_sleep_stop_reg1;
    volatile word_t apb1_periph_clk_enable_sleep_stop_reg2;
    volatile word_t apb2_periph_clk_enable_sleep_stop;
    word_t reserved_6;
    volatile word_t periph_independent_clk_config;
    word_t reserved_7;
    volatile word_t backup_domain_ctrl;
    volatile word_t clock_ctrl_status;
} rcc_t;

#define rcc                         ((rcc_t*)RCC_BASE)

/* Clock control */
#define RCC_CLOCK_CTRL_RESET        0x00000063
#define RCC_CLK_PLL_SAI2_READY      (1UL << 29)
#define RCC_CLK_PLL_SAI2_ENABLE     (1UL << 28)
#define RCC_CLK_PLL_SAI1_READY      (1UL << 27)
#define RCC_CLK_PLL_SAI1_ENABLE     (1UL << 26)
#define RCC_CLK_PLL_READY           (1UL << 25)
#define RCC_CLK_PLL_ENABLE          (1UL << 24)
#define RCC_CLK_SEC_SYS_ENABLE      (1UL << 19)
#define RCC_HSE_BYPASS              (1UL << 18)
#define RCC_CLK_HSE_READY           (1UL << 17)
#define RCC_CLK_HSE_ENABLE          (1UL << 16)
#define RCC_HSI_AUTO_START          (1UL << 11)
#define RCC_CLK_HSI_READY           (1UL << 10)
#define RCC_HSI_ALWAYS_ENABLE       (1UL << 9)
#define RCC_CLK_HSI_ENABLE          (1UL << 8)

#define MSI_RANGE_100kHz            0
#define MSI_RANGE_200kHz            1
#define MSI_RANGE_400kHz            2
#define MSI_RANGE_800kHz            3
#define MSI_RANGE_1MHz              4
#define MSI_RANGE_2MHz              5
#define MSI_RANGE_4MHz              6
#define MSI_RANGE_8MHz              7
#define MSI_RANGE_16MHz             8
#define MSI_RANGE_24MHz             9
#define MSI_RANGE_32MHz             10
#define MSI_RANGE_48MHz             11

#define RCC_CLK_MSI_RANGE(rn)       (rn << 4)
#define RCC_CLK_MSI_RANGE_SEL       8UL
#define RCC_CLK_MSI_PLL_ENABLE      4UL
#define RCC_CLK_MSI_READY           2UL
#define RCC_CLK_MSI_ENABLE          1UL

void set_rcc_clock_control(word_t rcc_clk_cfg, int state) {
    if(state) {
        rcc->clock_ctrl |= rcc_clk_cfg;
    }
    else {
        rcc->clock_ctrl &= ~rcc_clk_cfg;
    }
}

/* Internal clock sources calibration */
#define RCC_INTER_CLK_SRC_RESET     0x40000000
#define RCC_CLK_HSI_TRIM(t)         (t << 24)
#define RCC_CLK_HSI_CAL(c)          (c << 16)
#define RCC_CLK_MSI_TRIM(t)         (t << 8)
#define RCC_CLK_MSI_CAL(c)          c

void set_rcc_internal_clk_src_calib(word_t rcc_calib, int state) {
    if(state) {
        rcc->internal_clock_sources_calib |= rcc_calib;
    }
    else {
        rcc->internal_clock_sources_calib &= ~rcc_calib;
    }
}

/* Clock configuration */
#define RCC_CLK_CONFIG_RESET        0x0

/* Microcontroller clock output prescaler */
#define MCO_DIV_1                   0UL
#define MCO_DIV_2                   1UL
#define MCO_DIV_4                   2UL
#define MCO_DIV_8                   3UL
#define MCO_DIV_16                  4UL

#define RCC_CLK_MC_OUT_PRE(mco)     (mco << 28)

/* Microcontroller clock output */
#define MC_OUT_DISABLED             0UL
#define MC_OUT_SYS_CLK              1UL
#define MC_OUT_MSI_CLK              2UL
#define MC_OUT_HSI_CLK              3UL
#define MC_OUT_HSE_CLK              4UL
#define MC_OUT_PLL_CLK              5UL
#define MC_OUT_LSI_CLK              6UL
#define MC_OUT_LSE_CLK              7UL
#define MC_OUT_INTERN_HSI_CLK       8UL

#define RCC_CLK_MC_OUT_SEL(mco)     (mco << 24)

#define RCC_WAKEUP_STOP_CLK         (1UL << 15)

#define HCLK_DIV_1                  0
#define HCLK_DIV_2                  4
#define HCLK_DIV_4                  5
#define HCLK_DIV_8                  6
#define HCLK_DIV_16                 7

#define RCC_APB2_HI_PRESCALER(hclk) (hclk << 11)
#define RCC_APB1_LO_PRESCALER(hclk) (hclk << 8)

#define SYS_CLK_DIV_1               0x0
#define SYS_CLK_DIV_2               0x8
#define SYS_CLK_DIV_4               0x9
#define SYS_CLK_DIV_8               0xA
#define SYS_CLK_DIV_16              0xB
#define SYS_CLK_DIV_64              0xC
#define SYS_CLK_DIV_128             0xD
#define SYS_CLK_DIV_256             0xE
#define SYS_CLK_DIV_512             0xF

#define RCC_AHB_PRESCALER(sysclk)   (sysclk << 4)

#define SYS_CLK_SW_MSI              0
#define SYS_CLK_SW_HSI              1
#define SYS_CLK_SW_HSE              2
#define SYS_CLK_SW_PLL              3

#define RCC_SYS_CLK_SW_STATUS(sws)  (sws << 2)
#define RCC_SYS_CLK_SWITH(sws)      sws

// TODO: PLL config register and everything beyond thst register

// TODO: Comments what exacly some stuff means
/* AHB 1 clock enable values */
#define RCC_AHB1_DMA1               0UL
#define RCC_AHB1_DMA2               1UL
#define RCC_AHB1_FLASH              8UL
#define RCC_AHB1_CRC                12UL
#define RCC_AHB1_TSC                16UL

/* AHB 2 clock enable values */
#define RCC_AHB2_GPIO_A             0UL                                /* GPIO A clock enable value */
#define RCC_AHB2_GPIO_B             1UL                                /* GPIO B clock enable value */
#define RCC_AHB2_GPIO_C             2UL                                /* GPIO C clock enable value */
#define RCC_AHB2_GPIO_D             3UL                                /* GPIO D clock enable value */
#define RCC_AHB2_GPIO_E             4UL                                /* GPIO E clock enable value */
#define RCC_AHB2_GPIO_F             5UL                                /* GPIO F clock enable value */
#define RCC_AHB2_GPIO_G             6UL                                /* GPIO G clock enable value */
#define RCC_AHB2_GPIO_H             7UL                                /* GPIO H clock enable value */
#define RCC_AHB2_OTGFS              12UL
#define RCC_AHB2_ADC                13UL
#define RCC_AHB2_RNG                18UL                               /* RNG clock enable value */

/* AHB 3 clock enable values */
#define RCC_AHB3_FMC                0UL
#define RCC_AHB3_QSPI               8UL

/* APB 1 register 1 clock enable values */
#define RCC_APB11_TIM2              0UL
#define RCC_APB11_TIM3              1UL
#define RCC_APB11_TIM4              2UL
#define RCC_APB11_TIM5              3UL
#define RCC_APB11_TIM6              4UL
#define RCC_APB11_TIM7              5UL
#define RCC_APB11_LCD               9UL
#define RCC_APB11_WWDG              11UL
#define RCC_APB11_SPI2              14UL
#define RCC_APB11_SPI3              15UL
#define RCC_APB11_USART2            17UL
#define RCC_APB11_USART3            18UL
#define RCC_APB11_UART4             19UL
#define RCC_APB11_UART5             20UL
#define RCC_APB11_I2C1              21UL
#define RCC_APB11_I2C2              22UL
#define RCC_APB11_I2C3              23UL
#define RCC_APB11_CAN1              25UL
#define RCC_APB11_PWR               28UL
#define RCC_APB11_DAC1              29UL
#define RCC_APB11_OPAMP             30UL
#define RCC_APB11_LPTIM1            31UL

/* APB 1 register 2 clock enable values */
#define RCC_APB12_LPUART1           0UL
#define RCC_APB12_SWPMI1            2UL
#define RCC_APB12_LPTIM2            5UL

/* APB 2 clock enable */
#define RCC_APB2_SYS_CFG            0UL
#define RCC_APB2_FW                 7UL
// TODO: End

#define rcc_enable_clk(io)          (1 << io)                           /* Set enabled clocks */

/**
 * 
 * Real-Time Clock
 * 
*/

#define RTC_BACKUP_NUMBER           32UL

typedef struct {
    volatile word_t time;
    volatile word_t date;
    volatile word_t ctrl;
    volatile word_t init_status;
    volatile word_t prescaler;
    volatile word_t wakeup_timer;
    word_t reserverd_0;
    volatile word_t alarm_a;
    volatile word_t alarm_b;
    volatile word_t write_protection;
    volatile word_t sub_sec;
    volatile word_t shift_ctrl;
    volatile word_t time_stamp_time;
    volatile word_t time_stamp_date;
    volatile word_t time_stamp_sub_sec;
    volatile word_t calib;
    volatile word_t tamper_config;
    volatile word_t alarm_a_sub_sec;
    volatile word_t alarm_b_sub_sec;
    volatile word_t option;
    volatile word_t backup[RTC_BACKUP_NUMBER];

} rtc_t;

#define rtc                         ((rtc_t*)RTC_BASE)

#endif