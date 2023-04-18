void init(void);
void def_handler(void);
int main(void);

extern unsigned char INIT_DATA_VALUES;
extern unsigned char INIT_DATA_START;
extern unsigned char INIT_DATA_END;
extern unsigned char BSS_START;
extern unsigned char BSS_END;

const void* Vectors[] __attribute__((section(".vectors"))) = {
    (void*)0x20018000,
    init,
    def_handler,	/* Hard Fault */
	def_handler,  	/* MemManage */
	def_handler,      	/* BusFault  */
	def_handler,        /* UsageFault */
	def_handler,	/* Reserved */ 
	def_handler,	/* Reserved */
	def_handler,	/* Reserved */
	def_handler,	/* Reserved */
	def_handler,        /* SVCall */
	def_handler,	/* Debug */
	def_handler,	/* Reserved */
	def_handler,        /* PendSV */
	def_handler,      	/* SysTick */	
    /* External interrupt handlers follow */
	def_handler, 	/* 0 WWDG */
	def_handler, 	/* 1 PVD */
	def_handler, 	/* 2 TAMP_SAMP */
	def_handler, 	/* 3 RTC_WKUP */
	def_handler, 	/* 4 FLASH */
	def_handler, 	/* 5 RCC */
	def_handler, 	/* 6 EXTI0 */
	def_handler, 	/* 7 EXTI1 */
	def_handler, 	/* 8 EXTI2 and TSC */
	def_handler, 	/* 9 EXTI3 */
	def_handler, 	/* 10 EXTI4 */
	def_handler, 	/* 11 DMA_CH1 */
	def_handler, 	/* 12 DMA_CH2 */
	def_handler, 	/* 13 DMA_CH3 */
	def_handler, 	/* 14 DMA_CH4 */
	def_handler, 	/* 15 DMA_CH5 */
	def_handler, 	/* 16 DMA_CH6 */
	def_handler, 	/* 17 DMA_CH7 */
	def_handler, 	/* 18 ADC1_2 */
	def_handler, 	/* 19 CAN_TX */
	def_handler, 	/* 20 CAN_RX0 */
	def_handler, 	/* 21 CAN_RX1 */
	def_handler, 	/* 22 CAN_SCE */
	def_handler, 	/* 23 EXTI9_5 */
	def_handler, 	/* 24 TIM1_BRK/TIM15 */
	def_handler, 	/* 25 TIM1_UP/TIM16 */
	def_handler, 	/* 26 TIM1_TRG/TIM17 */
	def_handler, 	/* 27 TIM1_CC */
	def_handler, 	/* 28 TIM2 */
	def_handler, 	/* 29 TIM3 */
	def_handler, 	/* 30 TIM4 */
	def_handler, 	/* 31 I2C1_EV */
	def_handler, 	/* 32 I2C1_ER */
	def_handler, 	/* 33 I2C2_EV */
	def_handler, 	/* 34 I2C2_ER */
	def_handler, 	/* 35 SPI1 */
	def_handler, 	/* 36 SPI2 */
	def_handler, 	/* 37 USART1 */
	def_handler, 	/* 38 USART2 */
	def_handler, 	/* 39 USART3 */
	def_handler, 	/* 40 EXTI15_10 */
	def_handler, 	/* 41 RTCAlarm */
	def_handler, 	/* 42 DFSDM1_FLT3 */
	def_handler, 	/* 43 TIM8_BRK */
	def_handler, 	/* 44 TIM8_UP */
	def_handler, 	/* 45 TIM8_TRG_COM */
	def_handler, 	/* 46 TIM8_CC */
	def_handler, 	/* 47 ADC3 */
	def_handler, 	/* 48 FMC */
	def_handler, 	/* 49 SDMMC1 */
	def_handler, 	/* 50 TIM5 */
	def_handler, 	/* 51 SPI3 */
	def_handler, 	/* 52 UART4 */
	def_handler, 	/* 53 UART5 */
	def_handler, 	/* 54 TIM6_DACUNDER */
	def_handler, 	/* 55 TIM7 */
	def_handler, 	/* 56 DMA2_CH1 */
	def_handler, 	/* 57 DMA2_CH2 */
	def_handler, 	/* 58 DMA2_CH3 */
	def_handler, 	/* 59 DMA2_CH4 */
	def_handler, 	/* 60 DMA2_CH5 */
	def_handler, 	/* 61 DFSDM1_FLT0 */
	def_handler, 	/* 62 DFSDM1_FLT1 */
	def_handler, 	/* 63 DFSDM1_FLT2*/
	def_handler, 	/* 64 COMP */
	def_handler, 	/* 65 LPTIM1 */
	def_handler, 	/* 66 LPTIM2 */
	def_handler, 	/* 67 OTG_FS */
	def_handler, 	/* 68 DMA_CH6 */
	def_handler, 	/* 69 DMA_CH7 */
	def_handler, 	/* 70 LPUART1 */
	def_handler, 	/* 71 QUADSPI */
	def_handler, 	/* 72 I2C3_EV */
	def_handler, 	/* 73 I2C3_ER */
	def_handler, 	/* 74 SAI1 */
	def_handler, 	/* 75 SAI2 */
	def_handler, 	/* 76 SWPMI1 */
	def_handler, 	/* 77 TSC */
	def_handler, 	/* 78 LCD */
	def_handler, 	/* 79 AES */
	def_handler, 	/* 80 RNG */
	def_handler, 	/* 81 FPU */
};

void init() {
    unsigned char *src;
    unsigned char *dest;
    unsigned int len;

    src = &INIT_DATA_VALUES;
    dest = &INIT_DATA_START;
    len = &INIT_DATA_END - &INIT_DATA_START;

    while(len--) {
        *dest++ = 0;
    }

    main();
}

void def_handler() {
    while(1) {

    }
}