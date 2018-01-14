#include <stm32f0xx.h>

void delay_init(void);
void delay(uint16_t);
void delay_ms ( uint16_t nTime );

uint16_t prescaler_ms; // Value of prescaler to set delay in //ms.(MCU_frequency/ 1000 = 48 000)
uint16_t prescaler_us;// Value of prescaler to set delay in micro seconds.

void InitDelayTIM6(void);
void TIM6delay_ms(uint16_t value);
void TIM6delay_us(uint16_t value);
