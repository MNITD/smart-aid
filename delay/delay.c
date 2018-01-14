#include "delay.h"


/*******************************************************
* TIM6 setup; defining the delays *
*******************************************************/
void InitDelayTIM6(void)
{
	prescaler_ms =  SystemCoreClock / 1000 -1; // Prescaler for ms
	prescaler_us =  SystemCoreClock / 1000000 -1;//Prescaler for us
	RCC->APB1ENR |=  RCC_APB1ENR_TIM6EN;// TIM6 clock enable
}


/******************************
*   delay in miliseconds.   *
*   value = 1 ... 65536       *
******************************/
void TIM6delay_ms(uint16_t value)
{
	TIM6->PSC = prescaler_ms;//Set of prescaler for ms
	TIM6->ARR = value-1;// Set a value after which the timer will stop
	TIM6->CNT = 0;//Set zero to count register
	TIM6->CR1 |= TIM_CR1_CEN;//enable timer, start it.
	while((TIM6->SR & TIM_SR_UIF)==0){asm("NOP");} //Wait until the flag UIF is set => counting is over
	TIM6->SR &=~ TIM_SR_UIF;//Reset the flag.
}


/******************************
*	 delay in microseconds.  *
*   value = 1 ... 65536       *
******************************/
void TIM6delay_us(uint16_t value)
{
	TIM6->PSC = prescaler_us;// Set of prescaler for micro seconds
	TIM6->ARR = value-1;// Set a value after which the timer will stop
	TIM6->CNT = 0;// Set zero to count register
	TIM6->CR1 |= TIM_CR1_CEN;// enable timer, start it.
	while((TIM6->SR & TIM_SR_UIF)==0){asm("NOP");} // Wait until the flag UIF is set => counting is //over
	TIM6->SR &=~ TIM_SR_UIF;// Reset the flag.
}









void delay_init(void)
{  	RCC->APB2ENR |= RCC_APB2ENR_TIM17EN; /* Подаём тактирование (точнее включаем) таймер 17 */
	//TIM17->PSC = 4800-1;               /* выставляем предделитель счётчика так чтоб частота на входе счётчика равнялась 1МГц */

}

void delay(uint16_t a)//функция задержки в микросекундах
{
	//TIM17->EGR=TIM_EGR_UG;//генерируем событее обновления
	TIM17->PSC = SystemCoreClock / 1000000 -1;
	TIM17->ARR=a-1; //до скольки таймер считает
    TIM17->CNT=0; //обнуляем текущее значение счётчика

    TIM17->CR1 |= TIM_CR1_CEN|TIM_CR1_OPM; //включаем счетчик
    while((TIM17->CR1 & TIM_CR1_CEN)!=0){asm("NOP");} //проверка переполнения счётчика
}


void delay_ms(uint16_t a)//функция задержки в милісекундах
{
	//TIM17->EGR=TIM_EGR_UG;//генерируем событее обновления
	TIM17->PSC = SystemCoreClock / 1000 -1;
	TIM17->ARR=a-1; //до скольки таймер считает
    TIM17->CNT=0; //обнуляем текущее значение счётчика

    TIM17->CR1 |= TIM_CR1_CEN|TIM_CR1_OPM; //включаем счетчик
    while((TIM17->CR1 & TIM_CR1_CEN)!=0){asm("NOP");} //проверка переполнения счётчика
}

