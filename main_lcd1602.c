//#include "stm32f0xx.h"
//
//
//
//#include "i2c_lcd.h"
//
//#define I2C1_OWN_ADDRESS (0x3F) // extra important ! Real address of LCD device
//
//
//
//
///*void gpioInit(void);
//void i2cInit(void);
//void delay(uint32_t t);
//*/
//
////const uint8_t mes[] = "STM32F4 + I2C + LCD";
//const uint8_t mes[] = "TMW";
//const uint8_t mes1[] = "AAA";
//
//uint8_t str [20];
//
//int main(void) {
//
//SystemInit();
//
//RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
//
//
//  GPIOC->MODER = (GPIOC->MODER & ~(GPIO_MODER_MODER6 | GPIO_MODER_MODER7)) \
//				 | (GPIO_MODER_MODER6_0 | GPIO_MODER_MODER7_0);
//
//
//	gpioInit();
//	i2cInit();
//	InitDelayTIM6();
//	delay_init();
//	TIM6delay_ms(100);
//	lcd_Init();
//	/*TIM6delay_ms(10);
//	lcd_Init();*/
//
//	lcd_Command(0xC0);
//	sprintf(str, mes1);
//	lcd_PrintC(str);
//
//
//	lcd_Command(0x80);
//	sprintf(str, mes);
//	lcd_PrintC(str);
//
//	TIM6delay_ms(1000);
//	lcd_Command(0x01);
//	lcd_Command(0x80);
//
//	sprintf(str, "ABO");
//	lcd_PrintC(str);
//	sprintf(str, "ABO");
//	lcd_PrintC(str);
//
//	TIM6delay_ms(1000);
//	lcd_Command(0x01);
//	lcd_Command(0xC0);
//
//	/*lcd_PrintC(mes);
//	lcd_PrintC(mes);*/
//
////	lcd_Goto(1, 0);
//
//	sprintf(str, "Value of Pi");
//	lcd_PrintC(str);
//	//lcd_PrintC('A');
//	//lcd_PrintC("ABO");
//	//TIM6delay_ms(1000);
//	//lcd_PrintC("Hello world");
//	//lcd_PrintC("CSUCU");
///*
//	lcd_Goto(1, 3);
//	lcd_PrintC("\"Hello world!\"");
//
//	lcd_Goto(1, 0);
//	lcd_PrintC("how.net.ua");
//
//
//*/
//    while(1) {
//    	//iii2c_send(0xAA);
//    	/*GPIOC->ODR ^= 1<<6;
//    	TIM6delay_us(65000);*/
//
//    }
///*    lcd_PrintC(mes);
//
//    delay_ms(100000);*/
//}
//
////void delay(uint32_t t) {
////	uint32_t i = 0;
////	for (; i < t; i++);
////}
//
//void gpioInit(void) {
//	//GPIO_InitTypeDef gpio;
//	//RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
//
//	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
//
//	/* Select output mode (01) on PC8 and PC9 */
//	/*gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
//		gpio.GPIO_Mode = GPIO_Mode_AF;*/
//	  GPIOB->MODER = (GPIOB->MODER & ~(GPIO_MODER_MODER6 | GPIO_MODER_MODER7)) \
//					 | (GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1);
//
//	 //gpio.GPIO_Speed = GPIO_Speed_50MHz;
//	  //GPIOB->OSPEEDR = 0xFFFF;
//
//	 // gpio.GPIO_PuPd = GPIO_PuPd_UP;
//	  //GPIOB->PUPDR |= GPIO_PUPDR_PUPDR6_0 | GPIO_PUPDR_PUPDR7_0;
//
//	  GPIOB->OTYPER |= GPIO_OTYPER_OT_6 | GPIO_OTYPER_OT_7;
//
//	/*
//	gpio.GPIO_OType = GPIO_OType_OD;
//	*/
//	//GPIO_Init(GPIOB, &gpio);
//
//
//
//	//GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_I2C1);
//	//GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_I2C1);
//
//	GPIOB->AFR[0] = (GPIOB ->AFR[0] &~ (GPIO_AFRL_AFRL6 | GPIO_AFRL_AFRL7))\
//		                  | (1 << (6 * 4)) | (1 << (7 * 4));
//
//
//}
//
//void i2cInit(void) {
//	//I2C_InitTypeDef i2c;
//
//	//RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
//	RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
//
//	  /* Configure I2C2, master */
//	  /* (1) Timing register value is computed with the AN4235 xls file,
//	   fast Mode @400kHz with I2CCLK = 48MHz, rise time = 140ns, fall time = 40ns */
//	  /* (2) Periph enable */
//	  /* (3) Slave address = 0x5A, write transfer, 1 byte to transmit, autoend */
//	I2C1->CR1 &= ~(I2C_CR1_PE) /*| I2C_CR1_GCEN | I2C_CR1_WUPEN*/; /* (2) */
//
//	I2C1->TIMINGR = (uint32_t)0x00E0D3FF /*0x205078C1 *//*0x108065BE*//*0x60201B6B*//* 0x40422631*/; /* (1) */
//
//	  I2C1->CR1 |= I2C_CR1_PE /*| I2C_CR1_GCEN | I2C_CR1_WUPEN*/; /* (2) */
//
//	  I2C1->CR2 |=  I2C_CR2_AUTOEND | (1<<16) | (I2C1_OWN_ADDRESS<<1); /* (3) */
//
//
//
//
//	/*i2c.I2C_ClockSpeed = 50000;
//	i2c.I2C_Mode = I2C_Mode_I2C;
//	i2c.I2C_DutyCycle = I2C_DutyCycle_2;
//	i2c.I2C_OwnAddress1 = 0x00;
//	i2c.I2C_Ack = I2C_Ack_Enable;
//	i2c.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
//	I2C_Init(I2C1, &i2c);
//
//	I2C_Cmd(I2C1, ENABLE);*/
//}
///*
//void iii2c_send(uint8_t data){
//	// start I2C master transmission sequence
//	while(!(I2C1->ISR & I2C_ISR_TXE) == (I2C_ISR_TXE)) // Check Tx empty
//	{
//	  I2C1->TXDR = data; // Byte to send
//	  I2C1->CR2 |= I2C_CR2_START; // Go
//}
//}*/
