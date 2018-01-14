#include "i2c_lcd.h"

//#include "stm32f0xx_i2c.h"

uint8_t backlightState = 1;


#define I2C1_OWN_ADDRESS (0x27) // extra important ! Real address of LCD device


void gpioInit(void) {
	GPIO_InitTypeDef gpio_i2c;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

	/* Select output mode (01) on PC8 and PC9 */
	gpio_i2c.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	gpio_i2c.GPIO_Mode = GPIO_Mode_AF;
	gpio_i2c.GPIO_Speed = GPIO_Speed_50MHz;
	gpio_i2c.GPIO_PuPd = GPIO_PuPd_UP;
	gpio_i2c.GPIO_OType = GPIO_OType_OD;

	GPIO_Init(GPIOB, &gpio_i2c);



//	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_I2C1);
//	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_I2C1);

	GPIOB->AFR[0] = (GPIOB ->AFR[0] &~ (GPIO_AFRL_AFRL6 | GPIO_AFRL_AFRL7))\
		                  | (1 << (6 * 4)) | (1 << (7 * 4));


}

void i2cInit(void) {
//	I2C_InitTypeDef i2c;

//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

	  /* Configure I2C2, master */
	  /* (1) Timing register value is computed with the AN4235 xls file,
	   fast Mode @400kHz with I2CCLK = 48MHz, rise time = 140ns, fall time = 40ns */
	  /* (2) Periph enable */
	  /* (3) Slave address = 0x5A, write transfer, 1 byte to transmit, autoend */
	I2C1->CR1 &= ~(I2C_CR1_PE) /*| I2C_CR1_GCEN | I2C_CR1_WUPEN*/; /* (2) */

	I2C1->TIMINGR = (uint32_t)0x00E0D3FF /*0x205078C1 *//*0x108065BE*//*0x60201B6B*//* 0x40422631*/; /* (1) */

	  I2C1->CR1 |= I2C_CR1_PE /*| I2C_CR1_GCEN | I2C_CR1_WUPEN*/; /* (2) */

	  I2C1->CR2 |=  I2C_CR2_AUTOEND | (1<<16) | (I2C1_OWN_ADDRESS<<1); /* (3) */

//	i2c.I2C_ClockSpeed = 50000;
//	i2c.I2C_Mode = I2C_Mode_I2C;
//	i2c.I2C_DutyCycle = I2C_DutyCycle_2;
//	i2c.I2C_OwnAddress1 = 0x27;
//	i2c.I2C_Ack = I2C_Ack_Enable;
//	i2c.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
//	I2C_Init(I2C1, &i2c);

	I2C_Cmd(I2C1, ENABLE);
}

// LCD  ==================================





void lcd_Goto(uint8_t row, uint8_t col) {
//#ifdef LCD_2004
//	switch (row){
//		case 1:
//			lcd_Command(0x80 + col);
//			break;
//		case 2:
//			lcd_Command(0x80 + col + 0x40);
//			break;
//		case 3:
//			lcd_Command(0x80 + col + 0x14);
//			break;
//		case 4:
//			lcd_Command(0x80 + col + 0x54);
//			break;
//	}
//#endif
}

void lcd_PrintC(const uint8_t *str) {
 	uint8_t i=1;

 	while (i = *str){
 	//while (*str != '\0'){
 		//TIM6delay_ms(100);
 		lcd_Data(i);
 		TIM6delay_ms(5);
 		str++;
 		//str++;
 		//i++;

 		//lcd_Send(i);

 	}
	//lcd_Data('M');
}

void lcd_Init(void) {

	lcd_Command(0x00);
	TIM6delay_ms(500);
//	lcd_Command(0x33);
//	TIM6delay_ms(7);
//	lcd_Command(0x32);
//	delay(100);
	lcd_FirstCommand();
	/*	TIM6delay_ms(500);

	lcd_Command(0x28);// 4bit 2 lines, 5x8 font. Page 8 Function Set. Page 9 Top.

	lcd_Command(0x08);// display off entirely. Page 6, row 4. Page 8 Display On/Off


	lcd_Command(0x06);//Page 6 Row 3
	lcd_Command(0x01);
		TIM6delay_ms(20); // Clear display. Page 6, first row. Page 7 Clear Display.

	lcd_Command(0x02); //TIM6delay_ms(20);//Return home
	TIM6delay_ms(20);

	lcd_Command(0x80);
	lcd_Command(0x0E);
	TIM6delay_ms(1000);*/

	lcd_Command(0x38);        // enable 5x7 mode for chars



	lcd_Command(0x08);// display off entirely. Page 6, row 4. Page 8 Display On/Off
	lcd_Command(0x02);        // Clear Display

	lcd_Command(0x08);// display off entirely. Page 6, row 4. Page 8 Display On/Off
	lcd_Command(0x01);        // Clear Display
//	TIM6delay_ms(5);
	//lcd_Command(0x02);        // Clear Display

	TIM6delay_ms(7);
	lcd_Command(0x06);//Page 6 Row 3
	lcd_Command(0x0C);        // Display OFF, Cursor ON




	/*lcd_Command(0x80);        // Move the cursor to beginning of first line
		lcd_Command(0x02);        // Clear Display*/





	/*lcd_Backlight(0);
	GPIOC->ODR = 0xFF;
	delay_ms(1000);
	lcd_Backlight(1);
	GPIOC->ODR = 0x00;
	delay_ms(1000);
	lcd_Backlight(0);
	GPIOC->ODR = 0xFF;
	delay_ms(1000);
	lcd_Backlight(1);
	GPIOC->ODR = 0x00;*/


}

void lcd_Send(uint8_t data) {
/*
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));
	I2C_GenerateSTART(I2C1, ENABLE);

//	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
	//I2C_Send7bitAddress(I2C1, ((0x20+LCD_ADDR) << 1), I2C_Direction_Transmitter);
	//while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

	I2C_SendData(I2C1, 0x27);
	while (!(I2C1->ISR & I2C_ISR_TXE));
	I2C_SendData(I2C1, data);
	while (!(I2C1->ISR & I2C_ISR_TXE));
//	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	I2C_GenerateSTOP(I2C1, ENABLE);
*/

//	while((I2C2->ISR & I2C_ISR_BUSY) == (I2C_ISR_BUSY));
//	//I2C2->CR2 |= I2C_CR2_START;
//
//
//
//	// start I2C master transmission sequence
//	if((I2C2->ISR & I2C_ISR_TXE) == (I2C_ISR_TXE)) // Check Tx empty
//	{
//	I2C2->TXDR = data; // Byte to send
//	  I2C2->CR2 |= I2C_CR2_START; // Go

/*
	I2C1->CR1|=I2C_CR1_PE; // включаю тактирование переферии ≤2—
	I2C1->CR1 |= I2C_CR1_ START; // генерирую старт

	//if( !(operation_chek_set_bit( &I2C1->ISR, I2C_ISR_ SB))){return (0);}; // жду установлени€ бита I2C_SR1_SB или окончани€ тайм аута
	//while(!(I2C1->SR1 & I2C_SR1_SB)){}; // жду окончани€ генерации старт

	(void) I2C1->ISR; // сбрасываю бит SB
	I2C1->TXDR=0x27; // передаю адрес ведомого

	if( !(( I2C1->ISR & I2C_ISR_ADDR) == I2C_ISR_ADDR)){return (0);};
	//while(!(I2C1->SR1 & I2C_SR1_ADDR)){}; // жду окончани€ передачи адреса ведомого

	//(void) I2C1->ISR;
	(void) I2C1->ISR; // очищаю бит и I2C_SR1_ADDR I2C_SR1_TxE
	I2C1->TXDR=0xFF;

	if( !(( I2C1->ISR & I2C_ISR_TXE) == I2C_ISR_TXE)){return (0);};
	//while (!(I2C1->SR1 & I2C_SR1_TXE)){};

	I2C1->TXDR=data;

	if( !(( I2C1->ISR & I2C_ISR_BUSY) == I2C_ISR_BUSY)){return (0);};
	//while (!(I2C1->SR1 & I2C_SR1_BTF)){}; // последний бит

	I2C1->CR1 |= I2C_CR1_STOPIE; // формирование сигнала "—топ"

*/
	while((I2C2->ISR & I2C_ISR_BUSY) == (I2C_ISR_BUSY));
	while(!((I2C1->ISR & I2C_ISR_TXE) == (I2C_ISR_TXE))); // Check Tx empty
		{
		  I2C1->TXDR = data; // Byte to send
		  I2C1->CR2 |= I2C_CR2_START; // Go
	}
		//while(!((I2C1->ISR & I2C_ISR_TC) == (I2C_ISR_TC)));

		while(!((I2C1->ISR & I2C_ISR_TXE) == (I2C_ISR_TXE)));
		while((I2C2->ISR & I2C_ISR_BUSY) == (I2C_ISR_BUSY));
		//I2C1->CR2 |= I2C_CR2_STOP; // формирование сигнала "—топ"

	}


void lcd_FirstCommand(/*uint8_t com*/ void) {
	/*uint8_t data = 0;

	data |= (backlightState & 0x01) << BL;

	data |= (((com & 0x10) >> 4) << DB4);
	data |= (((com & 0x20) >> 5) << DB5);
	data |= (((com & 0x40) >> 6) << DB6);
	data |= (((com & 0x80) >> 7) << DB7);*/
	//lcd_Send(data);
	//lcd_checkBusy();
	//data |= (1 << EN);
	////////////lcd_Send(0x03 |(backlightState & 0x01) << BL);
	lcd_Send(0x03 | (1 << EN));
	TIM6delay_us(100);
	lcd_Send(0x03|((backlightState & 0x01) << BL));

	//TIM6delay_us(4500);
	TIM6delay_ms(5);

	//data &= ~(1 << EN);
	////////////lcd_Send(0x03|(backlightState & 0x01) << BL);
	lcd_Send(0x03 | (1 << EN));
	TIM6delay_us(100);
	lcd_Send(0x03|((backlightState & 0x01) << BL));



	TIM6delay_us(100);

	////////////lcd_Send(0x03|((backlightState & 0x01) << BL));
	lcd_Send(0x03 | (1 << EN));
	TIM6delay_us(100);
	lcd_Send(0x03|((backlightState & 0x01) << BL));

	TIM6delay_us(100);

	////////////lcd_Send(0x02|((backlightState & 0x01) << BL));
	lcd_Send(0x02 | (1 << EN));
	TIM6delay_us(100);
	lcd_Send(0x02|((backlightState & 0x01) << BL));
	TIM6delay_us(100);



	/*data = 0;

	data |= (backlightState & 0x01) << BL;

	data |= (((com & 0x01) >> 0) << DB4);
	data |= (((com & 0x02) >> 1) << DB5);
	data |= (((com & 0x04) >> 2) << DB6);
	data |= (((com & 0x08) >> 3) << DB7);
	lcd_Send(data);

	data |= (1 << EN);
	lcd_Send(data);
	lcd_pause_short;

	data &= ~(1 << EN);
	lcd_Send(data);
	//lcd_pause_short;*/
}

void lcd_Command(uint8_t com) {
	uint8_t data = 0;
	//lcd_checkBusy();
	data |= (backlightState & 0x01) << BL;

	data |= (((com & 0x10) >> 4) << DB4);
	data |= (((com & 0x20) >> 5) << DB5);
	data |= (((com & 0x40) >> 6) << DB6);
	data |= (((com & 0x80) >> 7) << DB7);
	//lcd_Send(data);

	//data |= (1 << EN);

	//lcd_Send(data);
	lcd_Send(0x00);
	lcd_Send(data | (1 << EN));
	//TIM6delay_us(100);
	lcd_Send(data);
	//lcd_pause_short;



	//lcd_pause;
		data = 0;

	data |= (backlightState & 0x01) << BL;

	data |= (((com & 0x01) >> 0) << DB4);
	data |= (((com & 0x02) >> 1) << DB5);
	data |= (((com & 0x04) >> 2) << DB6);
	data |= (((com & 0x08) >> 3) << DB7);
	//lcd_Send(data);

	//lcd_Send(data);
	lcd_Send(0x00);
	lcd_Send(data | (1 << EN));
	//TIM6delay_us(100);
	lcd_Send(data);


	//data &= ~(1 << EN);
	//lcd_Send(data);
	//lcd_pause_short;
}

void lcd_Backlight(uint8_t state) {
	backlightState = (state & 0x01) << BL;
	lcd_Send(backlightState);

}

void lcd_Data(uint8_t com) {
	uint8_t data = 0;
	//lcd_checkBusy();
	//data |= (1 << EN);
	data |= (1 << RS);
	//data |= (1 << RW);
	data |= (backlightState & 0x01) << BL;


	data |= (((com & 0x10) >> 4) << DB4);
		data |= (((com & 0x20) >> 5) << DB5);
		data |= (((com & 0x40) >> 6) << DB6);
		data |= (((com & 0x80) >> 7) << DB7);





	lcd_Send(0x00 | (backlightState & 0x01) << BL);
	//TIM6delay_us(100);
	//lcd_Send(data);
	lcd_Send(data | (1 << EN));
	//TIM6delay_us(100);
	lcd_Send(data);
	//TIM6delay_us(100);
	//lcd_Send(0x00 | (backlightState & 0x01) << BL);
	//TIM6delay_us(100);

	//TIM6delay_ms(2);
	//lcd_pause_short;


	data = 0;

//	data |= (1 << EN);
	data |= (1 << RS);
	//data |= (1 << RW);
	data |= (backlightState & 0x01) << BL;

	data |= (((com & 0x01) >> 0) << DB4);
		data |= (((com & 0x02) >> 1) << DB5);
		data |= (((com & 0x04) >> 2) << DB6);
		data |= (((com & 0x08) >> 3) << DB7);

	lcd_Send(0x00 | (backlightState & 0x01) << BL);
//	TIM6delay_us(100);
	//lcd_Send(data);
	lcd_Send(data | (1 << EN));
	//TIM6delay_us(100);
	lcd_Send(data);
	//lcd_Send(data);
	//lcd_Send(0x00 | (backlightState & 0x01) << BL);
	TIM6delay_us(100);

	}

void lcd_checkBusy(void)
{
	uint8_t flag = 1;
	uint8_t temp;
	while((I2C2->ISR & I2C_ISR_BUSY) == (I2C_ISR_BUSY));
	while(!((I2C1->ISR & I2C_ISR_TXE) == (I2C_ISR_TXE))); // Check Tx empty

	/*lcd_Send(0x00|(1 << RS)|(1 << RW)|(backlightState & 0x01) << BL);
	lcd_Send(0x00|(1 << RS)|(1 << RW) | (1 << EN)|(backlightState & 0x01) << BL);
	TIM6delay_us(2);
	lcd_Send(0x00|(1 << RS)|(1 << RW)|(backlightState & 0x01) << BL);
*/
	/*lcd_Send(0x00|(1 << RS)|(1 << RW)|(backlightState & 0x01) << BL);
	lcd_Send(0x00 |(1 << RS)|(1 << RW)| (1 << EN) |(backlightState & 0x01) << BL);
	TIM6delay_us(2);
	lcd_Send(0x00|(1 << RS)|(1 << RW)|(backlightState & 0x01) << BL);*/


	I2C1->CR2 |=  I2C_CR2_RD_WRN;
	while(flag==1)
	{  /* start I2C slave transmission sequence */
	  I2C2->CR2 |= I2C_CR2_START;

	  while(((I2C2->ISR & I2C_ISR_RXNE) == I2C_ISR_RXNE));

	  temp = I2C2->RXDR;
	  if((temp&0x08) != 0x08) flag++;
	  I2C2->CR2 |= I2C_CR2_START;

	  while(((I2C2->ISR & I2C_ISR_RXNE) == I2C_ISR_RXNE));
	  temp = I2C2->RXDR;
	}

	I2C1->CR2 &=  ~I2C_CR2_RD_WRN;

}

int LCD_launch(void ) {
		gpioInit();
				i2cInit();
				InitDelayTIM6();
				delay_init();
				TIM6delay_ms(100);
				lcd_Init();
	}
void LCD_test_run(void){
	// LCD ====================================================


	const uint8_t mes [] = "TWA";
	const uint8_t mes1 [] = "AAA";

	uint8_t str [20];

			lcd_Command(0xC0);
			sprintf(str, mes1);
			lcd_PrintC(str);


			lcd_Command(0x80);
			sprintf(str, mes);
			lcd_PrintC(str);

			TIM6delay_ms(1000);
			lcd_Command(0x01);
			lcd_Command(0x80);

			sprintf(str, "ABO");
			lcd_PrintC(str);
			sprintf(str, "ABO");
			lcd_PrintC(str);

			TIM6delay_ms(1000);
			lcd_Command(0x01);
			lcd_Command(0xC0);

			/*lcd_PrintC(mes);
			lcd_PrintC(mes);*/

	//		lcd_Goto(1, 0);

			sprintf(str, "Value of Pi");
			lcd_PrintC(str);
		// LCD ===================================================
}
