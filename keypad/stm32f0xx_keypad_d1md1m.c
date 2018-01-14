#include "stm32f0xx_keypad_d1md1m.h"



void keypad_init_GPIO_IN(GPIO_TypeDef* PORT, uint16_t Pin ){
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = Pin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(PORT, &GPIO_InitStructure);
};

void keypad_init_GPIO_OUT(GPIO_TypeDef* PORT, uint16_t Pin){
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = Pin;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_Init(PORT, &GPIO_InitStructure);
};



void initgpio_keyboard()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_GPIOC, ENABLE);
	keypad_init_GPIO_OUT(KEYPAD_ROW_4_PORT, KEYPAD_ROW_4_PIN);
	keypad_init_GPIO_OUT(KEYPAD_ROW_3_PORT, KEYPAD_ROW_3_PIN);
	keypad_init_GPIO_OUT(KEYPAD_ROW_2_PORT, KEYPAD_ROW_2_PIN);
	keypad_init_GPIO_OUT(KEYPAD_ROW_1_PORT, KEYPAD_ROW_1_PIN);


	keypad_init_GPIO_IN(KEYPAD_COLUMN_4_PORT, KEYPAD_COLUMN_4_PIN);
	keypad_init_GPIO_IN(KEYPAD_COLUMN_3_PORT, KEYPAD_COLUMN_3_PIN);
	keypad_init_GPIO_IN(KEYPAD_COLUMN_2_PORT, KEYPAD_COLUMN_2_PIN);
	keypad_init_GPIO_IN(KEYPAD_COLUMN_1_PORT, KEYPAD_COLUMN_1_PIN);

}

void keypadRowHight(GPIO_TypeDef* PORT, uint16_t PIN){
	PORT->BSRR = PIN;
}
void keypadRowLow(GPIO_TypeDef* PORT, uint16_t PIN){
	PORT->BRR = PIN;
}


int read_active_colomn(void){
	if(GPIO_ReadInputDataBit(KEYPAD_COLUMN_1_PORT, KEYPAD_COLUMN_1_PIN))//read input bit PB12
		return 1;
	if(GPIO_ReadInputDataBit(KEYPAD_COLUMN_2_PORT, KEYPAD_COLUMN_2_PIN)) //read input bit PB11
		return 2;
	if(GPIO_ReadInputDataBit(KEYPAD_COLUMN_3_PORT, KEYPAD_COLUMN_3_PIN)) //read input bit PB10
		return 3;
	if(GPIO_ReadInputDataBit(KEYPAD_COLUMN_4_PORT, KEYPAD_COLUMN_4_PIN)) //read input bit PB2
		return 4;
	return 0;
}


void setColHight(int col){
	keypadRowLow(KEYPAD_ROW_1_PORT, KEYPAD_ROW_1_PIN);
	keypadRowLow(KEYPAD_ROW_2_PORT, KEYPAD_ROW_2_PIN);
	keypadRowLow(KEYPAD_ROW_3_PORT, KEYPAD_ROW_3_PIN);
	keypadRowLow(KEYPAD_ROW_4_PORT, KEYPAD_ROW_4_PIN);
	switch (col){
	case 1:
			keypadRowHight(KEYPAD_ROW_1_PORT, KEYPAD_ROW_1_PIN);
			break;
	case 2:
			keypadRowHight(KEYPAD_ROW_2_PORT, KEYPAD_ROW_2_PIN);
			break;

	case 3:
			keypadRowHight(KEYPAD_ROW_3_PORT, KEYPAD_ROW_3_PIN);
			break;

	case 4:
			keypadRowHight(KEYPAD_ROW_4_PORT, KEYPAD_ROW_4_PIN);
			break;
	}
}


char read_keypad_key(){
	char keypad[] = {'1', '2', '3', 'A', '4', '5', '6','B' ,'7','8','9', 'C' ,  '*' , '0' , '#', 'D'};

	int key_pressed = 0;
	{
		int i = 1;
		for (int keypad_col = 1; keypad_col <= 4; keypad_col++){
			setColHight(keypad_col);
						key_pressed = read_active_colomn();
						if (key_pressed)
							return keypad[4 * (keypad_col -1 ) + key_pressed - 1 ];
		}
	}
	return 0;
}

