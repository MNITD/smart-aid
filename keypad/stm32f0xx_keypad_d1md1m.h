//#include "stm32f0xx.h"
#include "defines.h"
#include "stm32f0xx_gpio.h"
#include <stm32f0xx_rcc.h>

typedef struct {
	GPIO_TypeDef** Port;
	uint16_t Pin;
} KeypadControl;


/* Rows */
/* Row 1 default */
#ifndef KEYPAD_ROW_1_PIN
#define KEYPAD_ROW_1_PORT			GPIOC
#define KEYPAD_ROW_1_PIN			GPIO_Pin_6
#endif
/* Row 2 default */
#ifndef KEYPAD_ROW_2_PIN
#define KEYPAD_ROW_2_PORT			GPIOC
#define KEYPAD_ROW_2_PIN			GPIO_Pin_7
#endif
/* Row 3 default */
#ifndef KEYPAD_ROW_3_PIN
#define KEYPAD_ROW_3_PORT			GPIOC
#define KEYPAD_ROW_3_PIN			GPIO_Pin_8
#endif
/* Row 4 default */
#ifndef KEYPAD_ROW_4_PIN
#define KEYPAD_ROW_4_PORT			GPIOC
#define KEYPAD_ROW_4_PIN			GPIO_Pin_9
#endif

/* Columns */
/* Column 1 default */
#ifndef KEYPAD_COLUMN_1_PIN
#define KEYPAD_COLUMN_1_PORT		GPIOA
#define KEYPAD_COLUMN_1_PIN			GPIO_Pin_8
#endif
/* Column 2 default */
#ifndef KEYPAD_COLUMN_2_PIN
#define KEYPAD_COLUMN_2_PORT		GPIOA
#define KEYPAD_COLUMN_2_PIN			GPIO_Pin_9
#endif
/* Column 3 default */
#ifndef KEYPAD_COLUMN_3_PIN
#define KEYPAD_COLUMN_3_PORT		GPIOA
#define KEYPAD_COLUMN_3_PIN			GPIO_Pin_10
#endif
/* Column 4 default */
#ifndef KEYPAD_COLUMN_4_PIN
#define KEYPAD_COLUMN_4_PORT		GPIOA
#define KEYPAD_COLUMN_4_PIN			GPIO_Pin_15
#endif



/* Number of milliseconds between 2 reads */
#ifndef KEYPAD_READ_INTERVAL
#define KEYPAD_READ_INTERVAL        100
#endif

/* Keypad no pressed */
#define KEYPAD_NO_PRESSED			(uint8_t)0xFF

char read_keypad_key(void);
void setColHight(int col);
int read_active_colomn(void);
void keypadRowLow(GPIO_TypeDef* PORT, uint16_t PIN);
void keypadRowHight(GPIO_TypeDef* PORT, uint16_t PIN);
void initgpio_keyboard(void);
void keypad_init_GPIO_OUT(GPIO_TypeDef* PORT, uint16_t Pin);
void keypad_init_GPIO_IN(GPIO_TypeDef* PORT, uint16_t Pin );
