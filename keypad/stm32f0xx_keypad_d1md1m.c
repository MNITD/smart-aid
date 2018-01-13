#include "stm32f0xx_keypad_d1md1m.h"


/* Pins configuration, columns are outputs */
#define KEYPAD_COLUMN_1_HIGH		GPIO_SetBits(KEYPAD_COLUMN_1_PORT, KEYPAD_COLUMN_1_PIN)
#define KEYPAD_COLUMN_1_LOW			GPIO_ResetBits(KEYPAD_COLUMN_1_PORT, KEYPAD_COLUMN_1_PIN)
#define KEYPAD_COLUMN_2_HIGH		GPIO_SetBits(KEYPAD_COLUMN_2_PORT, KEYPAD_COLUMN_2_PIN)
#define KEYPAD_COLUMN_2_LOW			GPIO_ResetBits(KEYPAD_COLUMN_2_PORT, KEYPAD_COLUMN_2_PIN)
#define KEYPAD_COLUMN_3_HIGH		GPIO_SetBits(KEYPAD_COLUMN_3_PORT, KEYPAD_COLUMN_3_PIN)
#define KEYPAD_COLUMN_3_LOW			GPIO_ResetBits(KEYPAD_COLUMN_3_PORT, KEYPAD_COLUMN_3_PIN)
#define KEYPAD_COLUMN_4_HIGH		GPIO_SetBits(KEYPAD_COLUMN_4_PORT, KEYPAD_COLUMN_4_PIN)
#define KEYPAD_COLUMN_4_LOW			GPIO_ResetBits(KEYPAD_COLUMN_4_PORT, KEYPAD_COLUMN_4_PIN)

#define TM_GPIO_GetInputPinValue(GPIOx, GPIO_Pin)	(((GPIOx)->IDR & (GPIO_Pin)) == 0 ? 0 : 1)

/* Read input pins */
#define KEYPAD_ROW_1_CHECK			(!TM_GPIO_GetInputPinValue(KEYPAD_ROW_1_PORT, KEYPAD_ROW_1_PIN))
#define KEYPAD_ROW_2_CHECK			(!TM_GPIO_GetInputPinValue(KEYPAD_ROW_2_PORT, KEYPAD_ROW_2_PIN))
#define KEYPAD_ROW_3_CHECK			(!TM_GPIO_GetInputPinValue(KEYPAD_ROW_3_PORT, KEYPAD_ROW_3_PIN))
#define KEYPAD_ROW_4_CHECK			(!TM_GPIO_GetInputPinValue(KEYPAD_ROW_4_PORT, KEYPAD_ROW_4_PIN))

uint8_t KEYPAD_INT_Buttons[4][4] = {
	{0x01, 0x02, 0x03, 0x0C},
	{0x04, 0x05, 0x06, 0x0D},
	{0x07, 0x08, 0x09, 0x0E},
	{0x0A, 0x00, 0x0B, 0x0F},
};


/* Private functions */
void TM_KEYPAD_INT_SetColumn(uint8_t column);
uint8_t TM_KEYPAD_INT_CheckRow(uint8_t column);
uint8_t TM_KEYPAD_INT_Read(void);

/* Private variables */
//TM_KEYPAD_Type_t TM_KEYPAD_INT_KeypadType;
static TM_KEYPAD_Button_t KeypadStatus = TM_KEYPAD_Button_NOPRESSED;
//GPIO_Speed_2MHz;
//uint32_t GPIO_Pin;
//GPIOMode_TypeDef GPIO_Mode;
//GPIOSpeed_TypeDef GPIO_Speed;
//GPIOOType_TypeDef GPIO_OType;
//GPIOPuPd_TypeDef GPIO_PuPd;

void D1m_GPIO_init(
		GPIO_TypeDef* GPIOx,
		uint16_t GPIO_Pin,
		GPIOMode_TypeDef GPIO_Mode,
		GPIOSpeed_TypeDef GPIO_Speed,
		GPIOOType_TypeDef GPIO_OType,
		GPIOPuPd_TypeDef GPIO_PuPd )
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode;
		GPIO_InitStructure.GPIO_OType = GPIO_OType;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void D1m_KEYPAD_Init(void){
		/* Set keyboard type */
//		TM_KEYPAD_INT_KeypadType = type;

		/* Columns are output */
		/* Column 1 */

	D1m_GPIO_init(KEYPAD_COLUMN_1_PORT, KEYPAD_COLUMN_1_PIN, GPIO_Mode_OUT, GPIO_OType_PP, GPIO_PuPd_UP, GPIO_Speed_10MHz);
		/* Column 2 */
	D1m_GPIO_init(KEYPAD_COLUMN_2_PORT, KEYPAD_COLUMN_2_PIN, GPIO_Mode_OUT, GPIO_OType_PP, GPIO_PuPd_UP, GPIO_Speed_10MHz);
		/* Column 3 */
	D1m_GPIO_init(KEYPAD_COLUMN_3_PORT, KEYPAD_COLUMN_3_PIN, GPIO_Mode_OUT, GPIO_OType_PP, GPIO_PuPd_UP, GPIO_Speed_10MHz);
		/* Column 3 */

	D1m_GPIO_init(KEYPAD_COLUMN_4_PORT, KEYPAD_COLUMN_4_PIN, GPIO_Mode_OUT, GPIO_OType_PP, GPIO_PuPd_UP, GPIO_Speed_10MHz);


		/* Rows are inputs */
		/* Row 1 */
	D1m_GPIO_init(KEYPAD_ROW_1_PORT, KEYPAD_ROW_1_PIN, GPIO_Mode_IN, GPIO_OType_PP, GPIO_PuPd_DOWN, GPIO_Speed_10MHz);
		/* Row 2 */
	D1m_GPIO_init(KEYPAD_ROW_2_PORT, KEYPAD_ROW_2_PIN, GPIO_Mode_IN, GPIO_OType_PP, GPIO_PuPd_DOWN, GPIO_Speed_10MHz);
		/* Row 3 */
	D1m_GPIO_init(KEYPAD_ROW_3_PORT, KEYPAD_ROW_3_PIN, GPIO_Mode_IN, GPIO_OType_PP, GPIO_PuPd_DOWN, GPIO_Speed_10MHz);
		/* Row 4 */
	D1m_GPIO_init(KEYPAD_ROW_4_PORT, KEYPAD_ROW_4_PIN, GPIO_Mode_IN, GPIO_OType_PP, GPIO_PuPd_DOWN, GPIO_Speed_10MHz);

		/* All columns high */
		TM_KEYPAD_INT_SetColumn(0);
	}

/* Private */
void TM_KEYPAD_INT_SetColumn(uint8_t column) {
	/* Set rows high */
	KEYPAD_COLUMN_1_HIGH;
	KEYPAD_COLUMN_2_HIGH;
	KEYPAD_COLUMN_3_HIGH;
	KEYPAD_COLUMN_4_HIGH;

	/* Set column low */
	if (column == 1) {
		KEYPAD_COLUMN_1_LOW;
	}
	if (column == 2) {
		KEYPAD_COLUMN_2_LOW;
	}
	if (column == 3) {
		KEYPAD_COLUMN_3_LOW;
	}
	if (column == 4) {
		KEYPAD_COLUMN_4_LOW;
	}
}


