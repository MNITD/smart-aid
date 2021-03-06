
#define RCC_AHBPeriph_GPIOB               RCC_AHBENR_GPIOBEN
//#endif
#ifndef _I2C_LCD_H_
#define _I2C_LCD_H_

#include "stm32f0xx.h"
#include "stm32f0xx_i2c.h"
#include "stm32f0xx_gpio.h"

// ����� PCF8574
#define LCD_ADDR	0x27

// ����������������� ������ ������ ��� ������ ���
//#define LCD_2004
#define LCD_1602

// ������� �������� ������� ��������
//extern void delay(uint32_t t);
#define lcd_pause	TIM6delay_us(1000) // � ��� ������� �������� ��������
#define lcd_pause_short	TIM6delay_us(40) // � ��� ������� �������� ��������

#define lcd_pause	delay(1000) // � ��� ������� �������� ��������
#define lcd_pause_short	delay(40) // � ��� ������� �������� ��������

// �� ����� �������������.
// �������� ����� ����������� ������
#define PCF_P0	0
#define PCF_P1	1
#define PCF_P2	2
#define PCF_P3	3
#define PCF_P4	4
#define PCF_P5	5
#define PCF_P6	6
#define PCF_P7	7

// ������������ ����� ��� � ����������� ������. �������� ��� ���������������� ��� ����
#define DB4		PCF_P4
#define DB5		PCF_P5
#define DB6		PCF_P6
#define DB7		PCF_P7
#define EN		PCF_P2
#define RW		PCF_P1
#define RS		PCF_P0
#define BL		PCF_P3


void gpioInit(void);
void i2cInit(void);


// ��������� ����������
uint8_t backlightState;

// ������� API
void lcd_Init(void); // ������������� ���
void lcd_Backlight(uint8_t state); // ���������/���������� ���������
void lcd_Goto(uint8_t row, uint8_t col); // ������� �� ������/�������
void lcd_PrintC(const uint8_t *str); // ������� ������


// ������� ����������
void lcd_Send(uint8_t data); // ��������� �������� � ���
void lcd_Command(uint8_t com); // ��������� �������� � ���
void lcd_Data(uint8_t com);  // ��������� ������ � ���


int LCD_launch(void);
void LCD_test_run(void);
#endif
