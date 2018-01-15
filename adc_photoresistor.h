
#include <stm32f0xx_rcc.h>
#include <stm32f0xx_gpio.h>
#include "stm32f0xx.h"

void ConfigureADC(void);
void EnableADC(void);
void  CalibrateADC(void);
void SetClockForADC(void);
void  ConfigureGPIO_ADC(void);
int launch_photoresistor(void);


#define LR_RED_PORT GPIOC
#define LR_RED_Pin GPIO_Pin_7

#define LR_BLUE_PORT GPIOC
#define LR_BLUE_Pin GPIO_Pin_8

#define LR_RED_DIOD_PORT GPIOA
#define LR_RED_DIOD_Pin GPIO_Pin_8

#define LR_BLUE_DIOD_PORT GPIOA
#define LR_BLUE_DIOD_Pin GPIO_Pin_9

uint16_t LR_RED_active, LR_RED_passive, LR_BLUE_active, LR_BLUE_passive;

void config_light_resistor_pin(GPIO_TypeDef* PORT, uint16_t PIN);
int read_from_LR(GPIO_TypeDef* PORT, uint16_t PIN);
void listenLR(GPIO_TypeDef* PORT, uint16_t PIN);
void unlistenLR(GPIO_TypeDef* PORT, uint16_t PIN);
void calibrate_light_resistor(GPIO_TypeDef* LR_PORT, uint16_t LR_PIN);// changes global values
void turnON_LR_DIOD(GPIO_TypeDef* PORT, uint16_t PIN);
void turnOFF_LR_DIOD(GPIO_TypeDef* PORT, uint16_t PIN);

