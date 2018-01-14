
#include <stm32f0xx_rcc.h>
#include <stm32f0xx_gpio.h>
#include "stm32f0xx.h"

void ConfigureADC(void);
void EnableADC(void);
void  CalibrateADC(void);
void SetClockForADC(void);
void  ConfigureGPIO_ADC(void);
int launch_photoresistor(void);


