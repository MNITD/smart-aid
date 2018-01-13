#include "APS.h"

GPIO_InitTypeDef G;

//Timekeeping functions
volatile uint32_t MSec = 0;
//int count = 10;
int pill = 0;

void SysTick_Handler(void){
	MSec++;
}

//Delay function (milliseconds)
void Delay(uint32_t T){
	uint32_t MSS = MSec;
	while((MSec-MSS)<T) __NOP;
}

int main(void)
{
	//Enable GPIO clock
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

	//Intialize AnyPinServo library
	APS_Init();

	//Set PA0 and PA1 as outputs
	G.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	G.GPIO_Mode = GPIO_Mode_OUT;
	G.GPIO_OType = GPIO_OType_PP;
	G.GPIO_PuPd = GPIO_PuPd_NOPULL;
	G.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &G);

	//Add PA0 and PA1 to the servo list
	uint8_t res;
	res = APS_AddPin(GPIOA, GPIO_Pin_0, APS_SERVOMIDDLE);
	if(res != AE_SUCCESS){
		//Do something with result
	}

	res = APS_AddPin(GPIOA, GPIO_Pin_1, APS_SERVOMIDDLE);
	if(res != AE_SUCCESS){
		//Do something with result
	}

	//Initialize millisecond counter
	SysTick_Config(SystemCoreClock/1000);
	int start_pos = 0;
	int left = 45;
	int right = -45;
	int count = 3;
	int phres = 1;
	while (1){
		if(count > 0) {
			int cur_pos = start_pos;
			APS_SetPositionDegree(GPIOA, GPIO_Pin_0, cur_pos);
			APS_WaitForUpdate();
			Delay(2000);	// 2 sec

			while(!pill) {		// pill not caught yet
				cur_pos += left;
				APS_SetPositionDegree(GPIOA, GPIO_Pin_0, cur_pos);
				APS_WaitForUpdate();
				Delay(2000);	// 2 sec

				cur_pos += right;
				APS_SetPositionDegree(GPIOA, GPIO_Pin_0, cur_pos);
				APS_WaitForUpdate();
				Delay(2000);	// 2 sec

				if (phres == 1) {	// check "pill caught"
					cur_pos += right;
					APS_SetPositionDegree(GPIOA, GPIO_Pin_0, cur_pos);
					APS_WaitForUpdate();
					Delay(2000);	// 2 sec
					pill = 1;		// pill caught
				}

			}
			count--;
			pill = 0;
		}
	}

}
