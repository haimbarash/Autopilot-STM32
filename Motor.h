   
/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

#define resulution (12000)    /* pwm freqency = 24000/resulution [kHz] */
#define MOTOR1 ((unsigned int) 1)
#define MOTOR2 ((unsigned int) 2)
 

void Motor_Drive(unsigned int MOTORx, int Duty);
void Motor_Init(void);
void Motor_Stop(unsigned int MOTORx);
void Motor_Standby(void);
