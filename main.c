/*------------------------------------------------------------------------------
			<-----	Robot Program 2020	----->
------------------------------------------------------------------------------*/
/* Includes ------------------------------------------------------------------*/
#include "motor.h"
#include "Sensors.h"
#include "stm32f10x_tim.h"
#include "extra_built_in_functions.h"
/* Global typedef -----------------------------------------------------------*/
/* Global define ------------------------------------------------------------*/
/* Global macro -------------------------------------------------------------*/
/* Global variables ---------------------------------------------------------*/
struct Sensors sensors_read; /*Sensor Type Stracture- Store sensor data*/
struct Sensors Current_Sensors;
struct Sensors Old_Sensors;
NVIC_InitTypeDef NVIC_InitStructure; /*Initialize the interrup on the NVIC Controller*/
TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure; /*Time Stracture */

//PID position parameters

void main(void)
{
	/*!< At this stage the microcontroller clock setting is already configured,
	this is done through SystemInit() function which is called from startup
	file (startup_stm32f10x_it.s) before to branch to application main.
	To reconfigure the default setting of SystemInit() function, refer to
	system_stm32f10x.c file
	*/
	Blue_button_Init(); //Initialize button input
	Motor_Init(); //Initialize motor input
	Sensor_Reading_Init(); //Initialize Sensor input
	Blue_Led_Init(); //Initialize timer4-OC1 to synchronize blue led
	TIM_Cmd(TIM4, ENABLE); //Turn blue led ON
	/******* Stuck here until button is pushed *******************/
	while (!GPIO_ReadInputDataBit (GPIOA, GPIO_Pin_0));
	while (GPIO_ReadInputDataBit (GPIOA, GPIO_Pin_0));
	/*************************************************************/
	TIM_Cmd(TIM4, DISABLE); //Turn blue led OFF
	GPIO_WriteBit (GPIOC,GPIO_Pin_8, Bit_RESET); //Turn blue led OFF
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	/* Enable and set EXTI0 Interrupt to the lowest priority */
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = 23999;
	TIM_TimeBaseStructure.TIM_Prescaler = 49;//20HZ
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
	TIM_Cmd(TIM2,ENABLE);//starting the Interrupt
	while (1)
	{
	}//while end
}//void end
#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line)
{
/* Infinite loop */
while (1)
{
}
}
#endif