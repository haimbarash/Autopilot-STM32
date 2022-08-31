/*------------------------------------------------------------------------------
Interrupt Handler Program
This program contains all interrupt routines:
------------------------------------------------------------------------------*/
/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "Sensors.h"
//global struct variables definition:
extern struct Sensors current_Sensors;
extern struct Sensors prev_Sensors;
int wantedDistance=1100; // for 4-30 sensor wanted position=1100; for 10-80 sensor wanted position=1988(not
sure);
int max_PWM=6000; //maximum PWM value
int min_PWM=3000; //Minimum PWM value
//LPF parameters:
double alpha_optic = 0.025; //LPF for autopilot- alpha_optic=0.025
double alpha_dis=0.01; //LPF for ACC- alpha_optic=0.025
//distance parameters:
int distance_value=0;
//P_distance controller parameters
int motor_PWM_limit=0;
int distance_error=0;
double k_p_dis=14; //P controller value;
//Adaptive cruise control parameters:
int avg_motor=0;
double motor_factor=0;
//standstill counter:
int stand_counter=0;
//PID parameters
double dt=1/20;//dt in sec
int k_p=2;//Proportional
int k_i=1;//Integral
int k_d=2;//Deriveter
int current_position=0;
int error_position=0;
int integral_position=0;
int derivater_position=0;
int previous_error_position=0;
int motor_PID_output;
//engine maneuver parameters
int RightMotor=0;
int LeftMotor=0;
/*-----------------------------------------------------------*/
void itDelay(long long i)
{
	while(i!=0)
	i--;
}
// Interrupt deffined also in NVIC controller in main.c file//
/******************************************************************************/
/* STM32F10x Peripherals Interrupt Handlers */
/* Add here the Interrupt Handler for the used peripheral(s) (PPP), for the */
/* available peripheral interrupt handler's name please refer to the startup */
/* file (startup_stm32f10x_xx.s). */
/******************************************************************************/
void TIM2_IRQHandler(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;//GPIO Configuration
	/* GPIOC Configuration: output push-pull button */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	//LPF Sensors structs Configuration:
	struct Sensors current_Sensors=Read_Sensors();
	struct Sensors prev_Sensors;
	if(TIM_GetITStatus(TIM2,TIM_FLAG_Update)==SET)
	{
		TIM_ClearITPendingBit(TIM2,TIM_FLAG_Update);
		//LPF for sensors input:
		prev_Sensors=current_Sensors;
		current_Sensors=Read_Sensors();
		for (int i=0; i<8;i++)// read from 8 sensor photo-cells
		{
			current_Sensors.array_sensor[i]=alpha_optic*current_Sensors.array_sensor[i]+(1-alpha_optic)*prev_Sensors.array_sensor[i];
		}
		current_Sensors.distance=alpha_dis*current_Sensors.distance+(1-alpha_optic)*prev_Sensors.distance;//Low Pass Filter (LPF) formula
		distance_value=current_Sensors.distance;
		//position calculation
		current_position=(current_Sensors.array_sensor[0]*0+current_Sensors.array_sensor[1]*1000+current_Sensors.array_se
		nsor[2]*2000+current_Sensors.array_sensor[3]*3000+current_Sensors.array_sensor[4]*4000+current_Sensors.array_sens
		or[5]*5000+current_Sensors.array_sensor[6]*6000+current_Sensors.array_sensor[7]*7000)/(current_Sensors.array_sens
		or[0]+current_Sensors.array_sensor[1]+current_Sensors.array_sensor[2]+current_Sensors.array_sensor[3]+Current_Sen
		sors.array_sensor[4]+current_Sensors.array_sensor[5]+current_Sensors.array_sensor[6]+current_Sensors.array_sensor
		[7]);
		//P controller distance calculations:
		distance_error=wantedDistance-current_Sensors.distance;
		if(distance_value<600)// P controller minimum distance. should be less than wantedDistance
		{
			motor_PWM_limit=max_PWM;
		}
		if(distance_value>600)
			motor_PWM_limit=k_p_dis*distance_error;
		if(motor_PWM_limit>max_PWM)//Limits the PWM value
			motor_PWM_limit=max_PWM;
		if(motor_PWM_limit<=min_PWM) //when the robot stop
		{
			motor_PWM_limit=0; //Cancel option of reverse drive
			stand_counter++; // time counter
			if((stand_counter%10)==0)//flashing the Green led every 0.5 sec. 20Hz -> 20 iterations per sec. every 10 iteration the GPIO changes status
			{
				if (GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_9) == 1)
				{
					GPIO_WriteBit (GPIOC,GPIO_Pin_9, Bit_RESET );
				}
				else
				{
					GPIO_WriteBit (GPIOC,GPIO_Pin_9, Bit_SET );
				}
			}
			if(stand_counter==100)//finish the program after 5 sec. 100 iterations = 5 sec in 20Hz controller
			{
				Motor_Stop(MOTOR1);
				Motor_Stop(MOTOR2);
				exit();
			}
		}
		if(motor_PWM_limit>min_PWM&&stand_counter!=0)//resuming driving mode when the obstacle removed
		{
			stand_counter=0; // reset time counter 
			GPIO_WriteBit (GPIOC,GPIO_Pin_9, Bit_SET ); //turn on the green light= Drive mode
		}
		//PID controller position calculations:
		error_position=3500-current_position;//required position is 3500
		integral_position= integral_position+error_position*dt;
		derivater_position= (error_position-previous_error_position)/dt;
		motor_PID_output= k_p *error_position + k_i * integral_position + k_d * derivater_position ;
		previous_error_position= error_position;
		//PID maneuver values
		LeftMotor=3000-motor_PID_output;
		RightMotor=3000+motor_PID_output;
		//limits the motor to PWM of minimum 0:
		if(LeftMotor<0)
			LeftMotor=0;
		if(RightMotor<0)
			RightMotor=0;
		//calculation for proportional motor values to the motor_PWM_limit values for ACC
		avg_motor=(LeftMotor+RightMotor)/2;
		motor_factor=motor_PWM_limit/avg_motor;
		LeftMotor=LeftMotor*motor_factor;
		RightMotor=RightMotor*motor_factor;
		// makes sure non of the engines completely stop rotating while driving
		if(LeftMotor<min_PWM&&RightMotor>min_PWM)
			LeftMotor=min_PWM;
		if(LeftMotor>min_PWM&&RightMotor<min_PWM)
			RightMotor=min_PWM;
		//makes sure non of the engines over shoots the PWM limit
		if(LeftMotor>max_PWM)
			LeftMotor=max_PWM;
		if(RightMotor>max_PWM)
			RightMotor=max_PWM;
		//Motor drive command
		Motor_Drive(MOTOR1,LeftMotor); //Left PWM values must be non-negative integers between [-12000,12000] = [0,12000]
		Motor_Drive(MOTOR2,RightMotor); //Right PWM values must be non-negative integers between [-12000,12000] = [0,12000]
	}
}
void TIM4_IRQHandler(void)
{
	/* Toggle LED3 */
	if (TIM_GetITStatus(TIM4,TIM_FLAG_Update)==SET)
	{
		TIM_ClearITPendingBit( TIM4, TIM_FLAG_Update);
		GPIO_WriteBit ( GPIOC,GPIO_Pin_8,(BitAction) 1) ;
	}
	else if (TIM_GetITStatus(TIM4, TIM_IT_CC1)==SET)
	{
		TIM_ClearFlag( TIM4, TIM_IT_CC1);
		GPIO_WriteBit ( GPIOC,GPIO_Pin_8,(BitAction) 0) ;
	}
}
/*----------------------------------------------------------------------------*/
/******************************************************************************/
/* Cortex-M3 Processor Exceptions Handlers */
/******************************************************************************/
/**
* @brief This function handles NMI exception.
* @param None
* @retval None
*/
void NMI_Handler(void)
{
}
/**
* @brief This function handles Hard Fault exception.
* @param None
* @retval None
*/
void HardFault_Handler(void)
{
/* Go to infinite loop when Hard Fault exception occurs */
while (1)
{
}
}
/**
* @brief This function handles Memory Manage exception.
* @param None
* @retval None
*/
void MemManage_Handler(void)
{
/* Go to infinite loop when Memory Manage exception occurs */
while (1)
{
}
}
/**
* @brief This function handles Bus Fault exception.
* @param None
* @retval None
*/
void BusFault_Handler(void)
{
/* Go to infinite loop when Bus Fault exception occurs */
while (1)
{
}
}
/**
* @brief This function handles Usage Fault exception.
* @param None
* @retval None
*/
void UsageFault_Handler(void)
{
/* Go to infinite loop when Usage Fault exception occurs */
while (1)
{
}
}
/**
* @brief This function handles SVCall exception.
* @param None
* @retval None
*/
void SVC_Handler(void)
{
}
/**
* @brief This function handles Debug Monitor exception.
* @param None
* @retval None
*/
void DebugMon_Handler(void)
{
}
/**
* @brief This function handles PendSV_Handler exception.
* @param None
* @retval None
*/
void PendSV_Handler(void)
{
}
/**
* @brief This function handles SysTick Handler.
* @param None
* @retval None
*/
void SysTick_Handler(void)
{
}