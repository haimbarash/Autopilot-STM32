

#include "stm32f10x_tim.h"
#include "motor.h"
#include "stdlib.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
void mDelay(long long i)
{ 
  while(i!=0)
  i--; 
}

void Motor_Init()
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

  /* TIM3 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

  /* GPIOA and GPIOB clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |
                         RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);

  /* GPIOC Configuration: alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  /* GPIOC Configuration: output push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  /* GPIOA Configuration: output push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | \
                                GPIO_Pin_10 | GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /*pwm output from PC6,7*/
  GPIO_PinRemapConfig  ( GPIO_FullRemap_TIM3 ,ENABLE  ) ;


  unsigned int  BasePrescale;
  BasePrescale=0; /* pwm freqency = 24000/resulution [kHz] (resulution is set in header)*/

  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = resulution;
  TIM_TimeBaseStructure.TIM_Prescaler = BasePrescale;
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

  /* PWM1,2 Mode configuration: Channel1,2 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OC1Init(TIM3, &TIM_OCInitStructure);
  TIM_OC2Init(TIM3, &TIM_OCInitStructure);
  
  TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
  TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);

  TIM_ARRPreloadConfig(TIM3, ENABLE);
  
  /* TIM3 enable counter */
  TIM_Cmd(TIM3, ENABLE);
  
  /* !STBY set (motor ON) and mark by green led ON*/
  GPIO_WriteBit (GPIOC,GPIO_Pin_9, Bit_SET);

}

void Motor_Drive(unsigned int MOTORx, int Duty)
{
  /*Check Parameters*/
  while( !((MOTORx==MOTOR1) ^ (MOTORx==MOTOR2)) );
  while( (abs(Duty)>resulution));   
  
  if (MOTORx==MOTOR1)
  {
    if (Duty>=0)
    {
     TIM_SetCompare1(TIM3, Duty);
     GPIO_WriteBit (GPIOA,GPIO_Pin_9, Bit_RESET);
     GPIO_WriteBit (GPIOA,GPIO_Pin_8, Bit_SET);
    }
    if (Duty<0)
    {
     TIM_SetCompare1(TIM3, -Duty);
     GPIO_WriteBit (GPIOA,GPIO_Pin_8, Bit_RESET);
     GPIO_WriteBit (GPIOA,GPIO_Pin_9, Bit_SET);
    }
  }
  if (MOTORx==MOTOR2)
  {
    if (Duty>=0)
    {
      TIM_SetCompare2(TIM3, Duty);
      GPIO_WriteBit (GPIOA,GPIO_Pin_11, Bit_SET);
      GPIO_WriteBit (GPIOA,GPIO_Pin_10, Bit_RESET);
    }
    if (Duty<0)
    {
      TIM_SetCompare2(TIM3, -Duty);
      GPIO_WriteBit (GPIOA,GPIO_Pin_10, Bit_SET);
      GPIO_WriteBit (GPIOA,GPIO_Pin_11, Bit_RESET);
    }
  }
}

void Motor_Stop(unsigned int MOTORx)
{
  /*Check Parameters*/
  while( !((MOTORx==MOTOR1) ^ (MOTORx==MOTOR2)) );
  
  if (MOTORx==MOTOR1)
  {
    TIM_SetCompare1(TIM3, 0);
    GPIO_WriteBit (GPIOA,GPIO_Pin_8, Bit_RESET);
    GPIO_WriteBit (GPIOA,GPIO_Pin_9, Bit_RESET);
  }
  if (MOTORx==MOTOR2)
  {
    TIM_SetCompare2(TIM3, 0);
    GPIO_WriteBit (GPIOA,GPIO_Pin_10, Bit_RESET);
    GPIO_WriteBit (GPIOA,GPIO_Pin_11, Bit_RESET);
  }
}
void Motor_Standby()
{ 
  Motor_Stop(MOTOR1);
  Motor_Stop(MOTOR2);
  /* !STBY set (motor OFF)*/
  GPIO_WriteBit (GPIOC,GPIO_Pin_9, Bit_RESET);

}