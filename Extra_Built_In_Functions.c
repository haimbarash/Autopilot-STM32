#include "extra_built_in_functions.h"

void Blue_button_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /*Initializing RCC peripheral to enable clock to peripherals */
  RCC_APB2PeriphClockCmd  ( RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC |
                           RCC_APB2Periph_AFIO, ENABLE) ;

  /* GPIOA Configuration: input floating */
  GPIO_InitStructure.GPIO_Pin    = GPIO_Pin_0 |  GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Speed  = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode   = GPIO_Mode_IN_FLOATING;
  GPIO_Init (GPIOA, &GPIO_InitStructure);


}

void Delay(unsigned long count)
{
  while (count != 0)
    count--;
}


void Blue_Led_Init(void)  //Initialize timer4-OC1  to synchronize blue led
{
  NVIC_InitTypeDef NVIC_InitStructure;
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_OCInitTypeDef TIM_OCInitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

  /* GPIOC Configuration: output push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  RCC_APB1PeriphClockCmd  (RCC_APB1Periph_TIM4,ENABLE);

  NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure); 

  TIM_TimeBaseStructure.TIM_Period = 2000;
  TIM_TimeBaseStructure.TIM_Prescaler = 5000;
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;
  TIM_OCInitStructure.TIM_Pulse = 1000;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OC1Init(TIM4, &TIM_OCInitStructure);

  TIM_ARRPreloadConfig (TIM4,   DISABLE);  
  TIM_CCPreloadControl (TIM4,   DISABLE);  
  TIM_ITConfig(TIM4, TIM_IT_Update | TIM_IT_CC1, ENABLE);
}
