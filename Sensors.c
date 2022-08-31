
#include "stm32f10x_adc.h"
#include "Sensors.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
  ADC_InitTypeDef ADC_InitStructure;
/* Private function prototypes -----------------------------------------------*/
  void GPIO_Configuration(void);
/* Private functions ---------------------------------------------------------*/
void rsDelay(long long i)
{ 
  while(i!=0)
  i--; 
}

void Sensor_Reading_Init(void)
{
  /* System clocks configuration ---------------------------------------------*/
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOA |
                         RCC_APB2Periph_GPIOB |RCC_APB2Periph_GPIOC, ENABLE);
  /* GPIO configuration ------------------------------------------------------*/
  GPIO_Configuration();

  /* Voltage regulator check */

  /* ADC1 configuration ------------------------------------------------------*/
  ADC_InitStructure.ADC_Mode = ADC_Mode_RegSimult;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfChannel = 9;
  ADC_Init(ADC1, &ADC_InitStructure);

  ADC_DiscModeChannelCountConfig  (ADC1,(uint8_t)1);
  ADC_DiscModeCmd (ADC1, ENABLE);
  
  int n=7; //n=0..7 0->high speed, 7->low speed
  /* ADC1 regular channels configuration */ 
  ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 1, n);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 2, n);    
  ADC_RegularChannelConfig(ADC1, ADC_Channel_15, 3, n);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 4, n);    
  ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 5, n);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 6, n);    
  ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 7, n);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 8, n);    
  ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 9, n);

  /* Enable ADC1 */
  ADC_Cmd(ADC1, ENABLE);

  /*Delay*/
  int count=100;
  while (count != 0)
    count--;
 

  /* Enable ADC1 reset calibaration register */   
  ADC_ResetCalibration(ADC1);
  /* Check the end of ADC1 reset calibration register */
  while(ADC_GetResetCalibrationStatus(ADC1));
  /* Start ADC1 calibration */
  ADC_StartCalibration(ADC1);
  /* Check the end of ADC1 calibration */
  while(ADC_GetCalibrationStatus(ADC1));

}


void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Analog input config */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 |
                                GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 |
                                GPIO_Pin_6 | GPIO_Pin_7 ;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 ;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 |
                                GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  /* I/O config */
 
  /* AF config  */

}

struct Sensors Read_Sensors()
{
  struct Sensors temp;  //see header for Sensors type
    
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);
  while (!ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC));
  temp.array_sensor[0]=ADC_GetConversionValue (ADC1);   

  ADC_SoftwareStartConvCmd(ADC1, ENABLE);
  while (!ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC));
  temp.array_sensor[1]=ADC_GetConversionValue (ADC1);   

  ADC_SoftwareStartConvCmd(ADC1, ENABLE);
  while (!ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC));
  temp.array_sensor[2]=ADC_GetConversionValue (ADC1);   

  ADC_SoftwareStartConvCmd(ADC1, ENABLE);
  while (!ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC));
  temp.array_sensor[3]=ADC_GetConversionValue (ADC1);   

  ADC_SoftwareStartConvCmd(ADC1, ENABLE);
  while (!ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC));
  temp.array_sensor[4]=ADC_GetConversionValue (ADC1);   

  ADC_SoftwareStartConvCmd(ADC1, ENABLE);
  while (!ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC));
  temp.array_sensor[5]=ADC_GetConversionValue (ADC1);   

  ADC_SoftwareStartConvCmd(ADC1, ENABLE);
  while (!ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC));
  temp.array_sensor[6]=ADC_GetConversionValue (ADC1);   

  ADC_SoftwareStartConvCmd(ADC1, ENABLE);
  while (!ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC));
  temp.array_sensor[7]=ADC_GetConversionValue (ADC1);   

  ADC_SoftwareStartConvCmd(ADC1, ENABLE);
  while (!ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC));
  temp.distance=ADC_GetConversionValue (ADC1);   

/*****************************************
  temp.array_sensor[0]=Ain9;  //pin PB1, 
  temp.array_sensor[1]=Ain8;  //pin PB0, 
  temp.array_sensor[2]=Ain15; //pin PC5, 
  temp.array_sensor[3]=Ain14; //pin PC4, 
  temp.array_sensor[4]=Ain7;  //pin PA7, 
  temp.array_sensor[5]=Ain6;  //pin PA6, 
  temp.array_sensor[6]=Ain5;  //pin PA5, 
  temp.array_sensor[7]=Ain4;  //pin PA4, 

  temp.distance=Ain3;         //pin PA3, 
******************************************/
  return temp;
 }
