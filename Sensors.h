
/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"


struct Sensors
{ 
  unsigned int array_sensor[8];
  unsigned int distance;
};

void Sensor_Reading_Init(void);
struct Sensors Read_Sensors();
