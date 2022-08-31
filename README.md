# Autopilot-STM32
Full self-driving program including lane keeping and adaptive cruise control implementing on STM32

Make sure to watch the video “”, also available at: youtube

The purpose of the project:

Implement an adaptive cruise control (AAC) algorithm, on a robot with a STM32 controller.
Autopilot: The robot must follow a track marked with a black line on a white background. The speed determined using the ACC algorithm.
The ACC algorithm:
1. When the robot does not recognize a vehicle/obstacle in front of it, it must move at maximum speed.

2. When the robot approaches a robot traveling in front of it, the robot must slow down and maintain a constant distance from the robot in front of it.
In addition, when the robot detects an obstacle in front of it or another robot that has stopped completely, it must stop traveling and flash the green LED.
If a stop of 5 seconds or more occurs, the program will end.

Some Explanations:

This project contains several files written in C, some of which are system files with given functions.

The file stm32x_it.c contains the autopilot and ACC algorithm.

The file main.c contains importing code files, defining global variables and definitions for using interrupts and timers.

The void main area defines the increase of the program accompanied by a flashing blue LED, until pressed, the blue button to start the program.

As soon as the program starts, various settings are made for interrupts and timers, and then the interrupt in which the robot's travel algorithm is found is called.
A call to an interrupt triggered by a rising clock. (Refer to line 54 in main.c).
The TIM2 interrupt is inside the stm32f10x_it.c file in which the following tasks are performed:
1. Reading the sensors and using Low Pass Filter:
 Performed according to the formula:
(For more information about LPF and why its uses to read data from sensor read https://en.wikipedia.org/wiki/Low-pass_filter ): 
Y[n] = αX[n] + (1 – α) * Y[n – 1]
When a different α value was set between the position sensor and the distance sensor, because the distance sensor has greater sensitivity to background noise disturbances (a smaller α value was set). The α value may be vary between different types of sensors or even between sensors from same model with different production series. (Refer to lines 70-77 in stm32f10x_it.c).
2. Adaptive cruise control:
Cruise control was performed by reading values from the distance sensor placed on the front of the robot. While using a proportional controller that operates within a defined distance range. (Refer to lines 86-95 in stm32f10x_it.c).

3. Maintaining a desired path
Maintaining a desired lane was performed by reading the 8 brightness sensors directed to the driving surface. The reading values of the 8 sensors were entered into the formula:
Position = (0*val1 + 1000*val2 + ….) /(val1 + val2 + ….)
In order to get a number that represents the robot’s position, where the position value ranges from 0 to 7000 and the value for traveling in the center of the path is 3500. The current position values of the robot and the position necessary were entered into the PID type controller in order to determine the rotation speed of each motor and control the robot's maneuvering. (Refer to lines 123-141 in stm32f10x_it.c).
PID Controller parameters (determined by experiments, may vary between different robots(:
dt = 1/20  [sec]
Kp = 2
Ki = 1
Kd = 2
3. Complete stop and pause of the robot for 5 seconds
When the distance sensor used for the cruise control detects a reading that is too close, the engines stop. At the same time, since the measurement rate set for the sensors is 20 Hz, a counter is activated that counts the number of calls made while the engines are stopped, while the green LED is flashing. When the counter arrives for 100 (5 seconds at a frequency of 20 Hz) the program ends by using the exit command, otherwise the program continues and the counter resets. (Refer to lines 96-117 in stm32f10x_it.c).
