# LED Blink Example
On STM32F7 Discovery board, there are two LED's LD1 and LD2.  
LD1 is Red LED and is connected to PJ13 pin of the micro-controller.  
While LD2 is Green LED and is connected to PJ5 pin of the micro-controller.  

**NOTE: To save space consumption on GitHub, `Project Manager ` setting is selected as to `Add necessary library files as referen in the toolchain project configuration file`  
So, to use this example, please make sure to select the correct setting.**  

Development Environment: STM32CubeIDE 1.6.1, STM32CubeMX 6.1.2, STM32CubeMonior 1.1.0  
STM32Cube MCU Package for STM32F7 Series Version 1.16.1  

#### Led Blink Example Method-1
In this method, ```HAL_GPIO_WritePin``` function is used to set and clear the LED GPIO's of the micro-controller.  

#### Led Blink Example Method-2
In this method, ```HAL_GPIO_TogglePin``` function is used to set and clear the LED GPIO's of the micro-controller.  

#### Led Blink Example Method-3
The drawback of the above two examples is that there is delay of 1000 milliseconds, which blocks other task, this example, uses ```HAL_GetTick()``` function, so execute the individual Led On-Off Tasks, this method is non-blocking in nature.  
The following is the gif image of the working demo, which is prepared using CubeMonitor Software.  
![](./../Others/01-HelloWorld.gif)
