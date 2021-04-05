# Hello World Example
On STM32F7 Discovery board, there are three LED's LD1, LD2 and LD3.  
LD1 is Red LED and is connected to PJ13 pin of the micro-controller.  
While LD2 is Green LED and is connected to PJ5 pin of the micro-controller.  
And LD3 is also Green LED and is connected to PA12 pin of the micro-controller.  

The board also has one Blue Button, which can be used to provide some input, and it is connected to PA0 pin of the micro-controller.  
On Board the USART1 pins (PA9 and PA10), can be used to connect with PC as Virtual COM Port.  

With the help of this example project, we learnt how to use following:  
* GPIO Set, Reset and Toggle
* Non-Blocking Software development using `HAL_GetTick()` function
* External Interrupt for GPIO's
* UART
  * Simple Transmission
  * Transmission Over Interrupt
  * Reception
* ADC

Development Environment: STM32CubeIDE 1.6.1, STM32CubeMX 6.1.2, STM32CubeMonior 1.1.0  
STM32Cube MCU Package for STM32F7 Series Version 1.16.1  

#### Led Blink Using HCL_GetTick and Using External Interrupt Event
The drawback of the cure delay is that this blocks the CPU time for doing other tasks, this example, uses ```HAL_GetTick()``` function, so execute the individual Led On-Off Tasks, this method is non-blocking in nature.  

The project is also configured to generate interrupt whenever there is a rising edge on pin PA0, which is connected to Blue Button.  
Whenever this interrupt is triggered, a variable is set, which is used to toggle the state of LD3.  

The following is the gif image of the working demo, which is prepared using CubeMonitor Software.  
![](./../Others/01-HelloWorld.gif)

#### UART
STM32F7-DISCO USART1 can be used as Virtual COM port for PC, **but make sure to use the correct USART1 pins, by default PA9 and PA10 pins are not assigned by CubeMX software and this needs to be manually change** .  
* `HAL_UART_Transmit` function is used to transmit data over UART.  
* `HAL_UART_Transmit_IT` function is used to transmit data over UART, just make sure `USART1 Global Interrupt` is enabled in _NVIC Settings_ of CubeMX.  
* `HAL_UART_Receive_IT` function is used to configure UART module to receive data into buffer specified as argument and length is also specified. This function is used along with `HAL_UART_RxCpltCallback` function, which is interrupt handler, triggered when data of length specified is received over UART port.  


**NOTE: To save space consumption on GitHub, `Project Manager ` setting is selected as to `Add necessary library files as referen in the toolchain project configuration file`  
So, to use this example, please make sure to select the correct setting.**  