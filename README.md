# STM32F7 Discovery Board Examples

### Buttons and LEDs
The black button B2 located top side is the reset of the micro-controller STM32F769NIH6.  
The blue button B1 located top side is available to be used as a digital input or as a wakeup-alternate function.  
When the button is depressed the logic state is 1, otherwise the logic state is 0.  
Four LEDs located top side are available for the user. From left to right the LEDs are LD1, LD2, LD3 and LD4 with colors green, orange, red and blue respectively.  
To light a LED a low-logic state 0 should be written in the corresponding GPIO register.  
| Reference | Color | Name | Comment |
| --------- | ----- | ---- | ------- |
| B1 | BLUE | USER | Alternate Function Wake-Up |
| B2 | BLACK | RESET | - |
| LD1 | RED | USER1 | PJ13 Pin of micro |
| LD2 | GREEN | USER2 | PJ5 Pin of micro |
| LD3 | GREEN | Arduino | PA12 Pin of micro |
