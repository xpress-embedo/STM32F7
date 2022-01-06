/*********************************************************************
*                SEGGER MICROCONTROLLER SYSTEME GmbH                 *
*        Solutions for real time microcontroller applications        *
**********************************************************************
*                                                                    *
*        (c) 1996 - 2015  SEGGER Microcontroller Systeme GmbH        *
*                                                                    *
*        Internet: www.segger.com    Support:  support@segger.com    *
*                                                                    *
**********************************************************************

***** emWin - Graphical user interface for embedded applications *****
emWin is protected by international copyright laws.   Knowledge of the
source code may not be used to write a similar product.  This file may
only be used in accordance with a license and should not be re-
distributed in any way. We appreciate your understanding and fairness.
----------------------------------------------------------------------
File        : BASIC_HelloWorld.c
Purpose     : Simple demo drawing "Hello world"
----------------------------------------------------------------------
*/

#include "GUI.h"

/*********************************************************************
*
*       Public code
*
**********************************************************************
*/
/*********************************************************************
*
*       MainTask
*/
void MainTask(void)
{
  // Clear the Display
  GUI_Clear();
  // Select the Font
  GUI_SetFont(&GUI_Font32_1);
  // Display the String at Current Cursor Position
  /*--NOTE: \r\n will move the cursor to next line, else the next data will be
  printed on the same line--*/
  GUI_DispString("Hello world!\r\n");
  GUI_DispString("Getting Started with emWin Graphics Library\r\n");
  // Set Background Color
  GUI_SetBkColor(GUI_BLUE);
  GUI_DispStringAt("Writing at Specific Position", 0, 64);
  while(1);
}

/*************************** End of file ****************************/
