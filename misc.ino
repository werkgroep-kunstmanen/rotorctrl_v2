/*******************************************************************
 * RCSId: $Id: misc.ino,v 1.2 2023/09/06 11:50:45 ralblas Exp $
 *
 * Project: rotordrive
 * Author: R. Alblas
 *
 * content: 
 *   some debug functions
 *
 * History: 
 * $Log: misc.ino,v $
 * Revision 1.2  2023/09/06 11:50:45  ralblas
 * _
 *
 * Revision 1.1  2023/07/17 19:59:30  ralblas
 * Initial revision
 *
 * Revision 1.2  2021/08/03 09:12:38  ralblas
 * _
 *
 * Revision 1.1  2021/07/29 08:15:44  ralblas
 * Initial revision
 *
 *
 *******************************************************************/
/*******************************************************************
 * Copyright (C) 2020 R. Alblas. 
 *
 * This is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License 
 * as published by the Free Software Foundation.
 * 
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this software. If not, write to the Free Software 
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 
 * 02111-1307, USA.
 ********************************************************************/
#include "rotorctrl.h"
#include <stdarg.h>
extern boolean do_feedback;

// Print over ethernet or serial.
//   serial print without blocking: only print if enough buffer!
//   (Otherwise big delays will occur if sent bits are not received by xtrack)
//    hm, if not received at PC: filling up buffer in PC, if full: TX and RX cont. on.
//    So if TX blocks also RX blocks??? 

#define STRLEN 60

#if USE_DISPLAY
// print string to display (if available)
void dprint(int x,int y,char *s)
{
  char str[STRLEN];
  va_list ap;
  va_start(ap,frmt);
  vsnprintf(str,STRLEN,frmt,ap);
  va_end(ap);
  if ((x>=0) && (y>=0))
  {
    lcd.setCursor(x, y);
    lcd.print(str);
  }
}
#endif

// print format to ethernet or serial
void xprintf(const char *frmt,...)
{
  char str[STRLEN];
  va_list ap;
  va_start(ap,frmt);
  vsnprintf(str,STRLEN,frmt,ap);
  va_end(ap);
  if  (Serial.availableForWrite())
  {
    Serial.print(str);
  }
  #if USE_WIFI
    if (RemoteClient.connected())
    {
      RemoteClient.write((uint8_t* )str, strlen(str));
    }
  #endif
}

#if USE_WIFI
// for debugging: check stack
void stackcheck()
{
  static int ss;
  int s1=uxTaskGetStackHighWaterMark(NULL);
  if (s1!=ss)
  {
    printf(">>>STACK: %d\n",s1);
    ss=s1;
  }
}
#endif

// blink buit-in LED n times with delay d ms
void blink(int n,int d)
{
  for (; n>0; n--)
  {
    digitalWrite(LED_BUILTIN, HIGH);   // LED on; command received
    delay(d);
    digitalWrite(LED_BUILTIN, LOW);    // LED off; command not (yet) received
    delay(d);
  }
}

// set 3-colour led
void set_led(ROTOR *rot,int rgb,boolean enable)
{
  if (!rot) return;
  if (!enable) return;
  if (rot->id==EY_ID)
  {
    if (PIN_R_EY>=0) digitalWrite(PIN_R_EY , (rgb&4? HIGH : LOW));
    if (PIN_G_EY>=0) digitalWrite(PIN_G_EY , (rgb&2? HIGH : LOW));
    if (PIN_B_EY>=0) digitalWrite(PIN_B_EY , (rgb&1? HIGH : LOW));
  }

  if (rot->id==AX_ID)
  {
    if (PIN_R_AX>=0) digitalWrite(PIN_R_AX , (rgb&4? HIGH : LOW));
    if (PIN_G_AX>=0) digitalWrite(PIN_G_AX , (rgb&2? HIGH : LOW));
    if (PIN_B_AX>=0) digitalWrite(PIN_B_AX , (rgb&1? HIGH : LOW));
  }
}
