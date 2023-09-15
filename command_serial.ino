/*******************************************************************
 * RCSId: $Id: command_serial.ino,v 1.1 2023/07/18 14:49:00 ralblas Exp $
 *
 * Project: rotordrive
 * Author: R. Alblas
 *
 * content: 
 *   Read commands from serial/USB
 *
 * public functions:
 *   void readCommand_serial()
 *
 * History: 
 *   
 * $Log: command_serial.ino,v $
 * Revision 1.1  2023/07/18 14:49:00  ralblas
 * Initial revision
 *
 * Revision 1.1  2021/07/29 08:27:25  ralblas
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

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "rotorctrl.h"

#define LENBUF 50
static int get_serdata(boolean newdat,char *obuf)
{
  static int more;
  int Received;
  static char buf2[LENBUF+1];
  char buf1[LENBUF+1],buf3[LENBUF+1],*p;

  if (newdat)
  {
    more=0;
    *buf2=0;
  }  
  if (!more)
  {
    int i=0;
    int ch;
    if (!Serial.available()) return 0;
    while ((i<LENBUF) && ((ch=Serial.read())>=0)) buf1[i++]=ch;
    buf1[i]=0;
    strncat(buf2,buf1,LENBUF);
  }
  *obuf=0;
  if ((p=strchr(buf2,'\n')))
  {
    *p=0;
    strcpy(buf3,p+1);
    strcpy(obuf,buf2);
    strcpy(buf2,buf3);
    more=1;
  }
  else
  {
    more=0;
  }
  return 1;
}

void readCommand_serial()
{
  if (Serial.available())
  {
    char obuf[LENBUF];
    boolean newdat=true;
    *obuf=0;
    while (get_serdata(newdat,obuf))
    {
      newdat=false;
      if (parse_cmd(obuf))
      {
        execute_cmd();
      }
      else
      {
        if (*obuf) xprintf("serial: Wrong command: %s\n",obuf);
      }
    }
  }
}
