/*******************************************************************
 * RCSId: $Id: command_wifi.ino,v 1.1 2021/07/29 08:27:25 ralblas Exp $
 *
 * Project: rotordrive
 * Author: R. Alblas
 *
 * content: 
 *   Read commands from Wifi
 *
 * public functions:
 *   void readCommand_wifi()
 *
 * History: 
 *   
 * $Log: rotorctrl_wifi.ino,v $
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

#if USE_WIFI 

#include <WiFi.h>
#include <time.h>
#if USE_SGP4
#include "rotorctrl_sgp4.h"
#endif

#define LENBUF 50
#define LENBUF2 100
static int get_tcpdata(boolean newdat,char *obuf)
{
  static int more;
  int Received;
  static char buf2[LENBUF2+1];
  char buf1[LENBUF+1],buf3[LENBUF+1],*p;

  if (newdat)
  {
    more=0;
    *buf2=0;
  }  
  if (!more)
  {
    if (!RemoteClient.available()) return 0;
    Received = RemoteClient.read((uint8_t *)buf1, LENBUF);
    if (Received<0) return 0;
    buf1[Received]=0;
    strncat(buf2,buf1,LENBUF2);
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
    more=0;
  return 1;
}


void readCommand_wifi()
{
  char obuf[LENBUF+1];
  boolean newdat=true;
  int key=0;

  CheckForConnections();
  if (RemoteClient.connected())
  {
    while (get_tcpdata(newdat,obuf))
    {
      newdat=false;
      if (parse_cmd(obuf))
      {
        execute_cmd();
      }
      else
      {
        if (*obuf) xprintf("wifi: Wrong command: %s\n",obuf);
      }
    }
  }
}
#endif
