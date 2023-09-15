/*******************************************************************
 * RCSId: $Id: rotor_wififuncs.ino,v 1.2 2023/09/06 13:19:04 ralblas Exp $
 *
 * Project: rotordrive
 * Author: R. Alblas
 *
 * content: 
 *   Wifi related commands
 *
 * public functions:
 *   void CheckForConnections()
 *   void connect_wifi()
 *   void disconnect_wifi()
 *   void get_ntp()
 *   void send_keplers(WiFiClient RemoteClient,KEPLER *k)
 *   void send_refposition(WiFiClient RemoteClient,EPOINT *refpos)
 *   void send_pos(WiFiClient RemoteClient,GOTO_VAL *gv,char *name)
 *   void send_ctrldata(WiFiClient RemoteClient,ROTOR *AX_rot,ROTOR *EY_rot,GOTO_VAL *gv)
 *
 *
 * History: 
 *   
 * $Log: rotor_wififuncs.ino,v $
 * Revision 1.2  2023/09/06 13:19:04  ralblas
 * _
 *
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

#if USE_SGP4
#include "rotorctrl_sgp4.h"
#endif

// esp32-75EC4C
void CheckForConnections()
{
  if (Server.hasClient())
  {
    // If we are already connected to another computer, 
    // then reject the new connection. Otherwise accept
    // the connection. 
    if (RemoteClient.connected())
    {
      Serial.println("Connection rejected");
      Server.available().stop();
    }
    else
    {
      Serial.print("Connection accepted, IP address: ");
      Serial.println(WiFi.localIP());
      RemoteClient = Server.available();
    }
  }
}

// Connect to wifi as access point, use my_SSID2 / my_PASSWORD2
void connect_wifi_ap(char *ssid,char *pwd)
{
  Serial.println("Creating wifi access point: ");
  Serial.println(ssid);
  Serial.println(pwd);
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, pwd);
}

// Connect to wifi as station', use my_SSID1 / 'my_PASSWORD1'
void connect_wifi(char *ssid,char *pwd)
{
  Serial.print("Connecting to wifi: ");
  Serial.println(ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pwd);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void disconnect_wifi()
{
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
}

// yyyy-mm-dd_HH:MM:SS
void set_time(char *str)
{
  time_t t;
  struct tm tm;
  struct timeval curtime;

  memset(&tm,0,sizeof(tm));
  strptime(str,"%Y-%m-%d_%H:%M:%S",&tm);
  curtime.tv_sec = mktime(&tm);
  curtime.tv_usec=0;
  xprintf("time set to %s\n",ctime(&curtime.tv_sec));
  settimeofday(&curtime, NULL);
}

// Get time from Wifi
void get_ntp()
{
  struct tm tm;
  if (!(getLocalTime(&tm)))
  {
    xprintf("set time from ntp failed!\n");
  }
  else
  {
    xprintf("Got time from ntp.\n");
  }
}


#if USE_SGP4
// Send current keplers back to PC (for checking)
void send_keplers(WiFiClient RemoteClient,KEPLER *k,boolean use_degrees)
{
  char sb[30];
  sprintf(sb,"satname=%s\n",k->name);
  RemoteClient.write((uint8_t *)sb, strlen(sb));
  sprintf(sb,"epoch_year=%d\n",k->epoch_year);
  RemoteClient.write((uint8_t *)sb, strlen(sb));
  sprintf(sb,"epoch_day=%f\n",k->epoch_day);
  RemoteClient.write((uint8_t* )sb, strlen(sb));
  sprintf(sb,"decay_rate=%f\n",k->decay_rate);
  RemoteClient.write((uint8_t* )sb, strlen(sb));
  sprintf(sb,"bstar=%f\n",k->bstar);
  RemoteClient.write((uint8_t* )sb, strlen(sb));

  if (use_degrees)
    sprintf(sb,"inclination=%f\n",k->d_inclination);
  else
    sprintf(sb,"inclination=%f\n",k->inclination);
  RemoteClient.write((uint8_t* )sb, strlen(sb));

  if (use_degrees)
    sprintf(sb,"raan=%f\n",k->d_raan);
  else
    sprintf(sb,"raan=%f\n",k->raan);
  RemoteClient.write((uint8_t* )sb, strlen(sb));

  sprintf(sb,"eccentricity=%f\n",k->eccentricity);
  RemoteClient.write((uint8_t* )sb, strlen(sb));

  if (use_degrees)
    sprintf(sb,"perigee=%f\n",k->d_perigee);
  else
    sprintf(sb,"perigee=%f\n",k->perigee);
  RemoteClient.write((uint8_t* )sb, strlen(sb));

  if (use_degrees)
    sprintf(sb,"anomaly=%f\n",k->d_anomaly);
  else
    sprintf(sb,"anomaly=%f\n",k->anomaly);
  RemoteClient.write((uint8_t* )sb, strlen(sb));

  sprintf(sb,"motion=%f\n",k->motion);
  RemoteClient.write((uint8_t* )sb, strlen(sb));
}

// Send current reference back to PC (for checking)
void send_refposition(WiFiClient RemoteClient,EPOINT *refpos)
{
  char str[50];
  char sdig[2][10];
  dtostrf(R2D(refpos->lat),0, 2, sdig[0]);
  dtostrf(R2D(refpos->lon),0, 2, sdig[1]);
  snprintf(str,50,"refpos=[%s,%s]\n",sdig[0],sdig[1]);
//  xprintf(str);
  RemoteClient.write((uint8_t *)str, strlen(str));
}

#endif

void send_ctrltime(WiFiClient RemoteClient)
{
  char str[100];
  struct tm *tm;
  time_t t;
  time(&t);
  tm=gmtime(&t);
  if (tm)
  {
    strftime(str,100,"time=%F_%T\n",tm);
    RemoteClient.write((uint8_t *)str, strlen(str));
  }
}

// Send current position back to PC (can be local calculated or received position)
void send_pos(WiFiClient RemoteClient,GOTO_VAL *gv,char *name)
{
  char str[100];        // max. stringlen=23+4*6+2*3=53
  char sdig[2][10];
  dtostrf(gv->lat,0, 1, sdig[0]);
  dtostrf(gv->lon,0, 1, sdig[1]);
  snprintf(str,100,"subsat=[%s,%s]\n",sdig[0],sdig[1]);
  RemoteClient.write((uint8_t *)str, strlen(str));
}

// Send all current rotor info  back to PC
void send_ctrldata(WiFiClient RemoteClient,ROTOR *AX_rot,ROTOR *EY_rot,GOTO_VAL *gv)
{
  char sdig[4][10];
  char str[100];
  float axpos_degr=0.,axreq_degr=0.,eypos_degr=0.,eyreq_degr=0.;
  float a[2],b[2];
  int ax_speed=0,ey_speed=0;
  int swap=(SWAP_DIR? -1 : 1);
  if (AX_rot) 
  {
    axpos_degr=AX_rot->degr*swap;
    axreq_degr=AX_rot->req_degr;
    ax_speed  =AX_rot->speed;
  }
  if (EY_rot) 
  {
    eypos_degr=EY_rot->degr*swap;
    eyreq_degr=EY_rot->req_degr;
    ey_speed  =EY_rot->speed;
  }
  // floating: -123.4 so min. len buf=6 + 1 (for '0')
  dtostrf(axpos_degr,0, 1, sdig[0]);
  dtostrf(eypos_degr,0, 1, sdig[1]);
  dtostrf(axreq_degr,0, 1, sdig[2]);
  dtostrf(eyreq_degr,0, 1, sdig[3]);
  // max. len=23+4*6+2*3=53
  snprintf(str,100,"pos=[%s,%s] req=[%s,%s] spd=[%d,%d]\n",sdig[0],sdig[1],sdig[2],sdig[3],ax_speed,ey_speed);
//  xprintf(str);
  RemoteClient.write((uint8_t *)str, strlen(str));

  dtostrf(gv->x,0, 1, sdig[0]);
  dtostrf(gv->y,0, 1, sdig[1]);
  dtostrf(gv->a,0, 1, sdig[2]);
  dtostrf(gv->e,0, 1, sdig[3]);
  snprintf(str,100,"xy=[%s,%s] ae=[%s,%s]\n",sdig[0],sdig[1],sdig[2],sdig[3]);
//  xprintf(str);
  RemoteClient.write((uint8_t *)str, strlen(str));

  dtostrf(gv->ax,0, 1, sdig[0]);
  dtostrf(gv->ey,0, 1, sdig[1]);
  snprintf(str,100,"ax/ey=[%s,%s] ew=%d\n",sdig[0],sdig[1],gv->east_pass);
//  xprintf(str);
  RemoteClient.write((uint8_t *)str, strlen(str));

  dtostrf(gv->lat,0, 1, sdig[0]);
  dtostrf(gv->lon,0, 1, sdig[1]);
  snprintf(str,100,"subsat=[%s,%s]\n",sdig[0],sdig[1]);
  RemoteClient.write((uint8_t *)str, strlen(str));

  #if USE_SGP4
    send_ctrltime(RemoteClient);  // Send local used time back to PC

    dtostrf(gv->height,0, 1, sdig[0]);
    snprintf(str,100,"height=%s\n",sdig[0]);
    RemoteClient.write((uint8_t *)str, strlen(str));
  #endif
}
#endif
