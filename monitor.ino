/*******************************************************************
 * RCSId: $Id: monitor.ino,v 1.2 2023/08/16 20:42:11 ralblas Exp $
 *
 * Project: rotordrive
 * Author: R. Alblas
 *
 * content: 
 *   monitor functions:
 *     void send_specs(ROTOR *AX_rot,ROTOR *EY_rot)
 *     void send_stat(ROTOR *AX_rot,ROTOR *EY_rot)
 *
 * History: 
 * $Log: monitor.ino,v $
 * Revision 1.2  2023/08/16 20:42:11  ralblas
 * _
 *
 * Revision 1.1  2023/07/17 18:57:33  ralblas
 * Initial revision
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
// Send rotor specs
// delay between 2 write actions, to prevent buffer overflow
#define SERDEL 20
void send_specs(ROTOR *AX_rot,ROTOR *EY_rot)
{
  char tmp[10];
  xprintf("SPEC: Release %s\n",RELEASE);                              delay(SERDEL);
  xprintf("SPEC: AX_POffset     =%ld\n",(long)AX_POffset);            delay(SERDEL);
  xprintf("SPEC: EY_POffset     =%ld\n",(long)EY_POffset);            delay(SERDEL);

  xprintf("SPEC: AX_REFPOS      =%s\n",dtostrf(AX_REFPOS,5,1,tmp));   delay(SERDEL);
  xprintf("SPEC: EY_REFPOS      =%s\n",dtostrf(EY_REFPOS,5,1,tmp));   delay(SERDEL);

  xprintf("SPEC: AX_STEPS_DEGR  =%ld\n",(long)AX_STEPS_DEGR);         delay(SERDEL);
  xprintf("SPEC: EY_STEPS_DEGR  =%ld\n",(long)EY_STEPS_DEGR);         delay(SERDEL);

  xprintf("SPEC: CAL_ZENITH     =%d\n",(int)CAL_ZENITH);              delay(SERDEL);

  #ifdef USE_EASTWEST
    xprintf("SPEC: USE_EASTWEST   =%ld\n",(long)USE_EASTWEST);          delay(SERDEL);
  #endif
  #ifdef FULLRANGE_AZIM
    xprintf("SPEC: FULLRANGE_AZIM =%ld\n",(long)FULLRANGE_AZIM);        delay(SERDEL);
  #endif

 #if MOTORTYPE == MOT_STEPPER
  xprintf("SPEC: AX_MotorSpeed  =%d\n",(int)AX_MotorSpeed);           delay(SERDEL);
  xprintf("SPEC: AX_MotorAccel  =%d\n",(int)AX_MotorAccel);           delay(SERDEL);
  xprintf("SPEC: EY_MotorSpeed  =%d\n",(int)EY_MotorSpeed);           delay(SERDEL);
  xprintf("SPEC: EY_MotorAccel  =%d\n",(int)EY_MotorAccel);           delay(SERDEL);
 #else
  xprintf("SPEC: AX_MINSPEED    =%d\n",(int)AX_MINSPEED);             delay(SERDEL);
  xprintf("SPEC: AX_MAXSPEED    =%d\n",(int)AX_MAXSPEED);             delay(SERDEL);
  xprintf("SPEC: EY_MINSPEED    =%d\n",(int)EY_MINSPEED);             delay(SERDEL);
  xprintf("SPEC: EY_MAXSPEED    =%d\n",(int)EY_MAXSPEED);             delay(SERDEL);
  xprintf("SPEC: L_DEGR_MAXSPEED=%d\n",(int)L_DEGR_MAXSPEED);         delay(SERDEL);
  xprintf("SPEC: H_DEGR_MINSPEED=%d\n",(int)H_DEGR_MINSPEED);         delay(SERDEL);
  xprintf("SPEC: D_DEGR_STOP    =%s\n",dtostrf(D_DEGR_STOP,5,1,tmp)); delay(SERDEL);
  xprintf("SPEC: PWMFreq        =%d\n",(int)PWMFreq);                 delay(SERDEL);
  xprintf("SPEC: MAX_PWM        =%d\n",(int)MAX_PWM);                 delay(SERDEL);
 #endif
 #if USE_SGP4
 {
   #include <time.h>
   time_t t;
   struct tm tm;
   time(&t);
   tm=*gmtime(&t);
   xprintf("TIME: %d-%02d-%02d %02d:%02d:%02d\n",tm.tm_year+1900,tm.tm_mon,tm.tm_mday,tm.tm_hour,tm.tm_min,tm.tm_sec);
  }
  #endif
  xprintf("END!\n");
}

// Send status
void send_stat(ROTOR *AX_rot,ROTOR *EY_rot)
{
  int stat_ax=-1,stat_ey=-1;
  if (AX_rot) stat_ax=AX_rot->cal_status;
  if (EY_rot) stat_ey=EY_rot->cal_status;
  xprintf("STAT: ax=%d  ey=%d\n",stat_ax,stat_ey);
}

#ifdef DISPLAY_FUNCS
// current ax/ey to display
void rec2displ(int ep,float ax,float ey)
{
  static int nr;
  char str1[10],str2[10];

  dtostrf(ax,6, 1, str1);
  dtostrf(ey,6, 1, str2);
  xdispprintf(0,0,"cmd: %d  %6s  %6s",ep,str1,str2);
  xdispprintf(0,1,"nr=%d",++nr);
}
#endif
