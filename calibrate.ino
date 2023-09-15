/*******************************************************************
 * RCSId: $Id: calibrate.ino,v 1.6 2023/09/05 17:36:31 ralblas Exp $
 *
 * Project: rotordrive
 * Author: R. Alblas
 *
 * content: 
 *   calibration of one or two motors
 *
 * public functions:
 *   void calibrate(ROTOR *AX_rot, ROTOR *EY_rot)
 *
 * History: 
 *   
 * $Log: calibrate.ino,v $
 * Revision 1.6  2023/09/05 17:36:31  ralblas
 * _
 *
 * Revision 1.5  2023/07/25 09:28:07  ralblas
 * _
 *
 * Revision 1.4  2023/07/25 08:02:54  ralblas
 * _
 *
 * Revision 1.2  2023/07/21 19:06:18  ralblas
 * _
 *
 * Revision 1.1  2023/07/19 12:51:29  ralblas
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
extern ROTOR AX_rot,EY_rot;


/*********************************************************************
 * Calibrate rotors
 *   run for some time both motors from start position
 *   run both motors back to start position = end switch
 *   run both motors 
 * return: 0 if OK, 1 if error
 * AX_rot/EY_rot have calibration status:
 *   0=not done yet
 *   1=start
 *   2=run forward
 *   3=stop forward
 *   4=run backwards to end-stop
 *   5=stop, run to offset  
 *   6=ready, set calibrating=false
 * If calibrating stays true: error; status shows which rotor gives error.
 * AX_rot or EY_rot may be NULL to calibrate a single rotor
 *********************************************************************/

static int run_to_cal_pos(ROTOR *AX_rot, ROTOR *EY_rot,float ax_calpos,float ey_calpos)
{
  int err=0;
  // Set pulse count to end-position = pulses needed to go to zero-position
  reset_to_pos(AX_rot,-1*AX_POffset);
  reset_to_pos(EY_rot,-1*EY_POffset);

  // Run to (0.,0.), so run for (AX_POffset,EY_POffset) steps.
  //   This corresponds with pos. (AX_REFPOS,EY_REFPOS)
  run_to_pos(AX_rot, EY_rot,0.,0.,false);
    
  if ((AX_rot) && (AX_rot->cal_status!=cal_ready)) err|=1;
  if ((EY_rot) && (EY_rot->cal_status!=cal_ready)) err|=2;
  run_motor_hard(AX_rot,0);
  run_motor_hard(EY_rot,0);
  if (err) return err;


  // Set pulse count to reference position
  if (AX_rot) AX_rot->degr=AX_REFPOS;
  if (AX_rot) AX_rot->rotated=from_degr(AX_rot);
  if (EY_rot) EY_rot->degr=EY_REFPOS;
  if (EY_rot) EY_rot->rotated=from_degr(EY_rot);
  return 0;
}

static void reset_for_cal(ROTOR *rot)
{
  boolean led_ena=true;
  if (!rot) return;
  set_led(rot,1,led_ena);              // set B: start cal.
  rot->calibrated=false;
  rot->cal_status=cal_started;
  reset_to_pos(rot,0);
}

static void check_run(ROTOR *rot)
{
  boolean led_ena=true;
  if (!rot) return;
  if (rot->rotated)
  {
    rot->cal_status=cal_got_pulses;    // got pulses
  }
  else
  {
    set_led(rot,4,led_ena);            // set R: not running
    xprintf((char *)"MES: rotor not running, at endstop?\n");
  }
}

static int get_zenpos(ROTOR *rot)
{
  int zen;
  if (!rot) return -1;
  if (rot->id==AX_ID)
  {
    #if AX_ZENPIN_INV == true
      zen=!digitalRead(rot->pin_zen);
    #else
      zen=digitalRead(rot->pin_zen);
    #endif
  }
  if (rot->id==EY_ID)
  {
    #if EY_ZENPIN_INV == true
      zen=!digitalRead(rot->pin_zen);
    #else
      zen=digitalRead(rot->pin_zen);
    #endif
  }
  return zen;
}

// set speed soft to go to zenith
static int set_zspeed(ROTOR *rot,int speed,int dir)
{
  int zen;
  if (!rot) return 0;
  if (dir) speed=speed*-1;
  zen=get_zenpos(rot);
  if (zen!=dir) speed=0;
  speed=run_motor_soft(rot,speed);
  return speed;
}

// goto zenith from either side
static int goto_zenit(ROTOR *AX_rot,ROTOR *EY_rot,int speed)
{
  int err=0;
  int ax_direct,ey_direct,ax_busy=1,ey_busy=1;
  unsigned long start_time=millis();
  if (AX_rot) ax_direct=get_zenpos(AX_rot);
  if (EY_rot) ey_direct=get_zenpos(EY_rot);
  do
  {
    if (millis()-start_time > ROT_TIMEOUT) { xprintf("TIMEOUT!\n"); break; }

    ax_busy=set_zspeed(AX_rot,speed,ax_direct);
    ey_busy=set_zspeed(EY_rot,speed,ey_direct);
  } while ((ax_busy) || (ey_busy));
  if (AX_rot) AX_rot->degr=AX_REFPOS;
  if (AX_rot) AX_rot->rotated=from_degr(AX_rot);
  if (EY_rot) EY_rot->degr=EY_REFPOS;
  if (EY_rot) EY_rot->rotated=from_degr(EY_rot);
  if (ax_busy) err|=1;
  if (ey_busy) err|=2;

  return err;
}

// calibrate using zenith detection
static int calibrate_zenith(ROTOR *AX_rot,ROTOR *EY_rot,int spd_cal1,int spd_cal2)
{
  boolean led_ena=true;
  int err;
  reset_for_cal(AX_rot);                   // led=B
  reset_for_cal(EY_rot);                   // led=B
  xprintf("Zenit status: %d  %d\n",get_zenpos(AX_rot),get_zenpos(EY_rot)); // 0: < 90, 1: > 90

  if (!spd_cal2) spd_cal2=spd_cal1;
  // Go to zenith from either side
  xprintf("Start calibration zenit step 1\n");

  err=goto_zenit(AX_rot,EY_rot,spd_cal1);
  set_led(AX_rot,(err&1? 4 : 7),led_ena);  // set R or RGB
  set_led(EY_rot,(err&2? 4 : 7),led_ena);  // set R or RGB
  if (err&1) set_status(AX_rot,cal_timeout);
  if (err&2) set_status(AX_rot,cal_timeout);
  if (err) return 1;

  // correct if from wrong side
  {
    ROTOR *AX_roti,*EY_roti;
    if (!get_zenpos(AX_rot)) AX_roti=AX_rot; else AX_roti=NULL;
    if (!get_zenpos(EY_rot)) EY_roti=EY_rot; else EY_roti=NULL;
    set_led(AX_roti,5,led_ena);               // set RB: start going to cal. pos
    set_led(EY_roti,5,led_ena);               // set RB: start going to cal. pos
    err=goto_zenit(AX_roti,EY_roti,(int)((spd_cal1+spd_cal2)/2));
    set_led(AX_roti,(err&1? 4 : 6),led_ena);  // set R or RG
    set_led(EY_roti,(err&2? 4 : 6),led_ena);  // set R or RG
    if (err&1) set_status(AX_rot,cal_timeout);
    if (err&2) set_status(AX_rot,cal_timeout);
    if (err) return 1;
  }

  // actual calibration
  xprintf("Start actual calibration to zenit\n");
  set_led(AX_rot, 6,led_ena);  // set R or RG
  set_led(EY_rot, 6,led_ena);  // set R or RG
  err=goto_zenit(AX_rot,EY_rot,spd_cal2);
  if (err&1) set_led(AX_rot,4,led_ena);    // set R
  if (err&2) set_led(EY_rot,4,led_ena);    // set R

  delay(1000);
  // goto exact 90 degrees
  err|=run_to_cal_pos(AX_rot,EY_rot,90.,90.);
  set_led(AX_rot,(err&1? 4 : 2),led_ena);  // set R or G
  set_led(EY_rot,(err&2? 4 : 2),led_ena);  // set R or G

  if (err&1) set_status(AX_rot,cal_timeout); else set_status(AX_rot,cal_ready);
  if (err&2) set_status(EY_rot,cal_timeout); else set_status(EY_rot,cal_ready);

  if (err&1) xprintf("Calibration error for AX!\n");
  else       xprintf("Calibration OK for AX.\n");
  if (err&2) xprintf("Calibration error for EY!\n");
  else       xprintf("Calibration OK for EY.\n");

/*
  if (AX_rot)
  {
    char pm;
    if (get_zenpos(AX_rot)) pm='+'; else pm='-';
    xprintf("Calibration ready:\n  AX from: %c\n",pm); 
  }
  if (EY_rot)
  {
    char pm;
    if (get_zenpos(EY_rot)) pm='+'; else pm='-';
    xprintf("Calibration ready:\n  EY from: %c\n",pm); 
  }
*/

  if (err) return 1;
  return 0; 
}

// calibrate using end stops
static int calibrate_estop(ROTOR *AX_rot,ROTOR *EY_rot,int spd_cal1,int spd_cal2)
{
  int i;
  int n=0;
  int err=0;
  int speed[2];
  boolean led_ena=true;

  float degr_forward=5.; // must be enough to move rotors from their end-switch!
  #if SWAP_DIR == false
    spd_cal1*=-1;
    spd_cal2*=-1;
  #endif

  speed[0]=spd_cal1;
  speed[1]=spd_cal2;

  for (i=0; i<2; i++)
  {
    if (!speed[i]) continue;
    reset_for_cal(AX_rot);
    reset_for_cal(EY_rot);

    //---------- Run both rotors forward, from end-point, for some time.
    xprintf((char *)"MES: Move from end-switch\n");
    run_to_pos(AX_rot, EY_rot,degr_forward,degr_forward,true); // some degrees forward

    // Do some checks: did rotors move?
    check_run(AX_rot);
    check_run(EY_rot);

    delay(1000);

    //---------- Run both rotors backward, until endswitch
    set_led(AX_rot,5,led_ena);               // set RB: to endswitch
    set_led(EY_rot,5,led_ena);               // set RB: to endswitch
    xprintf((char *)"MES: Move to end-switch\n");
    run_to_endswitch(AX_rot,EY_rot,speed[i]);

    set_led(AX_rot,6,led_ena);               // set RGB: start going to cal. pos
    set_led(EY_rot,6,led_ena);               // set RGB: start going to cal. pos
    delay(1000);
  }

  //---------- Run both rotors to reference
  xprintf((char *)"MES: Goto reference\n");
  err=run_to_cal_pos(AX_rot,EY_rot,0.,0.);
  set_led(AX_rot,(err&1? 4 : 2),led_ena);  // set R or G
  set_led(EY_rot,(err&2? 4 : 2),led_ena);  // set R or G
  if (AX_rot) AX_rot->calibrated=(err&1? false : true);
  if (EY_rot) EY_rot->calibrated=(err&2? false : true);

  if (err)
  {
    xprintf((char *)"MES: Motors don't run!\n");
    return 3;
  }
  else
  {
    xprintf((char *)"MES: Calibration ready!\n");
  }

  return 0;
}

#define START_CALFLAG "Start calibration"
int calibrate(ROTOR *AX_rot,ROTOR *EY_rot)
{
  int err=1;
  run_motor_hard(AX_rot,0);
  run_motor_hard(EY_rot,0);
  xprintf("%s\n",START_CALFLAG);
  #if CAL_ZENITH
    err=calibrate_zenith(AX_rot,EY_rot,SPD_CAL1,SPD_CAL2);
  #else
    err=calibrate_estop(AX_rot,EY_rot,SPD_CAL1,SPD_CAL2);
  #endif

  if (err)
  {
    run_motor_hard(AX_rot, 0); // stop motors (just in case, should already be stopped)
    run_motor_hard(EY_rot, 0);
    xprintf((char *)"Calibration error!\n");
    blink(20, 100);       // Note: causes pin LED_BUILTIN to pulse!
    return err;
  }
  xprintf((char *)"Calibration done\n");
  digitalWrite(LED_BUILTIN, HIGH);   // LED on; calibration done

  return err;
}
