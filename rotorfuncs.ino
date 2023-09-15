/*******************************************************************
 * RCSId: $Id: rotorfuncs.ino,v 1.3 2023/08/03 08:52:29 ralblas Exp $
 *
 * Project: rotordrive
 * Author: R. Alblas
 *
 * content: 
 *   several motor/rotor functions
 *
 * public functions:
 *   float to_degr(ROTOR *rot)
 *   long from_degr(ROTOR *rot)
 *   void convert_eastwest(GOTO_VAL *gv)
 *   run_motor_soft(ROTOR *rot,int speed)
 *   int run_motor_hard(ROTOR *rot,int speed)
 *   int rotor_goto(ROTOR *rot,float val)
 *   void reset_to_pos(ROTOR *rot,long pos)
 *   void run_to_pos(ROTOR *AX_rot, ROTOR *EY_rot,float ax_pos,float ey_pos,boolean relative)
 *   void run_to_endswitch(ROTOR *AX_rot, ROTOR *EY_rot,int speed)
 *
 * History: 
 * $Log: rotorfuncs.ino,v $
 * Revision 1.3  2023/08/03 08:52:29  ralblas
 * _
 *
 * Revision 1.2  2023/07/21 19:12:47  ralblas
 * _
 *
 * Revision 1.1  2023/07/18 12:11:38  ralblas
 * Initial revision
 *
 * Revision 1.1  2021/07/29 08:25:38  ralblas
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
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "rotorctrl.h"

// calc. degrees from step for 'rot'
static float step2degr(ROTOR *rot,long step)
{
  float degr;
  if (!rot) return 0.;
  degr=(float)step*360./(float)rot->steps_degr;
  return degr;
}

// position rotor in degrees
float to_degr(ROTOR *rot)
{
  if (!rot) return 0.;
  return step2degr(rot,rot->rotated);
}

// calc. steps from degrees for 'rot'
static long degr2step(ROTOR *rot,float degr)
{
  long step;
  if (!rot) return 0.;

  step=(long)((degr*rot->steps_degr)/360);
  #if SWAP_DIR
  step*=-1;
  #endif
  return step;
}

// position rotor in steps, calculated from degrees of rot
long from_degr(ROTOR *rot)
{
  if (!rot) return 0;
  return degr2step(rot,rot->degr);
}

#if defined(USE_EASTWEST) && USE_EASTWEST
/*********************************************************************
 * For elevation/azimut, where azimut rotor cannot rotate 360 (400) degrees:
 *   use elevation rotor having 0...180 range; 
 *   determine flip-of-elevation rotor
 *
 * Input azimut: 0=north, 90=east, 180=south, 270=west
 * Input elevation 0...90
 * east pass:   0...180, azimut ranges between 340 (N-bound) and 200 (S-bound)
 * west pass: 180...360, azimut ranges between 160 (N-bound) and  20 (S-bound)
 * 
 *********************************************************************/
void convert_eastwest(GOTO_VAL *gv)
{
  // force to range 0...360
  if (!gv->east_pass)
  {                           // it's a west-pass
    gv->ax=gv->ax-180.;       // and substract 180 of 'goto' azimuth
    gv->ey=180.-gv->ey;       // so flip 'goto' elevation
  }
  if (gv->ax > 360.) gv->ax-=360.;
  if (gv->ax <   0.) gv->ax+=360.;

  // Check if azimut rotor is out-of-range, shouldn't happen with correct east/west pass info
  if ((gv->ax > SET_AZIM_MAX) && (gv->ax < SET_AZIM_MIN))
  {                            // correct
    gv->ax=gv->ax-180.;        // azimuth
    gv->ey=180.-gv->ey;        // elevation
  }


  // force to range 0...360
  if (gv->ax > 360.) gv->ax-=360.;
  if (gv->ax <   0.) gv->ax+=360.;
}
#endif

#if MOTORTYPE == MOT_STEPPER
static void moveto(ROTOR *rot)
{
  long step;
  if (!rot) return;

  step=degr2step(rot,rot->req_degr);
  CMDP(rot,moveTo(step)); // do requested
}

static boolean end_of_rot(ROTOR *rot,int speed)
{
  int end1=0,end2=0;
  boolean at_end;
  if (!rot) return at_end;
  end1=digitalRead(rot->pin_end1);
  end2=digitalRead(rot->pin_end2);
  at_end=(((end1) && (speed>0)) || ((end2) && (speed<0)));
  return at_end;
}

#else

/*********************************************************************
 * define speed from degrees between current and target position
 * For DC motors.
 * input: deg = difference current and requested angle in degrees
 * return: speed in percents (100=max. speed)
 *********************************************************************/
static int rotor_speed(ROTOR *rot,float deg)
{
  int speed;
  float adg=fabs(deg);
  if (!rot) return 0;

  if (adg<=D_DEGR_STOP)
  {
    speed=0;
  }
  else
  {
    #if MOTORTYPE == MOT_DC_PWM
    {
      float tmp;
      tmp=(float)(adg-H_DEGR_MINSPEED)*(rot->maxspeed-rot->minspeed)/(float)(L_DEGR_MAXSPEED-H_DEGR_MINSPEED);
      tmp=tmp + rot->minspeed;
      speed=(int)tmp;
      speed=MIN(speed,rot->maxspeed);
      speed=MAX(speed,rot->minspeed);
    }
    #else
    { // MOTORTYPE == MOT_DC_FIX
      speed=(int)((float)(adg-H_DEGR_MINSPEED)*100/(float)(L_DEGR_MAXSPEED-H_DEGR_MINSPEED));
    }
    #endif

    if (deg<0) speed*=-1;
    #if SWAP_DIR
      speed*=-1;
    #endif
  }

  return speed;
}

// accelerate motor
// ospeed follows ispeed in small steps
// uaccel: accleration up, daccel: acceleration down; 0=no acceleration
//   accel=steps per 10 ms; accel=1: 1 sec from speed=0 to speed=100 or back
static int accellerate(ROTOR *rot,int ispeed, int uaccel,int daccel)
{
  int ospeed=ispeed;
  int accel=0;
  static long t[2];
  int rid=(rot->id==AX_ID? 0 : 1);

  if (!rot) return ospeed;
  
  // Next is  to prevent return 0 as 'keep current speed' speed
  if ((ispeed) && (!rot->speed))
  {
    rot->speed=rot->minspeed*SIGN(ispeed); // Set speed to min.
  }

  // accelerate one time per 10 ms
  if (millis()-t[rid] < 10) return rot->speed; // keep current speed
  t[rid]=millis();

  if (abs(ispeed) > abs(rot->speed)) accel=uaccel; // accel. faster
  if (abs(ispeed) < abs(rot->speed)) accel=daccel; // accel. slower

  if ((accel) && (ispeed != rot->speed)) // requested speed != actual speed
  {
    if (ispeed < rot->speed) accel*=-1;
    ospeed=rot->speed+accel; // adapt ospeed
    if (abs(ospeed) > rot->maxspeed)
    {
      ospeed=rot->maxspeed*SIGN(ospeed);
    }

    // adapt ospeed if it is < minspeed
    if (abs(ospeed) < rot->minspeed)
    {
      if (ispeed)
      {
        if ((ospeed) && (SIGN(ispeed)!=SIGN(ospeed)))
        {
          ospeed=0;
        }
        else
        {
          ospeed=rot->minspeed*SIGN(ospeed);  // set to min. speed, still running
        }
      }
      else
      {
        ospeed=0;                           // stop
      }
    }
  }
  return ospeed;
}

// Set DC-motor speed; 'speed'=abs. value!
static void set_speed(ROTOR *rot,int speed)
{
  if (!rot) return;
  if (speed>100) speed=100;
  #if MOTORTYPE == MOT_DC_PWM
  {
    rot->pwm=(MAX_PWM*speed)/100;
    if (rot->pin_pwm>=0) pwmWrite(rot->pin_pwm,rot->pwm);
  }
  #elif MOTORTYPE==MOT_DC_FIX
  {
    rot->pwm=MAX_PWM;
    if (rot->pin_lsp>=0)
    {
      if (speed < SPEED_LOW)
      {
        digitalWrite(rot->pin_lsp, HIGH);
      }
      else
      {
        digitalWrite(rot->pin_lsp, LOW);
      }
    }
    if (rot->pin_pwm>=0) pwmWrite(rot->pin_pwm,rot->pwm);
  }
  #endif
}

// Set direction of motor
static void set_dir(ROTOR *rot,boolean dir)
{
  boolean din;
  if (!rot) return;
  if (dir!=rot->dir)
  {
    set_speed(rot,0);
    delay(10);
  }
  rot->dir=dir;

  digitalWrite(rot->pin_dir, rot->dir);
  if (rot->pin_din>=0)
  {
    digitalWrite(rot->pin_din, (rot->dir? LOW : HIGH));
  }
}

#endif


#if MOTORTYPE == MOT_STEPPER
static int set_temp_maxspeed(ROTOR *rot,int speed)
{
  int maxspeed,new_speed;
  maxspeed=CMDP(rot,maxSpeed());
  new_speed=maxspeed*(float)speed/100.;
  CMDP(rot,setMaxSpeed(new_speed));              // set speed to new according to 'speed'
  return maxspeed;
}
#endif

#define RUN_ENDSW_MAX -365.
// soft speed control (accel.)
// return: stepper: actual speed
//         DC: acc. speed from input speed
//         stepper: only acc. up, change maxspeed gives no nice accel!
int run_motor_soft(ROTOR *rot,int speed)
{
  if (!rot) return 0;

  // set speed and run (accelleration)
  #if MOTORTYPE == MOT_STEPPER
    if ((!speed) || (end_of_rot(rot,speed)))
    {
      CMDP(rot,setSpeed(0));            // force internally saved speed to 0
      CMDP(rot,stop());                 // force stop
      speed=0;                          //
    }
    else
    {
      int step;
      int maxspeed=set_temp_maxspeed(rot, speed);                  // set temp. maxspeed
      // set step 360 degrees from current pos. to accelerate
      step=CMDP(rot,currentPosition()+(rot->steps_degr*SIGN(speed))); 
      CMDP(rot,moveTo(step));                    // move accelerated to endpoint
      CMDP(rot,run());                           // run with accel. 
      CMDP(rot,setMaxSpeed(maxspeed));           // restore maxspeed
    }
    rot->rotated=CMDP(rot,currentPosition());
  #else
    speed=accellerate(rot,speed,1,1);
    if (speed)
      set_dir(rot,speed > 0? HIGH : LOW);
    set_speed(rot,abs(speed));
  #endif
  rot->speed=speed;
  return speed;
}

// Set speed of motor directly
int run_motor_hard(ROTOR *rot,int speed)
{
  if (!rot) return 0;

  // set speed and run (without accelleration)
  #if MOTORTYPE == MOT_STEPPER
    if (end_of_rot(rot,speed))
    {
      CMDP(rot,setSpeed(0));          // force internally saved speed to 0
      CMDP(rot,stop());               // force stop
      speed=0;                      //
    }
    else
    {
      float nspeed;
      nspeed=CMDP(rot,maxSpeed())*(float)speed/100.;
      CMDP(rot,setSpeed(nspeed));
      CMDP(rot,runSpeed());
    }
    rot->rotated=CMDP(rot,currentPosition());
  #else
    if (speed)
      set_dir(rot,speed > 0? HIGH : LOW);
    set_speed(rot,abs(speed));
  #endif
  rot->speed=speed;
  return speed;
}


// soft speed control (accel.)
// For DC motors accel. is defined by:
//   stop: #degr. to run: diff_degr
//   start: accellerate()
// For stepper: this is built-in
// return: stepper: speed detection; dc: actual speed
// end-stop for steppermotor is switch, so act directly.
// end-stop for DCmotor/pulsgiver needs some time to detect, do outside.
static int run_motor_soft_dd(ROTOR *rot)
{
  float diff_degr;
  int speed;
  if (!rot) return 0;

  diff_degr=rot->req_degr - rot->degr;
  rot->err_degr=diff_degr;


  #if MOTORTYPE == MOT_STEPPER
    float aspeed;
    aspeed=CMDP(rot,speed());                               // actual speed
    aspeed=(aspeed*100.) / CMDP(rot,maxSpeed());            // speed in %
    speed=(int)aspeed;                                      // now to integer

    if ((aspeed) && (!speed)) speed=(aspeed<0? -1 : +1);    // set to 1% if 0<s<1
    if (speed>100) speed=100;
    if (speed<-100) speed=-100;
    rot->speed=speed;                                       // calculated speed

    // end-stop for steppermotor is switch, so act directly.
    if (end_of_rot(rot,speed))                         // end-stop detected!
    {
      CMDP(rot,setSpeed(0));                           // force internally saved speed to 0
      CMDP(rot,stop());                                // force stop
      speed=0;
    }
    else
    {
      moveto(rot);
      CMDP(rot,run());                                 // run with accel. 
      speed=CMDP(rot,distanceToGo());                // to detect if ready, not actual speed...
    }
    rot->rotated=CMDP(rot,currentPosition());
  #else
    speed=rotor_speed(rot,diff_degr);
    speed=accellerate(rot,speed,1,0); // werkt veel te traag, grote overshoot!
    if (speed)
      set_dir(rot,speed > 0? HIGH : LOW);
    set_speed(rot,abs(speed));
    rot->speed=speed;
  #endif

  return speed; // DC: actual speed set (no endstop detection!), stepper: only speed detection
}


/*********************************************************************
 * Rotor to angle 'val'.
 * Must be used in loop; each time this func. is executed 
 *   speed is determined and used.
 * return: current speed; 0=stop=rotator is at requested position.
 *********************************************************************/
int rotor_goto(ROTOR *rot,float val)
{
  float rot_degr;      // current pos. rotor in decdegrees
  float req_degr;      // requested pos. rotor in decdegrees
  float diff_degr;
  int speed;
  if (!rot) return 0;

  rot->req_degr=val;                      // requested degrees
  rot->degr=to_degr(rot);
//printf("req=%f  act=%f\n",rot->req_degr,rot->degr);
  #if ROTORTYPE==ROTORTYPE_AE
    #if FULLRANGE_AZIM == false     // range azimut=0...+180
      if (rot->req_degr>270) rot->req_degr-=360;
    #else                           // range azimut=0...360
      if ((rot->rot_degr > 270) && (rot->req_degr+rot->round*360 < 90)) rot->round++;
      if ((rot->req_degr > 270) && (rot->rot_degr+rot->round*360 < 90)) rot->round--;
      rot->req_degr+=rot->round*360;
    #endif
  #endif
  speed=run_motor_soft_dd(rot);

  return speed;
}


/*********************************************************************
 * calibration funcs
 *********************************************************************/
static void set_status(ROTOR *rot,CAL_STATUS status)
{
  if (!rot) return;
  if (status==rot->cal_status+1) rot->cal_status=status; // status must increment
  rot->cal_status=status; // set status directly (change?)
}

// reset rotor to pos=steps
void reset_to_pos(ROTOR *rot,long step)
{
  if (!rot) return;
  #if SWAP_DIR
  step*=-1;
  #endif
  rot->rotated=step;
  #if MOTORTYPE == MOT_STEPPER
    CMDP(rot,setCurrentPosition(step));
  #endif
}

#if MOTORTYPE != MOT_STEPPER
static boolean is_moving(ROTOR *rot,unsigned long *start_time)
{
  boolean running=true;
  if (!rot) return false;
  if (rot->rotated!=rot->pre_rotated)
  {
    *start_time=millis();
  }
  else if (millis()-(*start_time) > PLS_TIMEOUT)
  {
    running=false;
  }
  rot->pre_rotated=rot->rotated;
  return running;
}
#endif

// blocks until pos. reached, or no pulses reached for some time (=error)
// AX_rot=NULL: only use EY_rot; EY_rot=NULL: only use AX_rot
void run_to_pos(ROTOR *AX_rot, ROTOR *EY_rot,float ax_pos,float ey_pos,boolean relative)
{
  int xbusy=0;
  int ybusy=0;
  long ax_run_err,ey_run_err;
  float ax_err,ey_err;
  unsigned long ax_start_time,ey_start_time;
  ax_start_time=millis();
  ey_start_time=millis();

  ax_run_err=0; ey_run_err=0;
  ax_err=0; ey_err=0;

  if (relative)
  {
    ax_pos+=to_degr(AX_rot);
    ey_pos+=to_degr(EY_rot);
  }

  do
  {
    xbusy=rotor_goto(AX_rot,ax_pos);
    ybusy=rotor_goto(EY_rot,ey_pos);
    #if MOTORTYPE != MOT_STEPPER
      if ((!is_moving(AX_rot,&ax_start_time)) && (!is_moving(EY_rot,&ey_start_time))) break;
    #endif
  } while ((xbusy) || (ybusy));

  run_motor_hard(AX_rot,0);
  run_motor_hard(EY_rot,0);

  if (!xbusy) set_status(AX_rot,cal_ready);
  if (!ybusy) set_status(EY_rot,cal_ready);
}

#define RUN_ENDSW_MAX -365.
static int prepare_speed(ROTOR *rot,int speed)
{
  int new_speed,maxspeed;
  if (!rot) return 0;
  #if MOTORTYPE == MOT_STEPPER
    maxspeed=CMDP(rot,maxSpeed());
    new_speed=maxspeed*(float)speed/100.;
    CMDP(rot,setMaxSpeed(new_speed));              // set speed to new according to 'speed'

    CMDP(rot,setCurrentPosition(0));               // reset current position
    rot->req_degr=RUN_ENDSW_MAX;                   // max. angle to rotate before give-up
    rot->at_end1=false;
    rot->at_end2=false;
    moveto(rot);
  #else
    maxspeed=rot->maxspeed;
    new_speed=fabs(maxspeed*(float)speed/100.);
    rot->maxspeed=new_speed;
    rot->rotated=0;
    rot->req_degr=RUN_ENDSW_MAX;                   // max. angle to rotate before give-up
  #endif
  return maxspeed;
}

static void restore_speed(ROTOR *rot,int speed,int pre_cnt)
{
  if (!rot) return;
  #if MOTORTYPE == MOT_STEPPER
    CMDP(rot,setMaxSpeed(speed));
    if (abs(CMDP(rot,currentPosition())) >= abs(degr2step(rot,RUN_ENDSW_MAX))-2) 
      rot->cal_status=cal_got_pulses;
    else
      rot->cal_status=cal_end_stop;
//      set_status(EY_rot,cal_end_stop);
  #else
    if (rot->rotated) rot->cal_status=cal_got_pulses;
    if (pre_cnt==rot->rotated); rot->cal_status=cal_end_stop;

//    if (pre_cnt==rot->rotated) set_status(rot,cal_end_stop);
  #endif
}

#define NWRUNEND
void run_to_endswitch(ROTOR *AX_rot, ROTOR *EY_rot,int speed)
{
  int busy=0;
  int ax_maxspeed,ey_maxspeed;
  int a,b;

  unsigned long ax_start_time,ey_start_time;

#ifdef NWRUNEND
#else
  ax_maxspeed=prepare_speed(AX_rot,speed);
  ey_maxspeed=prepare_speed(EY_rot,speed);
#endif

  ax_start_time=millis();
  ey_start_time=millis();

#if MOTORTYPE == MOT_STEPPER
  do
  {
#ifdef NWRUNEND
    busy=run_motor_soft(AX_rot,speed);
    busy|=run_motor_soft(EY_rot,speed);
#else
    busy=run_motor_soft_dd(AX_rot);
    busy|=run_motor_soft_dd(EY_rot);
#endif
  } while (busy);
#else
  do
  {
#ifdef NWRUNEND
    busy=run_motor_soft(AX_rot,speed);
    busy|=run_motor_soft(EY_rot,speed); 
#else
    busy=run_motor_soft_dd(AX_rot);
    busy|=run_motor_soft_dd(EY_rot); 
#endif
    a=is_moving(AX_rot,&ax_start_time); 
    b=is_moving(EY_rot,&ey_start_time);
  } while ((a) || (b));

#endif
  // both rotors at their endswitch, stop
  run_motor_hard(AX_rot,0);
  run_motor_hard(EY_rot,0);

#ifdef NWRUNEND
  if (AX_rot) AX_rot->cal_status=cal_end_stop;
  if (EY_rot) EY_rot->cal_status=cal_end_stop;
#else
  restore_speed(AX_rot,ax_maxspeed,0);//,AX_pre_cnt);
  restore_speed(EY_rot,ey_maxspeed,0);//,EY_pre_cnt);
#endif
  if (AX_rot) set_status(AX_rot,AX_rot->cal_status);
  if (EY_rot) set_status(EY_rot,EY_rot->cal_status);
}
