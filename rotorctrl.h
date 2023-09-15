/*******************************************************************
 * RCSId: $Id: rotorctrl.h,v 1.2 2023/09/03 18:39:05 ralblas Exp $
 *
 * Project: rotordrive
 * Author: R. Alblas
 *
 * content: header:
 *   def. of:
 *     structs etc.
 *
 * History: 
 * $Log: rotorctrl.h,v $
 * Revision 1.2  2023/09/03 18:39:05  ralblas
 * _
 *
 * Revision 1.1  2023/07/17 19:25:28  ralblas
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


#ifndef ROTORCTRL_HDR
#define ROTORCTRL_HDR

// processor, default: AVR/ATmega
#define PROC_AVR 0
#define PROC_ESP 1

/*******************************************************
 * Define some extra features
 *******************************************************/
// monitor via serial UART interface
// !! Not capturing TX shouldn't be a problem, taken care of, 
//    but sometimes that doesn't work and everything slows down! 
#ifndef USE_MON_UART
#define USE_MON_UART true
#endif

// LCD display
#ifndef USE_DISPLAY
#define USE_DISPLAY false
#endif

#if USE_DISPLAY
  #define LCD_NRCOL 20
  #define LCD_NRROW 4
#endif

// id's for rotors
#define AX_ID 2                  // id of rotor 2
#define EY_ID 1                  // id of rotor 1

// for SGP4: disc config.
#define X_AT_DISC 0
#define Y_AT_DISC 1

/*******************************************************
 * Def. of functions and structs. For all configurations.
 *******************************************************/
#ifndef MIN
#define MIN(a,b) (a)<(b)? (a) : (b)
#endif
#ifndef MAX
#define MAX(a,b) (a)>(b)? (a) : (b)
#endif

typedef bool boolean;

typedef enum 
{
  cal_notdone=0,
  cal_started=1,
  cal_got_pulses=2,
  cal_end_stop=3,
  cal_timeout=5,
  cal_ready=4
} CAL_STATUS;

#define MOT_DC_FIX 1
#define MOT_DC_PWM 2
#define MOT_STEPPER 3

#define ROTORTYPE_XY 1
#define ROTORTYPE_AE 2

typedef struct goto_val
{
  float ax;                      // goto value azimut or Y
  float ey;                      // goto value elevation or X
  float x,y;
  float a,e;
  float lon,lat;
  float height;
  boolean east_pass;             // true if sat. passes east
  boolean eastwest_pass_info;    // true if east_pass is valid
} GOTO_VAL;


typedef struct rotor
{
  char name[10];
  int id;
  long rotated;          // rotation done in some integer form (pulses, steps...)
  long pre_rotated;      // previous rotated (for run-check)
  long cnt        ;      // counter for run-check
  float req_degr;        // requested position rotor in degrees
  float degr;            // position rotor in degrees
  float p_degr;          // previous position rotor in degrees
  float err_degr;        // error
  long steps_degr;       // # steps (pulses) for 360 degrees rotation
  int speed;             // current speed
  int minspeed;          // minimum speed
  int maxspeed;          // maximum speed
  int motorspeed;        // max. motorspeed for stepper motor
  float d2v_slope;       // slope speed/degr
  int round;             // full 360 degrees
  boolean dir;           // direction
  boolean calibrated;    // calibration done and successfull
  CAL_STATUS cal_status; // status calibration
  int pwm;               // pwm: 0 (=stop)...100 (=max. speed) (percentage)
  boolean at_end1;       // end-stop activated
  boolean at_end2;       // end-stop activated
  int pin_pwm;           // pin nr. for PWM=speed
  int pin_dir;           // pin nr. for rotation direction
  int pin_din;           // pin nr. for inverted rotation direction
  int pin_zen;           // pin nr. for zenith detection
  int pin_lsp;           // pin nr. for low speed indication (if no PWM used)
  int pin_end1;          // pin nr. for end indication
  int pin_end2;          // pin nr. for end indication
  void *stepper;         // stepper class, for stepping motor
  boolean x_west_is_0;
  boolean y_south_is_0;
} ROTOR;

typedef enum CURRENT_COMMAND
{
  none=0,
  contrun_ax,contrun_ey,
  config,status,
  send_version,
  do_calibrate,
  monitor,
  pwm_freq,
  do_gotoval,
  send_satpos,
  send_rotdata,
  send_kep,
  get_refpos,
  send_refpos,
  get_time,
  send_time,
  do_setup,
  restart
};

typedef struct commands
{
  CURRENT_COMMAND cmd;
  boolean contrunning;
  int a_spd,b_spd;
  int pwm_freq;
  GOTO_VAL gotoval;
  boolean set_refpos;
  float ref_lat,ref_lon;
  boolean pri_cmd;
  boolean got_new_pos;
  char time[20];

  boolean send_kep;
  boolean get_pos;
  boolean get_ctrldata;
  boolean run_calc;
} COMMANDS;

#include "rotor_spec.h"

#define SIGN(a) ((a)<0? -1 : (a)>0? 1 : 0)

#if USE_SGP4
#include "rotorctrl_sgp4.h"
#else
#endif

#if PROCESSOR==PROC_ESP
  #define LED_BUILTIN 2
  // translate pwm-pin to channel connected to that pin
  #define pin2chan(a) (a==PIN_ROTPWM_AX? 0 : 1)
  #define pwmWrite(a,b) ledcWrite(pin2chan(a),b) 
  #define ISR_FUNC void IRAM_ATTR
#else
  #define ISR_FUNC void 
#endif

#if ((MOTORTYPE == MOT_DC_PWM) || (MOTORTYPE == MOT_DC_FIX)) // DC motor
  #if PROCESSOR==PROC_AVR
    #include <PWM.h>
  #endif
#endif

#if USE_SGP4
#include "keplerfuncs.h"
#endif

#endif
