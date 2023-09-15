/*******************************************************************
 * RCSId: $Id: rotor_spec.h,v 1.5 2023/09/07 07:12:08 ralblas Exp $
 *
 * Project: rotordrive
 * Author: R. Alblas
 *
 * content: header:
 *   def. of:
 *     motor/rotor characteristics
 *     pins
 *     pwm frequency
 *
 * History: 
 * $Log: rotor_spec.h,v $
 * Revision 1.5  2023/09/07 07:12:08  ralblas
 * _
 *
 * Revision 1.4  2023/09/06 19:18:54  ralblas
 * _
 *
 * Revision 1.3  2023/07/25 08:08:57  ralblas
 * _
 *
 * Revision 1.2  2023/07/24 20:32:19  ralblas
 * _
 *
 * Revision 1.1  2021/07/26 12:34:39  ralblas
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
#ifndef ROTORSPEC_HDR
#define ROTORSPEC_HDR

// Included only by rotorctrl.h
#define RELEASE "2023.3"

#define USE_WIFI true
#if USE_WIFI
  // use SGP4-calc. in controller (needs wifi)
  #define USE_SGP4 true
  #define CAL_AFTER_TRACK true

  // use webserver
  #define ADD_OTA_UPLOAD true

#else
  #define USE_SGP4 false  // Don't change!
#endif

// Define processor
#define PROCESSOR PROC_ESP

// Set rotortype
#ifndef ROTORTYPE
#define ROTORTYPE ROTORTYPE_XY
#endif

#ifndef MOTORTYPE
#define MOTORTYPE MOT_DC_PWM 
#endif

// Enable rotors used
#define ROTOR_AX true
#define ROTOR_EY true

// serial connection
#define SERIAL_SPEED 115200


// Define motor speed settings
#if MOTORTYPE == MOT_DC_FIX        // low speed done in hardware
#define SPEED_LOW 20               // below this speed PIN_LOWSPD_.. becomes active 
#endif

// Next also needed for MOT_DC_FIX, though not all really used as such!
#if ((MOTORTYPE == MOT_DC_PWM) || (MOTORTYPE == MOT_DC_FIX))
  #define AX_MINSPEED 30           // min. speed (% of max. voltage)
  #define AX_MAXSPEED 100          // max. speed (% of max. voltage)
  #define EY_MINSPEED 30           // min. speed (% of max. voltage)
  #define EY_MAXSPEED 100          // max. speed (% of max. voltage)
  #define L_DEGR_MAXSPEED 10.      // >= diff-degrees where rotorspeed is max.
  #define H_DEGR_MINSPEED 2.       // <= diff-degrees where rotorspeed is min.
  #define D_DEGR_STOP 0.2          // <= diff-degrees to stop rotor
#endif

// Speeds
#if MOTORTYPE == MOT_STEPPER       // 
  #define AX_MotorSpeed 100        // max. motorspeed
  #define AX_MotorAccel 50         // max. acceleration
  #define EY_MotorSpeed 100        // max. motorspeed
  #define EY_MotorAccel 50         // max. acceleration
#endif

#define SWAP_DIR false             // flip directions


//==================== Calibration ====================
#define CAL_ZENITH true

// invert zenit-flip
#define AX_ZENPIN_INV false
#define EY_ZENPIN_INV false

// Rotor characteristics
// _POffset: steps needed to go from end-stop to 0 degrees. 
//   > 0: end-stop is at -x degrees
//   < 0: end-stop is at +x degrees
//

// steps from reference pos. !Currently only for endstop-calibration!
#define AX_POffset 0              // Nr. pulses from End Switch To ref. pos = 0
#define EY_POffset 0              // Nr. pulses from End Switch To ref. pos = 0

// _REFPOS:  position where calibration ends. (E.g. for elevation: 90 = zenit)
#if CAL_ZENITH
  #define AX_REFPOS 90.            // Reference position (degrees)
  #define EY_REFPOS 90.            // Reference position (degrees)
#else
  #define AX_REFPOS 0.             // Reference position (degrees)
  #define EY_REFPOS 0.             // Reference position (degrees)
#endif

#define AX_STEPS_DEGR 10L*360L      // Nr. pulses per 360 degrees
#define EY_STEPS_DEGR 10L*360L      // Nr. pulses per 360 degrees

// Calibration motor speed:
//   if one is 0, single calibration
//   Other: 2-step calibration, one fast, one slow for accurate calibration
#if CAL_ZENITH
  #define SPD_CAL1 100             // speed1: % of **_MotorSpeed, cal. to 90
  #define SPD_CAL2  20             // speed2: same, actual calibration
#else
  #define SPD_CAL1 100             // speed1: % of **_MotorSpeed, cal. to endswitch
  #define SPD_CAL2   0             // speed2: same, for second cal. (0: single-calibration)
#endif

#if ROTORTYPE==ROTORTYPE_AE
  #define USE_EASTWEST true        // use east/west pass info (set false if X/Y rotor)
  #define FULLRANGE_AZIM false     // azimut rotor has limited range 0...180

  #if FULLRANGE_AZIM && USE_EASTWEST
    #error Errored configuration: FULLRANGE_AZIM and USE_EASTWEST!
  #endif

  // Azimut rotor has range of ROT_AZIM_MIN...ROT_AZIM_MAX (via 0)
  // Area ROT_AZIM_MAX...ROT_AZIM_MIN for azimut is forbidden.
  // East pass: azimut= -20...  0...200                    , elev =   0...90
  // West pass: azimut= 160...360...380 ==> -20...180...200, elev = 180...90
  #define SET_AZIM_MIN 340         // min. degrees from tracker   (ROT_AZIM_MIN+360)
  #define SET_AZIM_MAX 200         // max. degrees from tracker

#endif

// timeout for calibration: 
//   ROT_TIMEOUT=max. time needed for 180 degrees
//   PLS_TIMEOUT=max. time between 2 pulses
#define ROT_TIMEOUT 30000
#define PLS_TIMEOUT 4000


#if ((MOTORTYPE == MOT_DC_PWM) || (MOTORTYPE == MOT_DC_FIX)) // DC motor
// PWM frequency and max. PWM (some controllers MUST have pulses, so not 100%!)
  #define PWMFreq 10000            // PWM frequency
  #define MAX_PWM_BITS 8
  #define MAX_PWM 255              // 255=DC 5V
#endif

#if USE_WIFI
  // for wifi
  #define my_SSID1 "your_ssid"
  #define my_PASSWORD1 "your_password"

  #define my_SSID2 "rotorctrl"
  #define my_PASSWORD2 "xxxxyyyy"

  #define ServerPort 23

  #if USE_SGP4
    #define NTPSERVER "pool.ntp.org"
    #define ROTOR_AX_STOP 90
    #define ROTOR_EY_STOP 90

    // rotor config
    #define XY_CONFIG X_AT_DISC
  #endif
#endif

//==================== Define pins ====================
// Needed interrupt inputs: (AVR)
// 2	PD2
// 3	PD3

// Allowed PWM outputs: (AVR)
// pin	name	timer
// 9	PB1	1A
// 10	PB2	1B
// 11	PB3	2A

// If not used: set pinnumber to negative
// NOTE: ESP: pin 2 = builtin-LED, AVR: pin 13=builtin-LED
// NOTE: Use ESP32Dev Module otherwise GPIO25 will behave strange with Wifi!
#define PIN_ROTZEN_AX  19          // : zenit-detect
#define PIN_ROTPLS_AX  18          // : input pulses (interrupt)
#define PIN_ROTDIR_AX   5          // : output direction
#define PIN_ROTPWM_AX  17          // : output speed (pwm)
#define PIN_ROTDIN_AX -100         // : inverted output direction
#define PIN_LOWSPD_AX -100         // : output speed (low/high)

#define PIN_ROTZEN_EY  32          // : zenit-detect
#define PIN_ROTPLS_EY  33          // : input pulses (interrupt)
#define PIN_ROTDIR_EY  25          // : output direction
#define PIN_ROTPWM_EY  26          // : output speed (pwm)
#define PIN_ROTDIN_EY -100         // : inverted output direction
#define PIN_LOWSPD_EY -100         // : output speed (low/high)

#define PIN_ENDSW1_AX   -3 // off  // AX Rotor end stops
#define PIN_ENDSW2_AX   -2 // off
#define PIN_ENDSW1_EY   -5 // off  // EY Rotor end stops
#define PIN_ENDSW2_EY   -4 // off

#define PIN_SW1        23          // switch access point or not
#define PIN_SW2        22          // optional switch, not used yet


//==================== Extra pins for LED indication ====================
#define PIN_R_AX  16               // : output red 1
#define PIN_G_AX   4               // : output green 1
#define PIN_B_AX  15               // : output blue 1

#define PIN_R_EY  27               // : output red 2
#define PIN_G_EY  14               // : output green 2
#define PIN_B_EY  12               // : output blue 2

// Names
#if ROTORTYPE==ROTORTYPE_XY        // X/Y
  #define AX_NAME "X"              // name of rotor 2
  #define EY_NAME "Y"              // name of rotor 1
#elif ROTORTYPE==ROTORTYPE_AE
  #define AX_NAME "azi"            // name of rotor 2
  #define EY_NAME "ele"            // name of rotor 1
#endif

/**************************************************
 * End definitions
 **************************************************/
#endif
