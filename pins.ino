/*******************************************************************
 * RCSId: $Id: pins.ino,v 1.2 2023/09/06 19:19:40 ralblas Exp $
 *
 * Project: rotordrive
 * Author: R. Alblas
 *
 * content: 
 *   pin def. for motors
 *
 * public functions:
 *   AX_set_pins(ROTOR *rot)
 *   EY_set_pins(ROTOR *rot)
 *
 * History: 
 * $Log: pins.ino,v $
 * Revision 1.2  2023/09/06 19:19:40  ralblas
 * _
 *
 * Revision 1.1  2023/06/19 09:30:32  ralblas
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
#define set_pinmode(p,t) (p>=0? pinMode(p,t) : void())


// Define pins for rotor AX
// Pins defined as '< 0' are ignored.
void AX_set_pins(ROTOR *rot)
{
  if (!rot) return;

  rot->pin_pwm=PIN_ROTPWM_AX;
  rot->pin_zen=PIN_ROTZEN_AX;
  rot->pin_dir=PIN_ROTDIR_AX;
  rot->pin_din=PIN_ROTDIN_AX;
  rot->pin_lsp=PIN_LOWSPD_AX;

  rot->pin_end1=PIN_ENDSW1_AX;
  rot->pin_end2=PIN_ENDSW2_AX;

  set_pinmode(PIN_ROTPLS_AX, INPUT);        // dc rotor fb pulses
  set_pinmode(rot->pin_zen,  INPUT);        // zenith detect
  set_pinmode(rot->pin_pwm,  OUTPUT);       // rotor speed or step
  set_pinmode(rot->pin_dir,  OUTPUT);       // rotor direction
  set_pinmode(rot->pin_din,  OUTPUT);       // rotor direction inversed
  set_pinmode(rot->pin_lsp,  OUTPUT);       // dc rotor low speed

  set_pinmode(rot->pin_end1, INPUT_PULLUP); // step end switches
  set_pinmode(rot->pin_end2, INPUT_PULLUP); // step end switches

  set_pinmode(PIN_R_AX, OUTPUT);            // led indication
  set_pinmode(PIN_G_AX, OUTPUT);            // led indication
  set_pinmode(PIN_B_AX, OUTPUT);            // led indication

  set_pinmode(PIN_SW1, INPUT_PULLUP);       // switch 1
  set_pinmode(PIN_SW2, INPUT_PULLUP);       // switch 2

  #if MOTORTYPE == MOT_DC_PWM
    #if PROCESSOR == PROC_AVR
      // Setup For PWM
      InitTimersSafe();
      SetPinFrequencySafe(PIN_ROTPWM_AX,PWMFreq);
    #endif
    #if PROCESSOR == PROC_ESP
      ledcSetup(pin2chan(rot->pin_pwm),PWMFreq,MAX_PWM_BITS);
      ledcAttachPin(PIN_ROTPWM_AX, pin2chan(PIN_ROTPWM_AX)); 
    #endif
  #endif
}

// Define pins for rotor EY
// Pins defined as '< 0' are ignored.
void EY_set_pins(ROTOR *rot)
{
  if (!rot) return;

  rot->pin_pwm=PIN_ROTPWM_EY;
  rot->pin_zen=PIN_ROTZEN_EY;
  rot->pin_dir=PIN_ROTDIR_EY;
  rot->pin_din=PIN_ROTDIN_EY;
  rot->pin_lsp=PIN_LOWSPD_EY;

  rot->pin_end1=PIN_ENDSW1_EY;
  rot->pin_end2=PIN_ENDSW2_EY;

  set_pinmode(PIN_ROTPLS_EY, INPUT);        // dc rotor fb pulses
  set_pinmode(rot->pin_zen,  INPUT);        // zenith detect
  set_pinmode(rot->pin_pwm,  OUTPUT);       // rotor speed or step
  set_pinmode(rot->pin_dir,  OUTPUT);       // rotor direction
  set_pinmode(rot->pin_din,  OUTPUT);       // rotor direction inversed
  set_pinmode(rot->pin_lsp,  OUTPUT);       // dc rotor low speed

  set_pinmode(rot->pin_end1, INPUT_PULLUP); // step end switches
  set_pinmode(rot->pin_end2, INPUT_PULLUP); // step end switches

  set_pinmode(PIN_R_EY, OUTPUT);            // led indication
  set_pinmode(PIN_G_EY, OUTPUT);            // led indication
  set_pinmode(PIN_B_EY, OUTPUT);            // led indication

  #if MOTORTYPE == MOT_DC_PWM
    // Setup For PWM
    #if PROCESSOR == PROC_AVR
      InitTimersSafe();
      SetPinFrequencySafe(PIN_ROTPWM_EY,PWMFreq);
    #endif
    #if PROCESSOR == PROC_ESP
      ledcSetup(pin2chan(rot->pin_pwm),PWMFreq,MAX_PWM_BITS);
      ledcAttachPin(PIN_ROTPWM_EY, pin2chan(PIN_ROTPWM_EY)); 
    #endif
  #endif
}

