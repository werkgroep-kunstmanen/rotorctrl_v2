/*******************************************************************
 * RCSId: $Id: rotorctrl.ino,v 1.4 2023/09/07 07:11:58 ralblas Exp $
 *
 * Project: rotordrive
 * Author: R. Alblas
 *
 * content:
 *   main program, with setup() and loop()
 *
 * functions:
 *   void setup(void)
 *   void loop(void)
 *
 * History: 
 *   
 * $Log: rotorctrl.ino,v $
 * Revision 1.4  2023/09/07 07:11:58  ralblas
 * _
 *
 * Revision 1.3  2023/09/02 09:58:15  ralblas
 * _
 *
 * Revision 1.2  2023/07/25 07:48:43  ralblas
 * _
 *
 * Revision 1.1  2023/07/18 12:11:49  ralblas
 * Initial revision
 *
 * Revision 1.1  2021/07/29 08:18:30  ralblas
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
#include "rotorctrl.h"

#ifndef ADD_OTA_UPLOAD
#define ADD_OTA_UPLOAD false
#endif

#if USE_WIFI
  #include <WiFi.h>
  WiFiServer Server(ServerPort);
  WiFiClient RemoteClient;
  #if ADD_OTA_UPLOAD
    #include <AsyncTCP.h>
    #include <AsyncElegantOTA.h>
    AsyncWebServer otaserver(80);
  #endif
#endif


#if USE_SGP4
  KEPLER kepler;
  EPOINT refpos;
#endif

#if USE_DISPLAY
  #include <LiquidCrystal_I2C.h>
  LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);
#endif


#if MOTORTYPE == MOT_STEPPER       // stepper motor
  #include <AccelStepper.h>        // http://www.airspayce.com/mikem/arduino/AccelStepper/index.html
  AccelStepper stepperAX(1, PIN_ROTPWM_AX, PIN_ROTDIR_AX);
  AccelStepper stepperEY(1, PIN_ROTPWM_EY, PIN_ROTDIR_EY);

  #define CMD(n,r) ((AccelStepper *)(n.stepper))->r
  #define CMDP(n,r) ((AccelStepper *)(n->stepper))->r
#endif


ROTOR gAX_rot,gEY_rot;
ROTOR *SAX_rot,*SEY_rot;
COMMANDS command;

boolean do_feedback;

// Pulse counter
static void pos_handler(ROTOR *rot)
{
  if (!rot) return;
  if (rot->dir)
  {
    rot->rotated++;
  }
  else
  {
    rot->rotated--;
  }
}

// Pulse counter rotor Azimut/X
static ISR_FUNC AX_pos_handler(void)
{
  pos_handler(&gAX_rot);
}

// Pulse counter rotor elevation/Y
static ISR_FUNC EY_pos_handler(void)
{
  pos_handler(&gEY_rot);
}

// setup rotor AX
static void setup_ax(ROTOR *rot)
{
  if (!rot) return;
  strcpy(rot->name, AX_NAME);
  rot->id = AX_ID;
  rot->round = 0;
  rot->rotated = 0;
  rot->cal_status = cal_notdone;
  AX_set_pins(rot);
  rot->steps_degr = AX_STEPS_DEGR;
  #if ((MOTORTYPE == MOT_DC_PWM) || (MOTORTYPE == MOT_DC_FIX))
    rot->minspeed = AX_MINSPEED;
    rot->maxspeed = AX_MAXSPEED;
  #endif
  #if MOTORTYPE == MOT_STEPPER
    rot->stepper = &stepperAX;
    CMDP(rot, setMaxSpeed(AX_MotorSpeed));     // Set the X rotor-motor maximum speed
    CMDP(rot, setAcceleration(AX_MotorAccel)); // Set the X rotor-motor acceleration speed
  #endif
}

// setup rotor EY
static void setup_ey(ROTOR *rot)
{
  if (!rot) return;
  strcpy(rot->name, EY_NAME);
  rot->id = EY_ID;
  rot->round = 0;
  rot->rotated = 0;
  rot->cal_status = cal_notdone;
  EY_set_pins(rot);
  rot->steps_degr = EY_STEPS_DEGR;
  #if ((MOTORTYPE == MOT_DC_PWM) || (MOTORTYPE == MOT_DC_FIX))
    rot->minspeed = EY_MINSPEED;
    rot->maxspeed = EY_MAXSPEED;
  #endif
  #if MOTORTYPE == MOT_STEPPER
    rot->stepper = &stepperEY;
    CMDP(rot, setMaxSpeed(EY_MotorSpeed));     // Set the X rotor-motor maximum speed
    CMDP(rot, setAcceleration(EY_MotorAccel)); // Set the Y rotor-motor acceleration speed
  #endif
}

// setup and calibrate
void setup(void)
{
  int err = 0;
  SAX_rot = NULL;
  SEY_rot = NULL;

  // define serial input, if USB connected: commands (if no wifi), debugging
  Serial.begin(SERIAL_SPEED);       // Start Serial Communication Interface

  #if ROTOR_AX
    SAX_rot = &gAX_rot;
  #endif
  #if ROTOR_EY
    SEY_rot = &gEY_rot;
  #endif

  setup_ax(SAX_rot);
  setup_ey(SEY_rot);

  #if USE_WIFI
    if (WiFi.status() == WL_CONNECTED)
    {
      blink(5,100);
      disconnect_wifi();
    }

    // set wifi, get time
    if ((PIN_SW1>=0) && (digitalRead(PIN_SW1)==0))
      connect_wifi_ap(my_SSID2, my_PASSWORD2);
    else
      connect_wifi(my_SSID1, my_PASSWORD1);
    Server.begin();

    #if USE_SGP4
      configTime((long)0,(int)0, NTPSERVER);
      get_ntp();

      // load defaults
      load_default_refpos(&refpos);
      load_default_kepler(&kepler); // just some defaults, to make keplerdata valid

      calc_sgp4_const(&kepler,true);
    #endif
  #endif

  #if MOTORTYPE == MOT_STEPPER
    #ifdef PIN_AXEYEnable
      if (PIN_AXEYEnable >= 0)
        digitalWrite(PIN_AXEYEnable, HIGH);             // Enable use of M415C controllers
    #endif
  #endif

  pinMode(LED_BUILTIN, OUTPUT);     // calibration indication
  do_feedback = true;

  // define interrupts for backpulsecounters
  #if PIN_ROTPLS_AX && PIN_ROTPLS_AX >=0
    attachInterrupt(digitalPinToInterrupt(PIN_ROTPLS_AX) , AX_pos_handler , RISING);
  #endif
  #if PIN_ROTPLS_EY && PIN_ROTPLS_EY >=0
    attachInterrupt(digitalPinToInterrupt(PIN_ROTPLS_EY) , EY_pos_handler , RISING);
  #endif

  #if USE_DISPLAY
    // define LCD display
    lcd.begin(20, 4);
    lcd.clear();
  #endif

  #if ADD_OTA_UPLOAD
    otaserver.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
      request->send(200, "text/plain", "Upload ESP32 firmware. Add '/update' to URL.");
    });
    AsyncElegantOTA.begin(&otaserver);    // Start AsyncElegantOTA
    otaserver.begin();
    Serial.println("HTTP server started");
  #endif

  digitalWrite(LED_BUILTIN, LOW);   // LED off; start calibration
  delay(1000);
  err = calibrate(SAX_rot, SEY_rot);
  if (SAX_rot) command.gotoval.ax = SAX_rot->degr;
  if (SEY_rot) command.gotoval.ey = SEY_rot->degr;
}


// endless loop: catch position from serial interface and run motors
void loop(void)
{
  if (Serial.available())
  {
    readCommand_serial();        // from USB, do command
  }
//  else
  {
  #if USE_WIFI
    readCommand_wifi();       // from Wifi, do command
  #endif
  }

  #if USE_SGP4
    if (command.run_calc)
    {
      static boolean pabove_hor;
      boolean above_hor;
      above_hor=calc_pos(&command.gotoval,&kepler,&refpos);
    #if CAL_AFTER_TRACK
      if ((!above_hor) && (pabove_hor))
      {
        calibrate(SAX_rot, SEY_rot);
      }
      pabove_hor=above_hor;
    #endif

      #ifdef CONTSENDINFO
      {
        static int pa,pe;
        // detect >=1 degrees step
        if ((int)(command.gotoval.a)!=pa) command.got_new_pos=true;
        if ((int)(command.gotoval.e)!=pe) command.got_new_pos=true;
        pa=(int)(command.gotoval.a);
        pe=(int)(command.gotoval.e);
      }
      #endif
    }
  #endif

  if (command.contrunning)
  { // especially needed for stepper motors, see spec 'AccelStepper'
    run_motor_hard(SAX_rot, command.a_spd);
    run_motor_hard(SEY_rot, command.b_spd);
  }
  else
  {
    rotor_goto(SAX_rot, command.gotoval.ax);
    rotor_goto(SEY_rot, command.gotoval.ey);

    #ifdef CONTSENDINFO
     #if USE_WIFI
      // if pos. is calculated in controller then send results back for monitoring 
      if (command.got_new_pos)
      {
        send_ctrldata(RemoteClient,SAX_rot, SEY_rot,&command.gotoval); // send pos. to PC for monitoring
        usleep(1000000);
      }
     #endif
    #endif
    command.got_new_pos = false;
  }
}
