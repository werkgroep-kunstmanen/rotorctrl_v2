/*******************************************************************
 * RCSId: $Id: handle_commands.ino,v 1.2 2023/09/06 18:51:12 ralblas Exp $
 *
 * Project: rotordrive
 * Author: R. Alblas
 *
 * content: 
 *   handling of commands
 *
 * public functions:
 *   int parse_cmd(char *cmd)
 *      parses command in cmd and fills global 'command'
 *      returns 1 if command was processed
 *
 *   void execute_cmd()
 *      executes commands in global 'command'
 *
 * History: 
 *   
 * $Log: handle_commands.ino,v $
 * Revision 1.2  2023/09/06 18:51:12  ralblas
 * _
 *
 * Revision 1.1  2023/07/18 17:52:14  ralblas
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
#if USE_SGP4
  extern KEPLER kepler;
  extern EPOINT refpos;
#endif

extern COMMANDS command;

// Get arg. from command
//   cmd: command-string:
//     <cmd> <val(s)> or <cmd>=<val(s)>, <val(s)>=string without spaces
//   key: command
//   return: string with value or NULL (key doesn't match)
//
#define LENBUF 50
static char *get_val(char *cmd,char *key)
{
  char *p;
  static char cmdi[LENBUF];
  strncpy(cmdi,cmd,LENBUF);
  if ((p=strchr(cmdi,' '))) *p='=';    // for backward compatibility
  if ((p=strchr(cmdi,'=')))            // <key>=<val>
  {
    if (!strncmp(cmdi,key,strlen(key))) return p+1;
  }
  return NULL;
}

// Get boolean from command
//   cmd: command-string:
//     <cmd> <bool>, <bool>=0  or 1
//   key: command
//   val: found boolean
//   return: 1 or 0 (key doesn't match)
static int get_bool(char *cmd,char *key,boolean *val)
{
  char *p;
  if ((p=get_val(cmd,key)))
  {
    if (atoi(p)) *val=true; else *val=false;
    return 1;
  }
  return 0;
}

#if USE_SGP4
// Set if communicated keplers are in degrees or radians (communicated = what's exchanged between pc and rotorctrl)
#define kepler_in_degrees true // temp., MUST be true with version >=2023.3! 
void convert_kep(float ival,float *roval,float *doval)
{
  if (kepler_in_degrees)
  {
    *doval=ival;
    *roval=D2R(ival);
  }
  else
  {
    *roval=ival;
    *doval=R2D(ival);
  }
}

// get Kepler from interface
static int get_kepler_item(char *cmd,KEPLER *k)
{
  char *p;
  if      ((p=get_val(cmd,"satname=")))      strncpy(k->name,p,20);
  else if ((p=get_val(cmd,"epoch_year=")))   k->epoch_year=atoi(p);
  else if ((p=get_val(cmd,"epoch_day=")))    k->epoch_day=atof(p);
  else if ((p=get_val(cmd,"decay_rate=")))   k->decay_rate=atof(p);
  else if ((p=get_val(cmd,"bstar=")))        k->bstar=atof(p);
  else if ((p=get_val(cmd,"inclination=")))  convert_kep(atof(p),&k->inclination,&k->d_inclination);
  else if ((p=get_val(cmd,"raan=")))         convert_kep(atof(p),&k->raan,&k->d_raan);
  else if ((p=get_val(cmd,"eccentricity="))) k->eccentricity=atof(p);
  else if ((p=get_val(cmd,"perigee=")))      convert_kep(atof(p),&k->perigee,&k->d_perigee);
  else if ((p=get_val(cmd,"anomaly=")))      convert_kep(atof(p),&k->anomaly,&k->d_anomaly);
  else if ((p=get_val(cmd,"motion=")))       k->motion=atof(p);
  else  return 0;
//  calc_sgp4_const(k,false);
//  get_ntp();                 // get fresh time
  return 1;
}
#endif

// parsing commands from serial or wifi interface
// format commands: (no spaces!)
//    <command>=<val>\n
//    <command> <val>\n (for backwards compatibility, only place with space!)
//    <command>\n
// !!! command MUST end with '\n'!!!


int parse_cmd(char *cmd)
{
  char *p;
  if ((p=strchr(cmd,'\n'))) *p=0;
  if ((p=strchr(cmd,'\r'))) *p=0;
  if ((p=get_val(cmd,"a=")))
  {
    command.contrunning=true;
    command.cmd=contrun_ax;
    command.a_spd=atoi(p);
    return 1;
  }
  else if ((p=get_val(cmd,"b=")))
  {
    command.contrunning=true;
    command.cmd=contrun_ey;
    command.b_spd=atoi(p);
    return 1;
  }
  else if (strlen(cmd))    // no 'a', 'b', but another command
  {
    command.contrunning=false;
  }

  if (!strcmp(cmd,"version"))
  {
    command.cmd=send_version;
    return 1;
  }

  if (!strcmp(cmd,"gc"))
  {
    command.cmd=config;
    return 1;
  }
  if (!strcmp(cmd,"gs"))
  {
    command.cmd=status;
    return 1;
  }
  if ((p=get_val(cmd,"m=")))
  {
    command.cmd=monitor;
    return 1;
  }
  if (!strcmp(cmd,"cal"))
  {
    command.cmd=do_calibrate;
    return 1;
  }
  if ((p=get_val(cmd,"f=")))
  {
    command.cmd=pwm_freq;
    command.pwm_freq=atoi(p);
    return 1;
  }

  if (!strcmp(cmd,"setup"))
  {
    command.cmd=do_setup;
    return 1;
  }
  if (!strcmp(cmd,"restart"))
  {
    command.cmd=restart;
    return 1;
  }

  if (get_bool(cmd,"pri_cmd=",&command.pri_cmd))
  {
    return 1;
  }
  #if USE_SGP4
    if (get_bool(cmd,"run_calc=",&command.run_calc))
    {
      if (command.run_calc)
        get_ntp();                         // get fresh time
        calc_sgp4_const(&kepler,kepler_in_degrees);

      return 1;
    }

    if ((p=get_val(cmd,"download_time")))  // download time to PC
    {
      command.cmd=send_time;
      return 1;
    }

    if ((p=get_val(cmd,"upload_time=")))   // upload time from PC
    {
      command.cmd=get_time;
      strncpy(command.time,p,sizeof(command.time));
      return 1;
    }

    if (!strcmp(cmd,"download_refpos"))    // download to PC
    {
      command.cmd=send_refpos;
      return 1; 
    }

    if ((p=get_val(cmd,"upload_refpos="))) // upload from PC
    {
      command.cmd=get_refpos;
      command.ref_lat=atof(p);
      if (p=strchr(p,',')) command.ref_lon=atof(p+1);
      return 1;
    }

    if (!strcmp(cmd,"download_keplers"))   // download to PC
    {
      command.cmd=send_kep;
      return 1; 
    }

    if (!strcmp(cmd,"get_ctrldata"))
    {
      command.cmd=send_rotdata;
      return 1; 
    }

    if (!strcmp(cmd,"get_pos"))
    {
      command.cmd=send_satpos;
      return 1; 
    }

    if (get_kepler_item(cmd,&kepler))
    {
      return 1; // get one kepler-item
    }

  #endif

  // last to check!
  if ((isdigit(cmd[0])) && (strchr(cmd,','))) // command <val1>,<val2>[,<val3>]
  {
    char cmd2[30];
    strncpy(cmd2,cmd,30);
    sprintf(cmd,"gotopos=%s",cmd2);
  }

  // gotopos=<ew>,<lat>,<lon>
  if ((p=get_val(cmd,"gotopos=")))
  {
    char *p1;
    int nrval=1;
    for (p1=p; *p1; p1++) if (*p1==',') nrval++;
    if (nrval>0) command.cmd=do_gotoval;
    if (nrval==3)
    {
      command.gotoval.east_pass=atoi(p);
      if (!(p=strchr(p,','))) return 0;
      p++;
      nrval--;
    }
    if (nrval==2)
    {
      command.gotoval.east_pass=1;
      command.gotoval.ax=atof(p);
      if (!(p=strchr(p+1,','))) return 0;
      p++;
      command.gotoval.ey=atof(p);
      nrval-=2;
    }
    if (nrval==1)
    {
      command.gotoval.east_pass=1;
      command.gotoval.ax=atof(p);
      command.gotoval.ey=0.;
      nrval--;
    }
    return 1;
  }
  return 0;
}

// execute commands
void execute_cmd()
{
  #if PROCESSOR == PROC_ESP
    if (command.cmd==restart)    ESP.restart();
    if (command.cmd==do_setup)   setup();
  #else
    if (command.cmd==restart)    setup();
    if (command.cmd==do_setup)   setup();
  #endif
  if (command.cmd==contrun_ax)   run_motor_hard(SAX_rot, command.a_spd);
  if (command.cmd==contrun_ey)   run_motor_hard(SEY_rot, command.b_spd);
  if (command.cmd==send_version) xprintf("VERS: Release %s\n",RELEASE);

  if (command.cmd==config)       send_specs(SAX_rot, SEY_rot);
  if (command.cmd==status)       send_stat(SAX_rot, SEY_rot);
  if (command.cmd==do_calibrate)
  {
    command.a_spd=0;
    command.b_spd=0;
    calibrate(SAX_rot, SEY_rot);
    if (SAX_rot) command.gotoval.ax = SAX_rot->degr;
    if (SEY_rot) command.gotoval.ey = SEY_rot->degr;
  }
  if (command.cmd==monitor)   ; 

  #if ((MOTORTYPE == MOT_DC_PWM) || (MOTORTYPE == MOT_DC_FIX))
    if (command.cmd==pwm_freq)
    {
      #if PROCESSOR == PROC_AVR
        // Setup For PWM
        if (SAX_rot) SetPinFrequencySafe(SAX_rot->pin_pwm,command.pwm_freq);
        if (SEY_rot) SetPinFrequencySafe(SEY_rot->pin_pwm,command.pwm_freq);
      #endif
      #if PROCESSOR == PROC_ESP
        if (SAX_rot) ledcSetup(pin2chan(SAX_rot->pin_pwm),command.pwm_freq,MAX_PWM_BITS);
        if (SEY_rot) ledcSetup(pin2chan(SEY_rot->pin_pwm),command.pwm_freq,MAX_PWM_BITS);
      #endif

    }
  #endif
  if (command.cmd==do_gotoval)
  {
    command.contrunning=false;
    command.got_new_pos=true;
  }

  #if USE_WIFI
    if (command.cmd==get_time)
    {
      set_time(command.time);
    }

    if (command.cmd==send_time)
    {
      send_ctrltime(RemoteClient);
    }
  #endif

  #if USE_SGP4
    if (command.cmd==send_rotdata)
    {
      send_ctrldata(RemoteClient,SAX_rot,SEY_rot,&command.gotoval);
    }
    if (command.cmd==send_satpos)
    {
      send_pos(RemoteClient,&command.gotoval,kepler.name);
    }



    if (command.cmd==send_refpos)
    {
      send_refposition(RemoteClient,&refpos);
    }

    if (command.cmd==get_refpos)
    {
      refpos.lat=D2R(command.ref_lat);
      refpos.lon=D2R(command.ref_lon);
    }
    if (command.cmd==send_kep)
    {
      send_keplers(RemoteClient,&kepler,kepler_in_degrees);
    }
  #endif
//  if (command.cmd!=none) printf("Command: %d\n",command.cmd);
  command.cmd=none;
}

