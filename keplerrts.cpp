#include "rotorctrl.h"
#include "rotorctrl_sgp4.h"
#include "keplerfuncs.h"
#include <time.h>
#include <string.h>
#include <math.h>

#ifndef ROTOR_AX_STOP
  #define ROTOR_AX_STOP 90
#endif
#ifndef ROTOR_EY_STOP
  #define ROTOR_EY_STOP 90
#endif

// set default sat keplers (NOAA19)
void load_default_kepler(KEPLER *kepler)
{
  strcpy(kepler->name,"N19_fixed");
  kepler->epoch_year=123;
  kepler->epoch_day=100.8288727;
  kepler->decay_rate=0.0000027;
  kepler->bstar=0.000169;
  kepler->d_inclination=99.1081009;
  kepler->d_raan=145.656;
  kepler->eccentricity=0.001418;
  kepler->d_perigee=74.42179;
  kepler->d_anomaly=285.8587;
  kepler->motion=14.1272907;
}

// set default refpos
void load_default_refpos(EPOINT *refpos)
{
  refpos->lon=D2R(POSLON);
  refpos->lat=D2R(POSLAT);
}

/*************************************
 * Correct tm-struct and return #seconds since 1970.
 * Similar to mktime, except that always UTC is used.
 * Corrects tm_sec...tm_mon and tm_mday (possibly adapting tm_year)
 * Calcs tm_yday
 * Ignores tm_wday 
 * Sets tm_isdst to 0
 * Calcs # secs since 1970
 *************************************/
long mktime_ntz(struct tm *tm)
{
  long secs;
  int i;
  int days[]={31,28,31,30,31,30,31,31,30,31,30,31};

  while (tm->tm_sec >= 60) { tm->tm_min++; tm->tm_sec-=60; }
  while (tm->tm_sec <   0) { tm->tm_min--; tm->tm_sec+=60; }
  while (tm->tm_min >= 60) { tm->tm_hour++; tm->tm_min-=60; }
  while (tm->tm_min <   0) { tm->tm_hour--; tm->tm_min+=60; }
  while (tm->tm_hour>= 24) { tm->tm_mday++; tm->tm_hour-=24; }
  while (tm->tm_hour<   0) { tm->tm_mday--; tm->tm_hour+=24; }
  while (tm->tm_mon >= 12) { tm->tm_year++; tm->tm_mon-=12; }
  while (tm->tm_mon <   0) { tm->tm_year--; tm->tm_mon+=12; }
  if (tm->tm_year/4==tm->tm_year/4.) days[1]=29;

  while (tm->tm_mday>days[tm->tm_mon])
  {
    tm->tm_mday-=days[tm->tm_mon]; tm->tm_mon++;
    while (tm->tm_mon>= 12)  { tm->tm_year++; tm->tm_mon-=12; }
    if (tm->tm_year/4==tm->tm_year/4.) days[1]=29; else days[1]=28;
  }

  while (tm->tm_mday<=0)
  {
    tm->tm_mon--;
    tm->tm_mday+=days[tm->tm_mon];
    while (tm->tm_mon< 0)  { tm->tm_year--; tm->tm_mon+=12; }
    if (tm->tm_year/4==tm->tm_year/4.) days[1]=29; else days[1]=28;
  }

  secs=tm->tm_sec+60*(tm->tm_min+60*(tm->tm_hour+24*(tm->tm_mday-1)));
  for (i=0; i<tm->tm_mon; i++) secs+=(days[i]*24*3600);
  secs+=((tm->tm_year-70)*365*24*3600); 
  secs+=((tm->tm_year-70+1)/4)*24*3600; 
  tm->tm_yday=0;
  for (i=0; i<tm->tm_mon; i++) tm->tm_yday+=days[i];
  tm->tm_yday+=(tm->tm_mday-1);
  tm->tm_isdst=0;
  return secs;
}

boolean calc_pos(GOTO_VAL *gotoval,KEPLER *kepler,EPOINT *refpos)
{
  static int prevsec;
  static struct tm tm;
  time_t t;
  DIR dir;
  EPOINT pos_sat,pos_subsat;
  static boolean above_hor;

  time(&t);
  tm=*gmtime(&t);
  if (prevsec!=tm.tm_sec)
  {
    calc_sat_earth_v2(&tm,0,kepler,NULL,&pos_sat,&pos_subsat);
//printf("%f  %f\n",pos_sat.lat,pos_sat.lon);
    gotoval->height=calceleazim_v2(tm,0,&pos_subsat,&pos_sat,refpos,&dir);
    elevazim2xy(&dir,NULL); // 2e arg.: ROTOR, alleen voor x_west_is_0, y_south_is_0
    gotoval->a=R2D(dir.azim);
    gotoval->e=R2D(dir.elev);
    gotoval->x=R2D(dir.x);
    gotoval->y=R2D(dir.y);
    gotoval->lon=R2D(pos_subsat.lon);
    gotoval->lat=R2D(pos_subsat.lat);

    if (gotoval->e < 0)
    {
        gotoval->ax=ROTOR_AX_STOP;
        gotoval->ey=ROTOR_EY_STOP;
        above_hor=false;
    }
    else
    {
      #if ROTORTYPE == ROTORTYPE_XY
        gotoval->ax=gotoval->x;
        gotoval->ey=gotoval->y;
      #else
        gotoval->ax=gotoval->a;
        gotoval->ey=gotoval->e;
      #endif
      above_hor=true;
    }
    prevsec=tm.tm_sec;
  }
  return above_hor;
}
