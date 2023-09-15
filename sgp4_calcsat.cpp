/**************************************************
 * RCSId: $Id: sgp4_calcsat.cpp,v 1.2 2023/09/06 19:18:25 ralblas Exp $
 *
 * Calculate current observation point for polar satellites 
 * Project: xtrack
 * Author: R. Alblas
 *
 * History: 
 * $Log: sgp4_calcsat.cpp,v $
 * Revision 1.2  2023/09/06 19:18:25  ralblas
 * _
 *
 * Revision 1.3  2021/05/12 13:51:06  ralblas
 * _
 *
 * Revision 1.2  2021/04/07 07:42:21  ralblas
 * _
 *
 * Revision 1.1  2020/06/04 12:28:39  ralblas
 * Initial revision
 *

 *
 **************************************************/
/*******************************************************************
 * Copyright (C) 2000 R. Alblas 
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
/**************************************************
 *  'Public' functions:
 * int calc_sgp4_const(KEPLER *kepler,boolean from_degrees)
 * double calceleazim_v2(struct tm cur_tm,int ms,EPOINT *pos_subsat,EPOINT *pos_sat,EPOINT *refpos,DIRECTION *satdir)
 * //void calcposrel_v2(KEPLER *kepler,EPOINT *pos_sat,EPOINT *pos_earth,EPOINT *pos_rel)
 * void calc_sat_earth_v2(struct tm *cur_tm,int cur_ms, // time
 *                  KEPLER *kepler,               // sat. parameters
 *                  EPOINT *pos_earth,            // pos. earth (rotation), may be NULL
 *                  EPOINT *pos_sat,              // pos. satellite, may be NULL
 *                  EPOINT *pos_subsat)           // sub-satellite position w.r.t. earth
 **************************************************/
#include "norad.h"
#include "norad_in.h"
#include "rotorctrl.h"
#include "rotorctrl_sgp4.h"
#include <math.h>

#define J2000 2451545.5
#define J1900 (J2000 - 36525. - 1.)
#define YEAR1970 25568. // 365.25*70 + 0.5
#define UNIX2JD(t) (((double)(t)/86400.)+J1900+YEAR1970)

#define MINUTES_PER_DAY 1440.
#define MINUTES_PER_DAY_SQUARED (MINUTES_PER_DAY * MINUTES_PER_DAY)
#define MINUTES_PER_DAY_CUBED (MINUTES_PER_DAY * MINUTES_PER_DAY_SQUARED)
#define AE 1.0

long mktime_ntz(struct tm *tm);

static void kepler2tle(KEPLER *kepler, tle_t *tle)
{
  // tle->norad_number
  // tle->bulletin_number
  // tle->classification
  // tle->intl_desig[8]
  tle->epoch=(double)(kepler->epoch_year*365 + (kepler->epoch_year-1)/4) +
                    kepler->epoch_day + J1900;


  // tle->revolution_number=kepler->epoch_rev; // not used
  tle->xmo    = kepler->anomaly;
  tle->xnodeo = kepler->raan;
  tle->omegao = kepler->perigee;
  tle->xincl  = kepler->inclination;
  tle->eo     = kepler->eccentricity;
  tle->xno    = kepler->motion*2*PI/MINUTES_PER_DAY;
  tle->xndt2o = kepler->decay_rate  * 2*PI / MINUTES_PER_DAY_SQUARED;
  //  tle->xndd6o = kepler->decay_rate2 * 2*PI / MINUTES_PER_DAY_CUBED; // not used
  tle->bstar  = kepler->bstar * AE;
  // tle->ephemeris_type = kepler->ephemeris_type; // not used
}

int calc_sgp4_const(KEPLER *kepler,boolean from_degrees)
{
  if (from_degrees)
  {
    kepler->inclination=D2R(kepler->d_inclination);
    kepler->raan       =D2R(kepler->d_raan);
    kepler->perigee    =D2R(kepler->d_perigee);
    kepler->anomaly    =D2R(kepler->d_anomaly);
  }
  kepler2tle(kepler, &kepler->tle);
  SGP4_init(kepler->sgp4_params, &kepler->tle);

  return 1;
}

static double Modulus(double arg1,double arg2)
{
  double modu,Modulus;
  modu = arg1 - (int)(arg1/arg2) * arg2;
  if (modu >= 0)
    Modulus = modu;
  else
    Modulus = modu + arg2;
  return Modulus;
}

#define Frac(n) ((n)-(int)(n))
// Reference:  The 1992 Astronomical Almanac, page B6. 
static double ThetaG_JD(double jd)
{
  double UT,TU,GMST;
  UT   = Frac(jd + 0.5);
  jd   = jd - UT;
  TU   = (jd - 2451545.0)/36525;
  GMST = 24110.54841 + TU * (8640184.812866 + TU * (0.093104 - TU * 6.2E-6));
  GMST = Modulus(GMST + 86400.0*1.00273790934*UT,86400.0);
  return twopi * GMST/86400.0;
}

static void calcposearth_v2(struct tm *cur_tm, int ms, EPOINT *pos_earth)
{
  double theta,jd;
  if (!pos_earth) return;

  jd=UNIX2JD(mktime_ntz(cur_tm)+((float)ms/1000.));
  theta = Modulus(ThetaG_JD(jd),twopi);
  pos_earth->lon=theta-PI/2.;
  pos_earth->lat=0.;
}

#ifdef WEG
// Not neeed for SGP4!
void calcposrel_v2(KEPLER *kepler,
                EPOINT *pos_sat,EPOINT *pos_earth,EPOINT *pos_rel)
{
  if (kepler)
  {
    float d=sqrt(pos_sat->x*pos_sat->x+pos_sat->y*pos_sat->y+pos_sat->z*pos_sat->z);
    pos_rel->lon=-1.*(atan2(pos_sat->x/d,pos_sat->y/d)+pos_earth->lon);
    pos_rel->lat=asin(pos_sat->z/d);
  }
  else
  {
    pos_rel->lon=pos_sat->lon-pos_earth->lon;
    pos_rel->lat=pos_sat->lat;
  }
}
#endif

static EPOINT pos_rel(double xs,double ys,double zs,double jd)
{
  double x,y,z,d;
  double theta;
  EPOINT pos;

  d=sqrt(xs*xs+ys*ys+zs*zs);
  x=xs/d;
  y=ys/d;
  z=zs/d;
  theta = Modulus(ThetaG_JD(jd),twopi);
  pos.lon=-1.*(atan2(x,y)+theta-PI/2.); 
  if (pos.lon<-1*PI) pos.lon+=2.*PI;
  if (pos.lon>+1*PI) pos.lon-=2.*PI;
  pos.lat=asin(z);
  return pos;
}

void calc_sat_earth_v2(struct tm *cur_tm,int cur_ms, // time
                    KEPLER *kepler,               // sat. parameters
                    EPOINT *pos_earth,            // pos. earth (rotation), may be NULL
                    EPOINT *pos_sat,              // pos. satellite, may be NULL
                    EPOINT *pos_subsat)           // sub-satellite position w.r.t. earth
{
  double jd=UNIX2JD(mktime_ntz(cur_tm)+((float)cur_ms/1000.));
  double tsince=(jd-kepler->tle.epoch)*24.*60.; // minutes
  double pos[3];
  double vel[3];
  SGP4(tsince, &kepler->tle,  kepler->sgp4_params,pos, vel);

  *pos_subsat=pos_rel(pos[0],pos[1],pos[2],jd);
  if (pos_sat)
  {
    pos_sat->x=pos[0];
    pos_sat->y=pos[1];
    pos_sat->z=pos[2];
  }
  calcposearth_v2(cur_tm, cur_ms,pos_earth); // pos_earth may be NULL, -> not used
}


// Reference:  The 1992 Astronomical Almanac, page K11. 
static double Calculate_User_Pos(double lat,double lon,double alt,double time,
                        double *x,double *y, double *z)
{
  const double re = earth_radius_in_km;
  double theta;

#ifdef GEEN_CORR
  double r;
  theta = Modulus(ThetaG_JD(time) + lon,twopi);
  r = (re + alt)*cos(lat);
  *x = r*cos(theta);
  *y = r*sin(theta);
  *z = (re + alt)*sin(lat);
#else
  double phi=lat;
  double f=1./298.26;
//  double phia=atan((1-f)*(1-f)*tan(phi));
  double C=1./sqrt(1+f*(f-2)*sin(phi)*sin(phi));
  double S=(1-f)*(1-f)*C;
  theta = Modulus(ThetaG_JD(time) + lon,twopi);
  *x=(re + alt)*C*cos(phi)*cos(theta);
  *y=(re + alt)*C*cos(phi)*sin(theta);
  *z=(re + alt)*S*sin(phi);
#endif
  return theta;
}

static void Calculate_Look(double xs,double ys,double zs,double lat,double lon,
                    double alt,double time,
                    double *az,double *el)
{
  double xo,yo,zo;
  double rx,ry,rz,rg;
  double top_s,top_e,top_z;
  double theta;

  Calculate_User_Pos(lat,lon,alt,time,&xo,&yo,&zo);
  theta = Modulus(ThetaG_JD(time) + lon,twopi);
  rx = xs - xo;
  ry = ys - yo;
  rz = zs - zo;
  top_s = sin(lat)* cos(theta)*rx
         + sin(lat)* sin(theta)*ry
         - cos(lat)*rz;
  top_e = - sin(theta)*rx
         + cos(theta)*ry;
  top_z = cos(lat)* cos(theta)*rx
         + cos(lat)* sin(theta)*ry
         + sin(lat)*rz;
/*
  *az = atan(-1. * top_e/top_s);

  if (top_s > 0) 
    *az = *az + PI;
  if (az < 0) 
    *az = *az + twopi;
*/
  *az = atan2(-1. * top_e,top_s) + PI;
  rg = sqrt(rx*rx + ry*ry + rz*rz);
  *el = asin(top_z/(rg));
}


double calceleazim_v2(struct tm cur_tm,int ms,EPOINT *pos_subsat,EPOINT *pos_sat,EPOINT *refpos,DIR *satdir)
{
  double jd=UNIX2JD(mktime_ntz(&cur_tm)+((float)ms/1000.));
  double az,el,height;
  Calculate_Look(pos_sat->x,pos_sat->y,pos_sat->z,refpos->lat,refpos->lon,refpos->alt,jd,&az,&el);
  satdir->azim=az;
  satdir->elev=el;
  height=1000.*sqrt(pos_sat->x*pos_sat->x+pos_sat->y*pos_sat->y+pos_sat->z*pos_sat->z)-Rearth;
  return height; // in meters, from earth surface
}

// compare of defines: seems always present as 0 even if not defined at all!
#ifndef XY_CONFIG
  #define XY_CONFIG 0
#endif

#include <stdio.h>
//#define X_AT_DISC 0
//#define Y_AT_DISC 1
//#define XY_CONFIG X_AT_DISC
void elevazim2xy(DIR *satdir,ROTOR *rot)
{
  #if XY_CONFIG == X_AT_DISC
  {
    satdir->y=atan2(cos(satdir->azim),tan(satdir->elev));
    satdir->x=asin(sin(satdir->azim)*cos(satdir->elev));

  }
  #else
  {
    satdir->x=atan2(sin(satdir->azim),tan(satdir->elev));
    satdir->y=asin(cos(satdir->azim)*cos(satdir->elev));
  }
  #endif

  // x/y: -90...+90 ==> 0...180
  if ((rot) && (rot->x_west_is_0))
    satdir->x+=D2R(90.);
  else
    satdir->x=D2R(90.)-satdir->x;

  if ((rot) && (rot->y_south_is_0))
    satdir->y+=D2R(90.);
  else
    satdir->y=D2R(90.)-satdir->y;
}
