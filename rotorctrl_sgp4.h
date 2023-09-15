#ifndef ROTORCTRL_SGP4_H
#define ROTORCTRL_SGP4_H
#include <time.h>
#include "norad.h"
#include "rotorctrl.h"

#define PI 3.14159265
#define PIx2 (PI*2.)           /* not present in math.h */
#define D2R(g) ((g)*PI/180.)     /* degree --> radians */
#define R2D(g) ((g)*180./PI)     /* radians --> degree */

#define Rearth 6378135.
#define G0    9.798

#define POSLON 5.
#define POSLAT 52.

typedef struct epoint
{
  float x,y,z;
  float lon,lat;
  float alt;
} EPOINT;

typedef struct kepler
{
  char   name[20];
  int    epoch_year;
  float  epoch_day;
  float  decay_rate;
  float  bstar;
  float  inclination;
  float  d_inclination; // inclination in degrees
  float  raan;
  float  d_raan;
  float  eccentricity;
  float  perigee;
  float  d_perigee;
  float  anomaly;
  float  d_anomaly;
  float  motion;
  tle_t  tle;                 // for SGP4
  double sgp4_params[N_SAT_PARAMS];
} KEPLER;

typedef struct dir
{
  float elev,azim;
  float x,y;
} DIR;


#endif
