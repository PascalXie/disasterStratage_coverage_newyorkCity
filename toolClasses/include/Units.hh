#ifndef UNITS_H
#define UNITS_H 1
// Units

// length
static const double m	= 1.;
static const double km  = 1e3*m;
static const double cm	= 1e-1*m;
static const double mm	= 1e-3*m;

// area
static const double mm2 = mm*mm;
static const double cm2 = cm*cm;
static const double  m2 = m*m;

// volume
static const double mm3 = mm*mm*mm;
static const double cm3 = cm*cm*cm;
static const double  m3 = m*m*m;

// mass
static const double g = 1.;
static const double kg = 1000.*g;

// time 
static const double second	= 1.;
static const double minute	= 60.*second;
static const double hour	= 60.*minute;

// speed
static const double m_s	= 1.;


// PI
static const double PI = 3.1415926;
#endif
