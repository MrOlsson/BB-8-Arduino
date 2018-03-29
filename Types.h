#ifndef TYPES_H
#define TYPES_H

typedef struct RADS{
  double L_wheel;
  double R_wheel;
} RADS;

typedef struct IFS{
  long leftif;
  long rightif;
  long leftelse;
  long rightelse;
} IFS;

#endif
