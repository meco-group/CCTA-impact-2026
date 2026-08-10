// Minimal demo of embedding calls to code-generated CasADi Function 'fitter'
// fitter:(d[5])->(a,b) MXFunction

#include "Arduino_Alvik.h"
#include "fitter.h" // File not found? Run `python construct.py`   

Arduino_Alvik alvik;

// Scratch space needed for code-generated CasADi Function 'fitter'
casadi_int     iw[fitter_SZ_IW];
double          w[fitter_SZ_W];
const double* arg[fitter_SZ_ARG];
double*       res[fitter_SZ_RES];
int mem;

void setup(){
  mem = fitter_checkout(); // May involve allocations
  while((!Serial) && (millis()<3000));
  alvik.begin();
}

void loop(){
  float distances[5];
  float reference = 5.0;
  alvik.get_distance(distances[0], distances[1], distances[2], distances[3], distances[4]);

  // Input: distances in cm (converting from float to double)
  double d[5]; // 5-by-1 matrix
  arg[0] = d;
  for (int i=0;i<5;++i) d[i] = distances[i];

  // Output: Parameters to be fitted
  double a, b;
  res[0] = &a; res[1] = &b;

  // Perform the call; allocation free
  fitter(arg, res, iw, w, mem);

  // NOTE: no notion of signals/time, no semantics, no input output scheme; just blobs of floating point numbers

  // Proportional feedback
  float error_position    = b - 5; // reference is 5cm away
  float error_orientation = a - 0; // reference is to face perpendicularly
  alvik.set_wheels_speed(
    error_position * 10.0 - 100.0*error_orientation,
    error_position * 10.0 + 100.0*error_orientation);
  delay(100);
}
