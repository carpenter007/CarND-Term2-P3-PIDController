#include "PID.h"

using namespace std;

/*
* the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  /* Set Errors to zero */
  p_error = 0;
  i_error = 0;
  d_error = 0;
  /* Set coefficients to one */
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
}

void PID::UpdateError(double cte) {

  d_error = cte - p_error;
  i_error += cte;
  p_error = cte; /* Note: update p_error after d_error is updated with previous value */
}

double PID::TotalError() {
  /* Sum of all Errors p, i, d */
  return (d_error + i_error + p_error);
}

