#include "PID.h"
#include <iostream>
using std::string;


/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  // PID errors
  this->p_error = 0.0;
  this->i_error = 0.0;
  this->d_error = 0.0;

  // PID coefficients
  this->Kp = Kp_;
  this->Ki = Ki_;
  this->Kd = Kd_;
}

void PID::UpdateError(double cte) {
  // set `p_error` to `cte` if this is the first iteration
  if (p_error == 0 && i_error == 0 && d_error == 0) { p_error = cte; }

  // `D` error is the derivative of the CTE between the last iteration and the current one
  d_error = cte - p_error; // at this step, p_error is the CTE from the previous iteration

  // `P` error is the current CTE
  p_error = cte;

  // `I` error is the sum of all CTE until now
  i_error += cte;
}

double PID::TotalError() {
  return -Kp*p_error - Kd*d_error - Ki*i_error;
}