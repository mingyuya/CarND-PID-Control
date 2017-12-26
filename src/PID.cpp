#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_i, double Ki_i, double Kd_i) {
  Kp = Kp_i;
  Ki = Ki_i;
  Kd = Kd_i;

  p_error = 0.;
  i_error = 0.;
  d_error = 0.;
}

void PID::UpdateError(double cte) {
  d_error = cte - p_error; 
  p_error = cte;
  i_error += cte;
}

double PID::TotalError() {
}

vector<int> PID::Twiddle(double tolerance) {
  vector<double> p;
  p.resize(3);

  return p; 
}
