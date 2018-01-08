#include "PID.h"
#include <iostream>
#include <fstream>
#include <math.h>

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_i, double Ki_i, double Kd_i) {
  Kp = Kp_i;
  Ki = Ki_i;
  Kd = Kd_i;

  p_error = 0.;
  i_error = 0.;
  d_error = 0.;

  p = {0.0, 0.0, 0.0};
  //dp = {1.0, 1.0, 1.0};
  dp = {Kp_i*0.1, Ki_i*0.1, Kd_i*0.1};

  init_done = false;
  opt_done = false;
  p_idx = 0;
  runs = 0;
  best_err = 0.0;
  error = 0.0;
}

void PID::UpdateError(double cte) {
  d_error = cte - p_error; 
  p_error = cte;
  i_error += cte;
}

double PID::TotalError() {
  double steer;
  steer = -(Kp * p_error) - (Kd * d_error) - (Ki * i_error) ;
  return steer;
}

void PID::Twiddle(double cte, double tolerance, int n) {
  
  p = {Kp, Ki, Kd};

  if ( init_done == false ) {
    dp = {Kp*0.1, Ki*0.1, Kd*0.1};
    dp = {Kp*0.2, Ki*0.2, Kd*0.2};
    error += cte;
    runs++;
    if ( runs == n ) {  // Init.
      runs = 0;
      error = 0.0;
      best_err = error/n;
      init_done = true;
      // Debug
      cout << ">> Initialization is done." << endl;
    }
    return;
  }
  else {
    if ( (fabs(dp[0])+fabs(dp[1])+fabs(dp[2])) > tolerance ) {
      double current_err;
      if (runs == 0) {
        p[p_idx] += dp[p_idx];
        Kp = p[0];
        Ki = p[1];
        Kd = p[2];
        runs++;
        return;
      }
      else if (runs == n) {
        current_err = error/n;
        if (current_err < best_err) {
          best_err = current_err;
          dp[p_idx] = dp[p_idx] * 1.1;
          runs = 0;
          error = 0;
          if (p_idx < 2)
            p_idx++;
          else
            p_idx = 0;
          return;
        }
        else {
          p[p_idx] = p[p_idx] - 2*dp[p_idx];
          Kp = p[0];
          Ki = p[1];
          Kd = p[2];
          runs++;
          return;
        }
      }
      else if (runs == 2*n) {
        current_err = error/n;
        if (current_err < best_err) {
          best_err = current_err;
          dp[p_idx] = dp[p_idx] * 1.1;
        }
        else {
          p[p_idx] += dp[p_idx];
          dp[p_idx] = dp[p_idx]*0.9;
        }
        runs = 0;
        error = 0;
        if (p_idx < 2)
          p_idx++;
        else
          p_idx = 0;
      }
      else {  // (abs(dp[0])+abs(dp[1])+abs(dp[2]) <= tolerance 
        runs++;
        //error += cte;
        error += cte*cte;
      }
    }
    else {
      //cout << "Optimized Parameter : " << "Kp = " << p[0] << "Ki = "<< p[1] << "Kd = " << p[2] << endl;
      ofstream log;
      log.open ("result_optimization.txt", ios::out | ios::app);
      log << "Optimized Parameters : " << "Kp = " << p[0] << ", Ki = "<< p[1] << ", Kd = " << p[2] << " with best_err = " << best_err << endl;
      log.close();

      init_done = false;
      opt_done = true;
      //error = 0.0;
      //p_idx = 0;
      //runs = 0;
    }
  }
}
