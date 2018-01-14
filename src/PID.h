#include <vector>

#ifndef PID_H
#define PID_H

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  /*
  * For Twiddle
  */
  bool twiddle_on;
  std::vector<double> p;
  std::vector<double> dp;

  bool init_done, opt_done;
  int p_idx;
  int runs;
  double best_err, error;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the output of PID controller.
  */
  double CalcPIDOut();

  /*
  * Calculate the total PID error.
  */
  bool TotalError(double cte, int times);

  /*
  * Twiddle for optimizing parameters
  */
  void Twiddle(double cte, double tolerance, int n);
};

#endif /* PID_H */
