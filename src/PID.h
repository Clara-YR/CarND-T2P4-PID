#ifndef PID_H
#define PID_H

#include <iostream>

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

  double prev_cte;
  double dt;

  /*
  * Auxiliary variables for TWIDDLE
  */
  double dKp;
  double dKi;
  double dKd;
  double best_error;
  double sum_cte2;
  int cte_counter;
  int twiddle_step;
  int n_run;  // the times for the run
  bool twiddle_done = true;

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
  void Init(double Kp, double Ki, double Kd, bool twiddle);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte, double speed);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  void Twiddle();
};

#endif /* PID_H */
