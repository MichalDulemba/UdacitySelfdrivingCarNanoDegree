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
  double throttle;
  double speed_d;
  double speed_p;
  double K_angle;
  double max_throttle;
  int steps;
  double average_cte;
  double average_speed;
  double total_cte;
  double max_speed;
  double max_error;
  double speed_sum;
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
  void Init(double Kp, double Ki, double Kd, double throttle, double speed_p, double speed_d, double k_angle);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  double SpeedError(double angle);

};

#endif /* PID_H */
