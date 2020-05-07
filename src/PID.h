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
  * Twiddle
  */
  float tolerance;
  double delta_p;

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
  void Init(double Kp, double Ki, double Kd, bool run_twiddle);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  /*
  * Used to reset the simulator and bring the car back to first position.
  */
  void Restart(uWS::WebSocket<uWS::SERVER> ws);

  /*
  * Twiddle or vanilla gradient descent for tuning one hyper parameter at a time
  */
  void Twiddle(double total_error, double hyperparameter);
};

#endif /* PID_H */