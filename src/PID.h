#ifndef PID_H
#define PID_H

#include <vector>

class PID {
public:
  PID();
  virtual ~PID();

  void Init(double kp, double ki, double kd);
  void UpdateError(double cte);
  double TotalError();

  double p_error;
  double i_error;
  double d_error;

  // Error coefficients
  double Kp;
  double Ki;
  double Kd;

  // Twiddle
  std::vector<double*> p;
  std::vector<double> dp;
  int duration, timestep, index;
  double error, best_error;
  bool twiddle, decreased;

  void Twiddle(double cte);
};

#endif  // PID_H
