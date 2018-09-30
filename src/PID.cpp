#include "PID.h"
#include <iostream>
#include <math.h>
#include <vector>

PID::PID() {}

PID::~PID() {}

void PID::Init(double kp, double ki, double kd) {
  Kp = kp;
  Ki = ki;
  Kd = kd;
  p_error = i_error = d_error = 0.0;

  twiddle = false;

  if (twiddle) {
    p = {&Kp, &Ki, &Kd};
    dp = {0.1*Kp, 0.1*Ki, 0.1*Kd};
    duration = 1000;
    timestep = 1;
    index = 0;
    error = 0;
    best_error = 0.5;
    decreased = false;
  }
}

void PID::UpdateError(double cte) {
  d_error = cte - p_error;
  p_error = cte;
  i_error += cte;

  if (twiddle) {
    if (timestep % 100 == 0) {
      std::cout << "Timestep: " << timestep << std::endl;
    }

    Twiddle(cte);
  }
}

double PID::TotalError() {
  return -Kp * p_error - Ki * i_error - Kd * d_error;
}

void PID::Twiddle(double cte) {
  if (timestep != duration) {
    error += pow(cte, 2);
    timestep++;
    return;
  }

  error /= duration;

  if (error < best_error) {
    std::cout << "Improved error: " << error;
    best_error = error;
    dp[index] *= 1.1;

    index = (index + 1) % 3;
    std::cout << "\n\nParameter " << index << " increased\n";
    *(p[index]) += dp[index];
    decreased = false;
  }
  else {
    if (!decreased) {
      std::cout << "Not improved. Best error: " << best_error;
      std::cout << "\nParameter decreased\n";
      *(p[index]) -= 2 * dp[index];
      decreased = true;
    }
    else {
      std::cout << "Not improved. Best error: " << best_error;
      std::cout << "\nParameter increased again";
      *(p[index]) += dp[index];
      dp[index] *= 0.9;

      index = (index + 1) % 3;
      std::cout << "\n\nParameter " << index << " increased\n";
      *(p[index]) += dp[index];
      decreased = false;
    }
  }
    
  timestep = 1;
  error = p_error = i_error = d_error = 0;
  std::cout << "P: " << Kp << "\nI: " << Ki << "\nD: " << Kd << std::endl;
}