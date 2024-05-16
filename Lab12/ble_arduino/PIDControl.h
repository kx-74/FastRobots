#ifndef MYFUNCTIONS_H
#define MYFUNCTIONS_H

// Global variables for the pid function
int current_time_pid;
int previous_time_pid;

int pid_ctrl = 0;       // in the range of -255 to 255
float p_ctrl = 0;
float i_ctrl = 0;
float d_ctrl = 0;

int pid_error = 0;            // error between the current and desired distances
int integral_error = 0;       // accumulated error over time
int previous_pid_error = 0;   // error history variable for d_ctrl

int ref_angle = 0;  // desired angle difference from the initial position in degree

void PID_control (float Kp, float Ki, float Kd, int ref_pos, int cur_pos) {
  int dt;
  float edot;

  // Calculate the elapsed time
  current_time_pid = millis();
  dt = current_time_pid - previous_time_pid;
  previous_time_pid = current_time_pid;

  // Error between the current position and the reference position
  pid_error = cur_pos - ref_pos;

  // Integral term
  integral_error = integral_error + pid_error*dt;

  // Wind-up protection
  if (integral_error > 10000)             integral_error = 10000;
  else if (integral_error < -10000)       integral_error = -10000;

  // Derivative term
  edot = ((float)(pid_error - previous_pid_error)) / ((float)dt);
  previous_pid_error = pid_error;

  // The PID control
  p_ctrl = pid_error * Kp;
  i_ctrl = integral_error * Ki;
  d_ctrl = edot * Kd;
  pid_ctrl = p_ctrl + i_ctrl + d_ctrl;

  // Clamp the control value in the range of -255 to 255
  // and avoid the values in the deadband
  if (pid_ctrl < -255)                          pid_ctrl = -255;
  else if (pid_ctrl < 0 && pid_ctrl >= -28)     pid_ctrl = -28;
  else if (pid_ctrl > 0 && pid_ctrl <= 28)      pid_ctrl = 28;
  else if (pid_ctrl > 255)                      pid_ctrl = 255;
}

#endif // MYFUNCTIONS_H