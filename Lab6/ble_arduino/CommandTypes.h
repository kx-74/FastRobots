#ifndef COMMAND_TYPES_H
#define COMMAND_TYPES_H

enum CommandTypes {
  START_TOF_PID,
  STOP_TOF_PID,
  START_IMU_PID,
  STOP_IMU_PID,
  START_MOTOR_CTRL,
  STOP_MOTOR_CTRL,
  GET_TOF_PID_INFO,
  GET_IMU_PID_INFO,
  // MOTOR_STOP,
  // MOTOR_FORWARD,
  // MOTOR_BACKWARD,
  KP_SET,
  KI_SET,
  KD_SET,
  REF_POS_SET,
  REF_ANGLE_SET,
};

#endif // COMMAND_TYPES_H