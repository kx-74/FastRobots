#ifndef COMMAND_TYPES_H
#define COMMAND_TYPES_H

enum CommandTypes {
  START_RECORD,
  STOP_RECORD,
  GET_RECORDED_ANGLES,
  GET_RECORDED_DISTANCES,
  GET_PID_INFO,
  MOTOR_STOP,
  MOTOR_FORWARD,
  MOTOR_BACKWARD,
  KP_SET,
  KI_SET,
  KD_SET,
};

#endif // COMMAND_TYPES_H