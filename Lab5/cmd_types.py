from enum import Enum

class CMD(Enum):
      START_RECORD = 0
      STOP_RECORD = 1
      GET_RECORDED_ANGLES = 2
      GET_RECORDED_DISTANCES = 3
      GET_PID_INFO = 4
      MOTOR_STOP = 5
      MOTOR_FORWARD = 6
      MOTOR_BACKWARD = 7
      KP_SET = 8
      KI_SET = 9
      KD_SET = 10