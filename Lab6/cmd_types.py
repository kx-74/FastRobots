from enum import Enum

class CMD(Enum):
      START_TOF_PID = 0
      STOP_TOF_PID = 1
      START_IMU_PID = 2
      STOP_IMU_PID = 3
      GET_TOF_PID_INFO = 4
      GET_IMU_PID_INFO = 5
      # // MOTOR_STOP,
      # // MOTOR_FORWARD,
      # // MOTOR_BACKWARD,
      KP_SET = 6
      KI_SET = 7
      KD_SET = 8
      REF_POS_SET = 9
      REF_ANGLE_SET = 10