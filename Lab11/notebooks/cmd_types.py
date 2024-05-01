from enum import Enum

class CMD(Enum):
      START_TOF_PID = 0
      STOP_TOF_PID = 1
      START_IMU_PID = 2
      STOP_IMU_PID = 3
      START_MOTOR_CTRL = 4
      STOP_MOTOR_CTRL = 5
      GET_TOF_PID_INFO = 6
      GET_IMU_PID_INFO = 7
      KP_SET = 8
      KI_SET = 9
      KD_SET = 10
      REF_POS_SET = 11
      REF_ANGLE_SET = 12
      MOTOR_PMW_SET = 13
      GET_MOTOR_TOF_INFO = 14
      GET_DATA_STUNT = 15
      START_MAPPING = 16
      STOP_MAPPING = 17
      GET_DATA_MAP = 18