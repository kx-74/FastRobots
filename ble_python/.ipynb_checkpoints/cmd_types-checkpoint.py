from enum import Enum

class CMD(Enum):
    GET_ANGLES = 0
    START_RECORD = 1
    STOP_RECORD = 2
    GET_RECORDED_DATA = 3