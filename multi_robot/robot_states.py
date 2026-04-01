from enum import Enum

class State(Enum):
    IDLE = 0
    ROTATING = 1
    READY = 2
    MOVING = 3