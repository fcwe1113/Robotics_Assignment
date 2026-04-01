from enum import Enum

class State(Enum):

    # idle states
    IDLE = 0

    # PID states
    ROTATING = 1
    READY = 2
    MOVING = 3
    WAITING = 4

    # teleop states
    TELEOP = 5