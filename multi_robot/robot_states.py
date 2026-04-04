from enum import Enum

class Bot_State(Enum):

    # idle states
    IDLE = "idle"

    # PID states
    ROTATING = "rotating"
    READY = "ready"
    MOVING = "moving"
    WAITING = "waiting"
    EVADING = "evading"

    # teleop states
    TELEOP = "teleop"