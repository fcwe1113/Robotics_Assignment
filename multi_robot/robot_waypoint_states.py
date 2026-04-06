from enum import Enum

class Waypoint_State(Enum):

    DESTINATION = "destination"
    WAYPOINT = "waypoint"
    HOLD = "hold"
    REQUESTED = "requested"