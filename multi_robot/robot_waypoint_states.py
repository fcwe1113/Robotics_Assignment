from enum import Enum

class Waypoint_State(Enum):

    # destination states
    DESTINATION = "destination" # planned destination, not reserved
    REQUESTED_DESTINATION = "requested_destination" # requested destination awaiting approval
    GRANTED_DESTINATION = "granted destination"

    # waypoint states
    HOLD = "hold"
    REQUESTED = "requested"
    WAYPOINT = "waypoint"