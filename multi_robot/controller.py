#!/usr/bin/env python3
import heapq
import math
import queue
import threading
from typing import Any, Tuple

import roslaunch
import rospy
import sys
import select
import termios
import tty
import random
import time

from robot_obj import robot_obj
from robot_states import Bot_State
from controller_states import State
from queue import PriorityQueue
from robot_waypoint_states import Waypoint_State

robot_list = {} # dict to hold robot objects
stop = False # stop flag for threaded controllers
control_thread = None # variable to hold threaded controller functions
request_queue = PriorityQueue() # thread safe priority queue for centralized mode
reservations = None # reservation dict with identifications on which bot reserved which coords

def spawn_bot(name, x, y) -> robot_obj:
    """
    Spawns a new turtlebot within the simulation

    :arg str name: Name of the turtlebot
    :arg float x: x coordinate of the spawn location
    :arg float y: y coordinate of the spawn location

    :returns: Robot object to control the spawned turtlebot
    :rtype: robot_obj
    """
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    cli_args = ["rss_assignment", "multi_robot_spawn.launch", f'robot_name:={name}', f"initial_x:=-x {x}", f"initial_y:=-y {y}", "model:=burger"]
    roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], cli_args[2:])]
    parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
    parent.start()
    return robot_obj(name, request_queue)

def display_bot_infos() -> str:
    """
    displays information about all robots, unless the robot list is empty

    :returns: aggregated bot information
    :rtype: str
    """
    output = ""
    if len(robot_list) == 0:
        return "robot list empty"
    
    for obj in robot_list.items(): # check functionality
        output += f"{obj}\n"
    
    return output

def getKey():
    """
    Blocks program and waits for next key input, and returns the input

    :return: key input
    :rtype: AnyStr
    """
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def BFS(start, destination):
    queuee = [start]
    parent_map = {start: None}
    visited = [start]

    # queue stores only coords
    # parent map stores coord and dir
    while queuee:
        # Pop the first node from the queue
        current_node = queuee.pop(0)
        # print(f"{current_node} -> {destination}")

        # Check if we have reached the end and return
        if current_node == destination:
            break

        (x, y) = current_node
        neighbours = []
        if x % 1 == 0:  # up down
            neighbours.append((x, y + 0.5, 0))
        else:
            neighbours.append((x, y - 0.5, 0))

        if x % 1 == 0:  # left right
            neighbours.append((x - 0.5, y, 1))
        else:
            neighbours.append((x + 0.5, y, 1))

        if (x + y) % 1 == 0:  # diagonals
            neighbours.append((x - 0.5, y + 0.5, 2))
            neighbours.append((x - 0.5, y - 0.5, 3))
        else:
            neighbours.append((x + 0.5, y + 0.5, 2))
            neighbours.append((x + 0.5, y - 0.5, 3))

        # Add neighbors to Queue
        for neighbour in neighbours:
            x, y, dirr = neighbour
            if (x, y) not in visited:
                visited.append((x, y))
                parent_map[(x, y)] = (current_node[0], current_node[1], dirr)
                queuee.append((x, y))

    path = []
    # Start from the end node
    current = parent_map[(destination[0], destination[1])]
    # print(parent_map)

    # Loop backwards until start node, parent is None
    # (x, y): (x, y, dir)
    while current is not None:
        path.append(parent_map[(current[0], current[1])])
        current = parent_map[(current[0], current[1])]
        # print(current)

    # Reverse the path to be from start to end and return
    return path[::-1][2:]

def selected_bot_control(selected_bot):
    """
    Sub method of individual robot control in __main__

    :param selected_bot: the selected robot object to be controlled
    :rtype: None
    """
    while True:
        inputt = input(f"selected bot:\n{selected_bot.to_string_long()}\n1. update information\n2. move robot\n9. exit to main menu\n")
        if inputt == "1":
            pass
        elif inputt == "2":
            while True:
                inputt = input(f"select movement mode\n1. teleop\n2. PID\n3. return to previous menu\n")
                if inputt == "1":
                    selected_bot.set_state(Bot_State.TELEOP)
                    instruction = "Reading from keyboard\n------------------------------\n    ^\n    |\n    w\n<-a   d->\n    s\n    |\n    v\n------------------------------\n\n10% \step per press\npress space to stop movement\npress any other key to exit"
                    print(instruction)
                    while True:
                        (speed, rotation) = selected_bot.get_movement_vars()
                        print(f"current speed: {round(speed * 100, 2)}%, current rotation: {round(rotation * 100, 2)}%")
                        key = getKey() # blocks main program while movement dealt with via twist
                        if key == "w":
                            selected_bot.set_movement_vars(speed + 0.1, rotation)
                        elif key == "a":
                            selected_bot.set_movement_vars(speed, rotation + 0.1)
                        elif key == "s":
                            selected_bot.set_movement_vars(speed - 0.1, rotation)
                        elif key == "d":
                            selected_bot.set_movement_vars(speed, rotation - 0.1)
                        elif key == " ":
                            selected_bot.set_movement_vars(0, 0)
                        else:
                            print("exiting teleop mode...")
                            selected_bot.set_state(Bot_State.IDLE)
                            break

                elif inputt == "2": # PID
                    selected_bot.set_state(Bot_State.WAITING)
                    selected_bot.set_controlled(False)
                    while True:
                        inputt = input(f"{selected_bot.PID_debug_string()}\nselect PID operation\n1. add waypoint\n2. add random waypoints\n3. clear PID queue\n4. return to previous menu\nhold enter to update display\n")
                        if inputt == "1":
                            x = 0
                            y = 0
                            waypoint_state = Waypoint_State.GRANTED_DESTINATION
                            inputt = input("input the x coordinate of the waypoint: ")
                            try:
                                x = float(inputt)
                            except ValueError:
                                print("invalid input")
                                continue
                            inputt = input("input the y coordinate of the waypoint: ")
                            try:
                                y = float(inputt)
                            except ValueError:
                                print("invalid input")
                                continue
                            inputt = input("what type of waypoint is this (D/w/h): ")
                            inputt = inputt.lower() if inputt != "" else "d"
                            if inputt == "w":
                                waypoint_state = Waypoint_State.WAYPOINT
                            elif inputt == "h":
                                waypoint_state = Waypoint_State.HOLD
                            elif inputt != "d":
                                print("invalid input")
                                continue
                            selected_bot.PID_enqueue(x, y, waypoint_state)
                        elif inputt == "2":
                            inputt = input("input the number of coords to travel to: ")
                            try:
                                n = int(inputt)
                            except ValueError:
                                print("invalid input")
                                continue
                            random.seed(time.time())
                            for i in range(n):
                                selected_bot.PID_enqueue(random.randint(-10, 10), random.randint(-10, 10), Waypoint_State.GRANTED_DESTINATION if random.randint(0, 1) == 1 else Waypoint_State.WAYPOINT)
                        elif inputt == "3":
                            print("cleared PID queue")
                            selected_bot.PID_clear()
                        elif inputt == "4":
                            print("exiting PID mode...")
                            selected_bot.PID_clear()
                            selected_bot.set_state(Bot_State.IDLE)
                            break

                elif inputt == "3":
                    print("returning...")
                    break
                else:
                    print("invalid input")

        elif inputt == "9":
            print("exitting to main menu...")
            return

def random_PID_movement_controller(): # thread to manage robots when on PID movement mode
    """
    Function to be run in a seperate thread to control all robots on simulation, switch the code flag to true to stop the function

    :rtype: None
    """
    # todo collision avoidance???

    # todo centralised control
    # todo 1. controller track environment via in thread memory grid (allowing bots to be in .5 coords mean the grid tracked by controller is -40 to 40 on both axis)
    # todo 2. bots are allowed to traverse up down left right and diagonal 45deg, maybe allow other diag angles in expense of reserving more than the bot needs
    # todo 3. controller on giving new dest coords also give waypoints from a* to bot, controller also allows certain bots to reserve entire path to simulate priority
    # todo 4. bot will reserve 3 coords, coord it just left, coord currently traversing to, and next valid coord past that. bots will wait at second coord until third coord reserved
    # todo 5. bot will report arrived coord for controller to free up coord reserve
    # todo 6. bot and controller will contact via predefined API via maybe a processing queue on controller thread

    # todo distributed control
    # todo 1. controller role limited to only providing new destination coords and robots will travel in straight line towards dest
    # todo 2. robots will publish their position and 1m trajectory to every other robot
    # todo 3. robots will keep track of the 1m trajectory of the all robots within 3m
    # todo 4. if 2 1m trajectories are found to be colliding, the robot furthest from the intersect point would be evading collision
    # todo 5: if the intersect angle > 90deg, evading robot would make course for parallel movement against the other robot (set rotation PID angle to opposing robot orientation), and resume normal course when the distance between the 2 increases
    # todo 6: if the intersect angle <= 90deg, evading robot would set angle to halfway point of opposing robot trajectory, and resume normal course when the distance between the 2 increases
    # todo 7: if one robot is arriving and one isnt, the non arriving robot would plot non stopping waypoint either opposing robot current location (when opposing trajectory > 0.5 long) or 0.5 distance away to opposing robot's current destination with angle set to opposing robot angle to attack towards destination (when opposing trajectory <= 0.5 long), and resume course when the distance between teh 2 bots increases
    # todo 8: if more than 1 robot arriving in destinations within 0.5 distance of each other, make a queue and wait for the robot arriving first to leave

    random.seed(time.time())
    while not stop:

        if reservations: # centralised control mode

            # free up space logic
            for reservation in list(reservations.items()):
                # print(reservation) # debug
                if reservation[1] is not None and len(reservation[1]) > 1:
                    bot: robot_obj
                    bot = robot_list[reservation[0]]
                    if bot.is_ready():
                        # print(f"test: {reservation[0]}, {reservation[1]}\ntype0: {type(reservation[1][0])}\ntype1: {type(reservation[1][1])}")
                        if bot.bot_distance_to_point(reservation[1][0][0], reservation[1][0][1]) > bot.bot_distance_to_point(reservation[1][1][0], reservation[1][1][1]):
                            # print(f"bot {bot.name} left {reservation[1][0]}")
                            reservation[1].pop(0)

            # reservation logic
            request_queue.put((2, "STOP"))
            queue_list = []
            # request_queue.mutex.acquire_lock()
            while True:
                request = request_queue.get()
                # print(request) # debug
                if request == (2, "STOP"):
                    break
                else:
                    queue_list.append(request)
            # request_queue.mutex.release_lock()

            while len(queue_list) > 0: # clear the queue
                # print(f"list: {queue_list}")
                request: Tuple[int, Tuple[float, float, Waypoint_State], str] = queue_list.pop(0)
                skip = False
                for key in list(reservations.keys()): # todo check if for loop modifies var outside
                    if (request[1][0], request[1][1]) in reservations[key]:
                        request_queue.put(request) # loop the unsolvable request back into the queue
                        skip = True
                        break

                if not skip:
                    # print(request)
                    reservations[request[2]].append((request[1][0], request[1][1]))
                    # if len(robot_list[request[2]].PID_queue) > request[3]:
                    robot_list[request[2]].update_waypoint(request[1], (request[1][0], request[1][1], Waypoint_State.WAYPOINT if request[1][2] == Waypoint_State.REQUESTED else Waypoint_State.GRANTED_DESTINATION))
                    # else:
                    #     print(f"bot {request[2]} had empty queue when attempting to update waypoint\nqueue: {robot_list[request[2]].PID_queue}\nupdating waypoint: {(request[1][0], request[1][1], Waypoint_State.WAYPOINT if request[1][2] == Waypoint_State.REQUESTED else Waypoint_State.GRANTED_DESTINATION)}\nqueue id: {request[3]}")
                    print(f"granted bot {request[2]} permission to travel to {(request[1][0], request[1][1])}")

            for name in list(robot_list.keys()):
                bot = robot_list[name]
                # (re)initialize bots when they are done/spawned
                if not bot.is_ready():
                    continue
                if bot.get_state() in [Bot_State.WAITING, Bot_State.IDLE]:
                    if bot.PID_if_queue_empty():
                        # print(f"{bot.name}: {bot.PID_if_queue_empty()}")
                        new_dest = (random.randint(-20, 20) / 2, random.randint(-20, 20) / 2)
                        current_coord = (round_point_5(bot.position[0]), round_point_5(bot.position[1]))

                        # pathing:
                        # up down => up only on whole int, down only on .5
                        # left right => left only on whole int, right only on .5
                        # left up right down diag => up only on whole int, down only on .5 (sum the coords)
                        # left down right up diag => down only on whole int, up only on .5 (sum the coords)

                        # 1. gather all available next nodes
                        # 2. get node that brings the bot closer to the target
                        # 3. put all nodes in a list
                        # 4. resolve into waypoints

                        last_dir = -1
                        path = BFS(current_coord, new_dest)
                        for (x, y, dirr) in path: # node format: (x, y, dir)
                            waypoint_state = Waypoint_State.HOLD if last_dir == -1 or last_dir == dirr else Waypoint_State.DESTINATION
                            bot.PID_enqueue(x, y, waypoint_state)
                            # print(f"enqueued bot {name}: ({x}, {y}) {waypoint_state.value}")
                            last_dir = dirr
                        bot.PID_enqueue(new_dest[0], new_dest[1], Waypoint_State.DESTINATION)

                        if bot.get_state() == Bot_State.WAITING:
                            print(f"bot {name} arrived at destination, given new destination: {new_dest}")
                        else:
                            bot.set_state(Bot_State.WAITING)
                            print(f"bot {name} given destination: {new_dest}")
                elif bot.get_state() == Bot_State.READY:
                    # print("greeeeeeeeeeeeeeen")
                    bot.give_green_light()

    print("stop signal received")
    for name in robot_list.keys():
        bot = robot_list[name]
        bot.PID_clear()
        bot.set_state(Bot_State.IDLE)
        print(f"bot {name} halted")

    print("thread joining...")

def round_point_5(num):
    return 0.5 * round(num / 0.5)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    rospy.init_node(f'multibot_node')
    state = State.MANUAL
    random.seed(time.time())
    while True:
        status = f"number of robots: {len(robot_list)}\ncurrent mode: {state.value}"
        options = "1. spawn new robot\n2. view robot info\n3. control individual robots\n4. multi robot control\n9. exit"
        inputt = input(f"{status}\n{options}")
        print(inputt)
        if inputt == "1":
            inputt = input("Please enter the x coordinate of the new robot: ")
            try:
                x = float(inputt)
            except ValueError:
                print("Please enter a valid number(x)")
            else:
                inputt = input("Please enter the y coordinate of the new robot: ")
                try:
                    y = float(inputt)
                except ValueError:
                    print("Please enter a valid number(y)")
                else:
                    name = f"tb3_{str(len(robot_list))}"
                    robot_list[name] = spawn_bot(name, x, y)
                    print(f"new robot spawned at ({x}, {y})")
        elif inputt == "2":
            if len(robot_list) == 0:
                print("list is empty")
            else:
                for bot in robot_list.items():
                    print(bot)

        elif inputt == "3":
            if len(robot_list) == 0:
                print("list is empty")
            else:
                counter = 0
                for bot in robot_list.items():
                    print(f"{counter}. {bot}")
                    counter += 1
                
                inputt = input("Please select robot to control: ") # fix ui
                try:
                    inputt = int(inputt)
                except ValueError:
                    print("invalid input, returning to main menu")
                    continue
                else:
                    if int(inputt) < len(robot_list): # add type check
                        selected_bot_control(robot_list[f"tb3_{inputt}"])
                    else:
                        print("invalid input, returning to main menu")
                        
        elif inputt == "4":
            while True:
                inputt = input("1. centralised control movement\n2. distributed control movement\n3. formation movement\n4. return to main menu\n")
                if inputt == "1":
                    stop = False
                    reservations = {}
                    for name in robot_list.keys():
                        robot = robot_list[name]
                        reservations[name] = [(round_point_5(robot.position[0]), round_point_5(robot.position[1]))]
                    control_thread = threading.Thread(target=random_PID_movement_controller)
                    control_thread.start()
                    print("control thread started")
                    while True:
                        inputt = input("1. add new bot\n2. print bot infos\n3. view reserved coordinates\n4. exit to previous menu\n")
                        if inputt == "1":
                            while True:
                                x, y = random.randint(-10, 10), random.randint(-10, 10)
                                for key in list(reservations.keys()):
                                    if (x, y) in reservations[key]:
                                        continue
                                break
                            name = f"tb3_{str(len(robot_list))}"
                            reservations[name] = [(x, y)]
                            bot = spawn_bot(name, x, y)
                            robot_list[name] = bot
                            robot_list[name].set_controlled(True)
                            print(f"new robot spawned at ({x}, {y})")
                        elif inputt == "2":
                            while True:
                                bot_list_temp = []
                                for bot in list(robot_list.keys()):
                                    bot_list_temp.append(robot_list[bot])

                                for i in range(len(bot_list_temp)):
                                    print(f"{i}. {bot_list_temp[i].PID_info_string()}")

                                inputt = input("input the corresponding number to view detailed stats\nhold enter to update\nenter any invalid character to exit\n")

                                if inputt == "":
                                    continue

                                try:
                                    inputt = int(inputt)
                                except ValueError:
                                    print("returning to previous menu")
                                    break

                                if 0 <= inputt < len(robot_list):
                                    selected_bot = bot_list_temp[inputt]
                                    while True:
                                        inputt = input(f"{selected_bot.PID_debug_string()}hold enter to update\nenter any character to exit\n")
                                        if inputt != "":
                                            break
                                else:
                                    print("returning to previous menu")
                                    break
                        elif inputt == "3":
                            for item in reservations.items():
                                print(f"{item[0]}: {item[1]}")
                        elif inputt == "4":
                            break
                        else:
                            print("invalid input")
                    stop = True
                    control_thread.join()
                    reservations = None
                    print("exiting to previous menu...")
                elif inputt == "2":
                    ...
                elif inputt == "3":
                    ...
                elif inputt == "4":
                    print("exiting to main menu...")
                    break
                else:
                    print("invalid input")

        elif inputt == "9" or input == '\x03':
            print("exiting...")
            break
        else:
            print("please enter a valid command")