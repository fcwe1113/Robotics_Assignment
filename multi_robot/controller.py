#!/usr/bin/env python3
import threading

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

robot_list = {} # dict to hold robot objects
stop = False # stop flag for threaded controllers
control_thread = None # variable to hold threaded controller functions

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
    return robot_obj(name)

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
                            selected_bot.PID_enqueue(x, y)
                        elif inputt == "2":
                            inputt = input("input the number of coords to travel to: ")
                            try:
                                n = int(inputt)
                            except ValueError:
                                print("invalid input")
                                continue
                            random.seed(time.time())
                            for i in range(n):
                                selected_bot.PID_enqueue(random.randint(-10, 10), random.randint(-10, 10))
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
    random.seed(time.time())
    while not stop:
        # 1. set all bots to PID mode
        # 2. give all bots random destinations
        # 3. print bot message queue
        # 4. if bot arrives print message and give new destination
        # 5. when stop flag triggered set all bots to idle

        for name in list(robot_list.keys()):
            bot = robot_list[name]
            if bot.get_state() in [Bot_State.WAITING, Bot_State.IDLE]:
                if bot.PID_if_queue_empty():
                    # print(f"{bot.name}: {bot.PID_if_queue_empty()}")
                    new_dest = [random.randint(-10, 10), random.randint(-10, 10)]
                    bot.PID_enqueue(new_dest[0], new_dest[1])
                    if bot.get_state() == Bot_State.WAITING:
                        print(f"bot {name} arrived at destination, given new destination: {new_dest}")
                    else:
                        bot.set_state(Bot_State.WAITING)
                        print(f"bot {name} given destination: {new_dest}")
            elif bot.get_state() == Bot_State.READY:
                bot.give_green_light()

    print("stop signal received")
    for name in robot_list.keys():
        bot = robot_list[name]
        bot.PID_clear()
        bot.set_state(Bot_State.IDLE)
        print(f"bot {name} halted")

    print("thread joining...")


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
                inputt = input("1. random PID movement\n2. formation movement\n3. return to main menu\n")
                if inputt == "1":
                    stop = False
                    control_thread = threading.Thread(target=random_PID_movement_controller)
                    control_thread.start()
                    print("control thread started")
                    while True:
                        inputt = input("1. add new bot\n2. print bot infos\n3. exit to previous menu\n")
                        if inputt == "1":
                            x, y = random.randint(-10, 10), random.randint(-10, 10)
                            name = f"tb3_{str(len(robot_list))}"
                            robot_list[name] = spawn_bot(name, x, y)
                            print(f"new robot spawned at ({x}, {y})")
                        elif inputt == "2":
                            while True:
                                bot_list_temp = []
                                for bot in list(robot_list.keys()):
                                    bot_list_temp.append(robot_list[bot])

                                for i in range(len(bot_list_temp)):
                                    bot_list_temp[i].set_controlled(True)
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
                            break
                        else:
                            print("invalid input")
                    stop = True
                    control_thread.join()
                    print("exiting to previous menu...")
                elif inputt == "2":
                    ...
                elif inputt == "3":
                    print("exiting to main menu...")
                    break
                else:
                    print("invalid input")

        elif inputt == "9" or input == '\x03':
            print("exiting...")
            break
        else:
            print("please enter a valid command")