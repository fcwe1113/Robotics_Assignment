#!/usr/bin/env python3
import roslaunch
import rospy
import sys
import select
import termios
import tty

from robot_obj import robot_obj

robot_list = {}

# all else fails run this via os
# roslaunch rss_assignment multi_robot_spawn.launch robot_name:="hiiii" initial_x:="-x 1" initial_y:="-y 2" model:="burger"
# note the above command does not enable sending individual commands to robots
# commands would just be applied globally

def spawn_bot(name, x, y) -> robot_obj:
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    cli_args = ["rss_assignment", "multi_robot_spawn.launch", f'robot_name:={name}', f"initial_x:=-x {x}", f"initial_y:=-y {y}", "model:=burger"]
    roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], cli_args[2:])]
    parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
    parent.start()
    return robot_obj(name)

def display_bot_infos() -> str:
    output = ""
    if len(robot_list) == 0:
        return "robot list empty"
    
    for obj in robot_list.items(): # check functionality
        output += obj + "\n"
    
    return output

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def selected_bot_control(selected_bot):
    while True:
        inputt = input(f"selected bot:\n{selected_bot.to_string_long()}\n1. update information\n2. move robot\n9. exit to main menu\n")
        if inputt == "1":
            pass
        elif inputt == "2":
            while True:
                # do normal movement b4 PID via {not_name}/cmd_vel
                inputt = input(f"select movement mode\n1. teleop\n2. PID\n3. return to previous menu\n")
                if inputt == "1":
                    selected_bot.set_moving(True)
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
                            selected_bot.stop_moving()
                            break

                elif inputt == "2": # PID
                    selected_bot.set_moving(True)
                    selected_bot.set_use_PID(True)
                    while True:
                        inputt = input(f"{selected_bot.PID_debug_string()}\nselect PID operation\n1. add waypoint\n2. return to previous menu\nhold enter to update display\n")
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
                            print("exiting PID mode...")
                            selected_bot.PID_clear()
                            selected_bot.stop_moving()
                            selected_bot.set_use_PID(False)
                            break

                elif inputt == "3":
                    print("returning...")
                    break
                else:
                    print("invalid input")

        elif inputt == "3":
            ...
        elif inputt == "9":
            print("exitting to main menu...")
            return

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    rospy.init_node(f'multibot_node')
    while True:
        status = f"""number of robots: {len(robot_list)}
        current mode: {11}
        """
        options = """1. spawn new robot
        2. view robot info
        3. control individual robots
        4. exit
        """
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
                if int(inputt) < len(robot_list): # add type check
                    selected_bot_control(robot_list[f"tb3_{inputt}"])
                else:
                    print("invalid input, returning to main menu")
        elif inputt == "4" or input == '\x03':
            print("exiting...")
            break
        else:
            print("please enter a valid command")