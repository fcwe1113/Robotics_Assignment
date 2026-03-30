#!/usr/bin/env python3
import roslaunch
import rospy

from robot_obj import robot_obj

robot_list = {}

# all else fails run this via os
# roslaunch rss_assignment multi_robot_spawn.launch robot_name:="hiiii" initial_x:="-x 1" initial_y:="-y 2" model:="burger"
# note the above command does not enable sending individual commands to robots
# commands would just be applied globally

## maybe split robots via subscribe topics

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

def selected_bot_control(selected_bot):
    while True:
        inputt = input(f"selected bot:\n{selected_bot.to_string_long()}\n1. update information\n2. move forward\n3. change orientation\n9. exit to main menu")
        if inputt == "1":
            pass
        elif inputt == "2":
            # do normal movement b4 PID via {not_name}/cmd_vel
            ...
        elif inputt == "3":
            ...
        elif inputt == "9":
            print("exitting to main menu...")
            return

if __name__=="__main__":
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
                    print(f"{bot}")
                    counter += 1
                
                inputt = input("Please select robot to control")
                if inputt < len(robot_list):
                    selected_bot_control(robot_list[f"tb3_{inputt}"])
                else:
                    print("invalid input, returning to main menu")
        elif inputt == "4" or input == '\x03':
            print("exiting...")
            break
        else:
            print("please enter a valid command")