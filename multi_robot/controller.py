#!/usr/bin/env python3
import roslaunch

robot_list = {}

# all else fails run this via os
# roslaunch rss_assignment multi_robot_spawn.launch robot_name:="hiiii" initial_x:="-x 1" initial_y:="-y 2" model:="burger"
# note the above command does not enable sending individual commands to robots
# commands would just be applied globally

def spawn_bot(x, y):
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    cli_args = ["rss_assignment", "multi_robot_spawn.launch",f'robot_name:="{"tb3_" + str(len(robot_list))}"', f"initial_x:=-x {x}", f"initial_y:=-y {y}", "model:=burger"]
    roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], cli_args[2:])]
    parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
    parent.start()
    return parent

if __name__=="__main__":
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
                    # global y
                    y = float(inputt)
                except ValueError:
                    print("Please enter a valid number(y)")
                else:
                    robot_list["tb3_" + str(len(robot_list))] = spawn_bot(x, y)
                    print(f"new robot spawned at ({x}, {y})")
        elif inputt == "2":
            if len(robot_list) == 0:
                print("list is empty")
            else:
                print(str(robot_list))
                print()

        elif inputt == "3":
            print("test")
        elif inputt == "4" or input == '\x03':
            print("exiting...")
            break
        else:
            print("please enter a valid command")