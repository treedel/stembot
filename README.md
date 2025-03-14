# Stembot
An example robot that demonstrates the capabilities of ROS2 ecosystem

## Installation instructions
    - In a empty terminal, go to a directory of your choice using ('ls' and 'cd' commands)
    - Now enter 'git clone https://github.com/treedel/stembot.git' to copy the packages into your PC
    - To install the packages in your system, you now have to build them using 'colcon build'.
    - To run the packages, in every terminal you use, you have to run 'source install/setup.bash' after going into 'stembot' folder

## Package instructions
    - Experiment-1:
        - Launch the gazebo simulator to simulate the robot
        ```ros2 launch stembot_gazebo gazebo.launch.py```
        The above command launches the gazebo simulator and supporting nodes to simulate the robot.

        - Run the teleop_twist node in another terminal to control the robot movements
        ```ros2 run teleop_twist_keyboard teleop_twist_keyboard```
        The above command launches a node that publishes to cmd_vel topic according to keystrokes given.

    - Experiment-2:
        - Same as the above but additional parameter should be added for launching gazebo
        ```ros2 launch stembot_gazebo gazebo.launch.py use_ekf_odom:=true```

- Experiment-3:
    There are two ways to proceed with this experiment. To create a node yourself or use an already created node.
    Either ways, dont forget to launch the simulator.

        - Launch the gazebo simulator to simulate the robot
        ```ros2 launch stembot_gazebo gazebo.launch.py```

        - To use the node that was already created and tested, run the two_point_looper node in another terminal
        ```ros2 run stembot_state_manipulator two_point_looper```

        - To create a node by yourself,
            - You have to change the working directory to src folder and create an empty ROS2 package
            ```ros2 pkg create --build-type ament_python --license Apache-2.0 <package_name>```

            - Now go to the folder that was newly created with given package_name as the name and again go to the subfolder
            with the same name as the package name (src/<package_name>/<package_name>). You will see '__init__.py' file by default where you should create
            a new python file. The code for the new node should be written in that python file.

            - Save the file and now open 'setup.py' file in the previous directory (src/<package_name>). To make ROS know
            about the node you created, you should add entry point for the node. type `<node_name> = <package_name>.<node_python_file_name>:main` between the square brackets
            of console_scripts. It should look something similar to this
            ```entry_points={
                    'console_scripts': [
                        'two_point_looper = stembot_state_manipulator.two_point_looper:main'
                    ],
                },```

            - Finally save the file and proceed to building the packages, sourcing it and run the node
            ```ros2 run <package_name> <node_name>```

            Note: The code for the nodes can be found in the `code` folder and the package files in the src folder can
            also be referred in case of issues.

- Experiment-4:
    Again, this is similar to the previous experiment. In the first approach, just run the following command to launch the node
    ```ros2 run stembot_obstacle_avoidance obstacle_avoider```
    Or alternatively, you can create an empty package and type in the code of obstacle_avoider for the node to work. This also requires the simulator to be launched.

- Experiment-5:
    - Launch the gazebo simulator to simulate the robot
    ```ros2 launch stembot_gazebo gazebo.launch.py use_ekf_odom:=true```
    The above command launches the gazebo simulator and supporting nodes to simulate the robot.

    - Launch the SLAM nodes to begin mapping
    ```ros2 launch stembot_mapping toolbox_oma.launch.py use_sim_time:=true```
    The above command launches the supporting nodes for mapping and the sim time parameter just tells it to use gazebo clock

    - Finally, you can manually move around and map the environment with teleop node
    ```ros2 run teleop_twist_keyboard teleop_twist_keyboard```
