# Stembot
An example robot that demonstrates the capabilities of the ROS2 ecosystem.

## Package installation Instructions
1. Open a terminal and navigate to a directory of your choice using `ls` and `cd` commands.
2. Clone the repository:
   ```bash
   git clone https://github.com/treedel/stembot.git
   cd stembot
   ```
3. Build the packages using:
   ```bash
   colcon build
   ```
4. Source the setup script before running any package (do this in every new terminal):
   ```bash
   source install/setup.bash
   ```
   Make sure you are in the `stembot` folder before running the command.

## Package Instructions

### Experiment 1: Robot teleoperation
- Launch the Gazebo simulator:
  ```bash
  ros2 launch stembot_gazebo gazebo.launch.py
  ```
  This starts the Gazebo simulator along with supporting nodes to simulate the robot.
- Run the `teleop_twist_keyboard` node in another terminal to control the robot:
  ```bash
  ros2 run teleop_twist_keyboard teleop_twist_keyboard
  ```
  This node publishes velocity commands (`cmd_vel`) based on keyboard inputs.

### Experiment 2: Robot_Localization Based Odometry
- Launch the Gazebo simulator with EKF odometry enabled:
  ```bash
  ros2 launch stembot_gazebo gazebo.launch.py use_ekf_odom:=true
  ```

### Experiment 3: Robot state manipulation using nodes
There are two approaches for this experiment: using a prebuilt node or creating your own.

#### Option 1: Using the Prebuilt Node
- Launch the Gazebo simulator:
  ```bash
  ros2 launch stembot_gazebo gazebo.launch.py
  ```
- Run the `two_point_looper` node in another terminal after sourcing:
  ```bash
  ros2 run stembot_state_manipulator two_point_looper
  ```

#### Option 2: Creating Your Own Node
1. Change to the `src` folder and create an empty ROS2 package:
   ```bash
   ros2 pkg create --build-type ament_python --license Apache-2.0 <package_name>
   ```
2. Navigate to the newly created package directory:
   ```bash
   cd src/<package_name>/<package_name>
   ```
   Here, you'll find a `__init__.py` file. Create a new Python script for your node in this folder.
3. Open `setup.py` (located in `src/<package_name>`) and add an entry point for the node under `console_scripts` as
   `'<node_name> = <package_name>.<python_file_name>:main'`. It should look similar to this:
   ```python
   entry_points={
       'console_scripts': [
           'two_point_looper = stembot_state_manipulator.two_point_looper:main'
       ],
   },
   ```
4. Build, source, and run the node:
   ```bash
   colcon build
   source install/setup.bash
   ros2 run <package_name> <node_name>
   ```

**Note:** You can find reference code in the `code` folder or the `src` package files.

### Experiment 4: Obstacle Avoidance using 2D Lidar
- Use the prebuilt `obstacle_avoider` node:
  ```bash
  ros2 run stembot_obstacle_avoidance obstacle_avoider
  ```
- Alternatively, create your own package and implement the obstacle avoidance logic in a custom node.
- Don't forget to launch the simulator before running the node.

### Experiment 5: Performing SLAM and Mapping
1. Launch the Gazebo simulator with EKF odometry:
   ```bash
   ros2 launch stembot_gazebo gazebo.launch.py use_ekf_odom:=true world_name:=shapes.sdf
   ```
   The `world_name` is a parameter that accepts demo world names from gazebo (ex: shapes.sdf, empty.sdf, etc)
2. Launch the SLAM nodes for mapping:
   ```bash
   ros2 launch stembot_mapping toolbox_oma.launch.py use_sim_time:=true
   ```
   The `use_sim_time:=true` parameter tells the system to use Gazebo's simulated clock.
3. Manually move the robot to explore the environment:
   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```

