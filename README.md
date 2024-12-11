# FrankaDiffusionPolicy
This repository contains a ROS package used to execute Diffusion Policy on a Franka Emika Panda Robot. This repository is responsible for executing action sequences on the Franka robot. It is used in series with the [Diffusion Repository](https://github.com/courtSmith77/diffusion_policy).

Check out the portfolio Post for this Project [here](https://courtsmith77.github.io/projects/07-diffusionpolicy).

## How to run
### Simulation
1. To begin simulation run `ros2 launch action_franka_bridge integrate_servo.launch.py`. This will launch all nodes and the franka in an rviz simulation.
2. Separately, follow the instruction in the diffusion repository to load and run inference on your diffusion model.
3. Once both are running, use the commands in the xterm window to enable and disable inference in simulation.

### Real Robot
1. On your commuter run `ros2 launch action_franka_bridge integrate_servo.launch.py use_fake_hardware:=false robot_ip:=panda0.robot launch_moveit:=false`
2. On the Franka run `ros2 launch action_franka_bridge integrate_servo.launch.py use_fake_hardware:=false robot_ip:=panda0.robot launch_controllers:=false`
3. Separately, follow the instruction in the diffusion repository to load and run inference on your diffusion model.
4. Once all are running, use the commands in the xterm window to enable and disable inference in simulation.


### Nodes

`action_franka_bridge` - Receives the action sequence and commands the Franka via Moveit

`command_mode` - Controls the xterm window and the status of diffusion inference

`data_collection` - Collects both action sequence and end effector data

`model_input_publisher` - Handles modification and publishing of the model observation


### Modules

`moveit_api` - a class that handles the Moveit services and actions