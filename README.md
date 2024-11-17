# ur3_gazebo_sim

## Install Pinocchio

```bash
 sudo apt install -qqy robotpkg-py3*-pinocchio
```

## Usage

- Launch the Gazebo Simulator

```bash
roslaunch ur3_gazebo ur3.launch
```
-  Open the controller node 

```bash
rosrun ur3_controller ur3_adaptive_controller
```

- Open the motion planning node

```bash
rosrun ur3_controller ur3_motion_planner
```
