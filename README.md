# robot-manipulation

- This package contains the code for the basics of robot manipulation using Orocos KDL for Kuka Youbot robot.

## Forward kinematics using KDL

### Dependencies
- KDL
- Eigen3
- Youbot Description

### Environment
- Ubuntu 20.04
- ROS Noetic

### Steps

1. Create a workspace
```bash
mkdir -p ~/rm_ws/src
cd ~/rm_ws/src
```

2. Clone the repository
```bash
git clone -b develop --recurse-submodules -j8 https://github.com/vamsikalagaturu/robot-manipulation.git
```

3. Build the workspace
```bash
source /opt/ros/noetic/setup.bash
catkin build
```

4. Run the launch file
```bash
source devel/setup.bash

roslaunch youbot_orocos_kdl forward_kinematics.launch
```

### Configuration

- The configuration for the forward kinematics is defined in the `config/forward_kinematics.yaml` file.

1. Chain links
    - The chain links are defined in the `chain_links` parameter. The `chain_links` parameter takes two strings as input. The `KDL::Chain` is created between the first and second link based on the URDF file.

2. Joint angles
    - The `joint_angles` parameter takes a list of joint angles as input for each joint defined in the `joint_names`. The joint angles are in radians.
    - The `default` parameter contains the default joint angles for the Youbot robot which are set to `0`.
    - The remaining are the test sets for the forward kinematics which are defined within the limits of Youbot.

3. DH Parameters
    - The DH parameters are defined in the `dh_params` parameter. The `dh_params` parameter takes a list of DH parameters as input for each joint defined in `joint_names`.
    - Each joint has 4 DH parameters in the order: `a, alpha, d, theta`.

4. Arm limits

    ```yaml
      limits:
        arm_joint_1:
            min: 0.0100692
            max: 5.84014
        arm_joint_2:
            min: 0.0100692
            max: 2.61799
        arm_joint_3:
            min: -5.02655
            max: -0.015708
        arm_joint_4:
            min: 0.0221239
            max: 3.4292
        arm_joint_5:
            min: 0.110619
            max: 5.64159
    ```