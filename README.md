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
cd ~/rm_ws/
source /opt/ros/noetic/setup.bash
catkin build
```

4. Run the launch file
```bash
cd ~/rm_ws
source devel/setup.bash

roslaunch youbot_orocos_kdl forward_kinematics.launch
```

- The script checks if URDF is available and if so, it will create a chain based on the links defined in the config.
- Based on the chain created from URDF, it will run FK on the defined set of joint angles.
- Then, the script will create a chain based on DH parameters and run the FK on the defined set of angles.
- The respective joint angles and the calculated position are visualized in RVIZ.

### Configuration

- The configuration for the forward kinematics is defined in the `config/forward_kinematics.yaml` file.

1. Chain links
    - The `chain_links` parameter takes two strings as input. The `KDL::Chain` is created between the first and second link based on the URDF file.

2. Joint angles
    - The `joint_angles` parameter takes a list of joint angles as input for each joint defined in the `joint_names`. The joint angles are in radians.
    - The `default` parameter contains the default joint angles for the Youbot robot which are set to `0`.
    - The remaining are the test sets for the forward kinematics which are defined within the limits of Youbot.

3. DH Parameters
    - The DH parameters are defined in the `dh_params` parameter. The `dh_params` parameter takes a list of DH parameters as input for each joint defined in `joint_names`.
    - Each joint has 4 DH parameters in the order: `JointType, a, alpha, d, theta`.
    - The `JointType` can be any one of the below enum and has to be given as a respective integer.
        ```cpp
        JointType enum {RotAxis(=0), RotX(=1), RotY(=2), RotZ(=3), TransAxis(=4), TransX(=5), TransY(=6), TransZ(=7), None(=8)};
        ```

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
