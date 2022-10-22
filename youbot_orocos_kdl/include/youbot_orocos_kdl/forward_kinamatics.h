// forward kinematics ros code for the youbot arm using Orocos KDL

#ifndef FK_H
#define FK_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PointStamped.h>

#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/frames_io.hpp>

class ForwardKinematics
{
public:
    ForwardKinematics(ros::NodeHandle nh);
    virtual ~ForwardKinematics();

    void jointAnglesToCartesianPose(KDL::Chain &chain);

    void treeFromUrdf();
    void chainFromDHParams();

    void publishJointAngles(std::vector<double> joint_angles);

    void publishPose(double x, double y, double z);

    ros::NodeHandle nh_;

    std::string robot_desc_string;
    KDL::Tree urdf_tree;
    KDL::Chain urdf_chain;
    KDL::Tree my_tree;
    KDL::Chain my_chain;

    // double vector of multiple joint angles
    std::vector<std::vector<double> > joint_angles_vector;

    ros::Publisher string_pub;

    // joint state publisher
    ros::Publisher joint_state_pub; 

    // cartesian pose publisher
    ros::Publisher cartesian_pose_pub;

};    

#endif // FK_H