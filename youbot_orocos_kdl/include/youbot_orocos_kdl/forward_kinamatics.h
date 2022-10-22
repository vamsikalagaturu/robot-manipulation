// forward kinematics ros code for the youbot arm using Orocos KDL

#ifndef FK_H
#define FK_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>

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

    ros::NodeHandle nh_;

    std::string robot_desc_string;
    KDL::Tree urdf_tree;
    KDL::Chain urdf_chain;
    KDL::Tree my_tree;
    KDL::Chain my_chain;

    // double vector of multiple joint angles
    std::vector<std::vector<double> > joint_angles_vector;

};    

#endif // FK_H