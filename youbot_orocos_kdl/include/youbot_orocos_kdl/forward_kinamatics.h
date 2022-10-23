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

    /** \brief calculate cartesian position from joint angles using FK solver
     *  \param[in] chain the chain to use for the FK solver
     */
    void jointAnglesToCartesianPose(KDL::Chain &chain);

    /** \brief create a chain from DH parameters 
     * \param[out] bool true if successful
     */
    bool treeFromUrdf();

    /** \brief create a chain from DH parameters */
    void chainFromDHParams();

    /** \brief Publish joint angles to /joint_states topic
    * \param joint_angles vector of joint angles
    */
    void publishJointAngles(std::vector<double> joint_angles);

    /** \brief Publish calculated position to rviz
    * \param[in] x input
    * \param[in] y input
    * \param[in] z input
    */
    void publishPosition(double x, double y, double z);

    ros::NodeHandle nh_;

    std::string robot_desc_string;
    KDL::Tree urdf_tree;
    KDL::Chain urdf_chain;
    KDL::Tree my_tree;
    KDL::Chain my_chain;

    // double vector of multiple joint angles
    std::vector<std::vector<double> > joint_angles_vector;

    // joint names
    std::vector<std::string> joint_names;

    // default joint angles
    std::vector<double> joint_angles_default;

    // vector to store specific links
    std::vector<std::string> chain_links;

    // joint state publisher
    ros::Publisher joint_state_pub; 

    // cartesian pose publisher
    ros::Publisher cartesian_position_pub;
};    

#endif // FK_H