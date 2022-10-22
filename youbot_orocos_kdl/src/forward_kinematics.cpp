#include <youbot_orocos_kdl/forward_kinamatics.h>

ForwardKinematics::ForwardKinematics(ros::NodeHandle nh):
    nh_(nh)
{
    // get urdf from parameter server
    nh_.param("robot_description", robot_desc_string, std::string());

    // initialize joint state publisher
    joint_state_pub = nh_.advertise<sensor_msgs::JointState>("joint_states", 1);

    // initialize cartesian pose publisher
    cartesian_pose_pub = nh_.advertise<geometry_msgs::PointStamped>("cartesian_point", 1);

    // create string publisher
    string_pub = nh_.advertise<std_msgs::String>("string", 1);

    // create a tree from the urdf
    treeFromUrdf();

    // get chain from base to tip
    urdf_tree.getChain("arm_link_0", "arm_link_5", urdf_chain);

    // create a tree from DH parameters
    chainFromDHParams();

    // get chain from base to tip
    // my_tree.getChain("arm_link_0", "arm_link_5", my_chain);

    // read 5 sets of joint angles from parameter server
    for (int i = 0; i < 5; i++)
    {
        std::vector<double> joint_angles;
        std::string joint_angles_string = "/forward_kinematics/joint_angles/set" + boost::lexical_cast<std::string>(i);
        if (nh_.hasParam(joint_angles_string))
        {
            nh_.getParam(joint_angles_string, joint_angles);
            joint_angles_vector.push_back(joint_angles);
        }
        else 
        {
            ROS_ERROR("No joint angles found for set %d", i);
        }
    }

    // calculate cartesian pose for each joint angle vector
    ROS_INFO("Calculating cartesian pose with urdf chain");

    // wait for 1 second to make sure the publishers are ready
    ros::Duration(4.0).sleep();
    jointAnglesToCartesianPose(urdf_chain);

    // ROS_INFO("Calculating cartesian pose with DH chain");
    // jointAnglesToCartesianPose(my_chain);
}

ForwardKinematics::~ForwardKinematics()
{
}

void ForwardKinematics::treeFromUrdf()
{
    // get urdf from parameter server
    nh_.param("robot_description", robot_desc_string, std::string());

    // parse urdf
    if (!kdl_parser::treeFromString(robot_desc_string, urdf_tree)){
        ROS_ERROR("Failed to construct kdl tree");
    }
}

void ForwardKinematics::chainFromDHParams()
{
    // add segments to the chain using DH parameters
    my_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(0.0, 0.0, 0.0, 0.0)));
    my_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(0.0, 0.0, 0.0, 0.0)));
    my_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(0.0, 0.0, 0.0, 0.0)));
    my_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(0.0, 0.0, 0.0, 0.0)));
    my_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(0.0, 0.0, 0.0, 0.0)));
}


void ForwardKinematics::jointAnglesToCartesianPose(KDL::Chain &chain)
{
    // create forward kinematics solver
    KDL::ChainFkSolverPos_recursive fksolver = KDL::ChainFkSolverPos_recursive(chain);

    // run forward kinematics for each joint angle vector
    for (int i = 0; i < joint_angles_vector.size(); i++)
    {
        // publish joint angles to joint_states topic
        // publishJointAngles(joint_angles_vector[i]);

        // create joint array
        unsigned int nj = chain.getNrOfJoints();
        KDL::JntArray jointpositions = KDL::JntArray(nj);

        // assign joint angles to joint array
        for (int j = 0; j < nj; j++)
        {
            jointpositions(j) = joint_angles_vector[i][j];
        }

        // create frame
        KDL::Frame ee_frame;

        // calculate forward kinematics
        bool kinematics_status;
        kinematics_status = fksolver.JntToCart(jointpositions, ee_frame);
        if(kinematics_status>=0){
            ROS_INFO("FK for joint angles %f, %f, %f, %f, %f", joint_angles_vector[i][0], 
                                                                joint_angles_vector[i][1], 
                                                                joint_angles_vector[i][2], 
                                                                joint_angles_vector[i][3], 
                                                                joint_angles_vector[i][4]);
        }else{
            ROS_INFO("Error: could not calculate forward kinematics :(");
        }

        // extract pose from frame
        double x = ee_frame.p.x();
        double y = ee_frame.p.y();
        double z = ee_frame.p.z();

        // print pose
        ROS_INFO("Pose: x: %f, y: %f, z: %f", x, y, z);

        // publish pose to rviz
        publishPose(x, y, z);
    }
}

void ForwardKinematics::publishJointAngles(std::vector<double> joint_angles)
{
    // create joint state message
    sensor_msgs::JointState joint_state_msg;

    // assign joint names
    joint_state_msg.name.push_back("arm_joint_1");
    joint_state_msg.name.push_back("arm_joint_2");
    joint_state_msg.name.push_back("arm_joint_3");
    joint_state_msg.name.push_back("arm_joint_4");
    joint_state_msg.name.push_back("arm_joint_5");

    // assign joint angles
    joint_state_msg.position.push_back(joint_angles[0]);
    joint_state_msg.position.push_back(joint_angles[1]);
    joint_state_msg.position.push_back(joint_angles[2]);
    joint_state_msg.position.push_back(joint_angles[3]);
    joint_state_msg.position.push_back(joint_angles[4]);

    // publish joint state message
    joint_state_pub.publish(joint_state_msg);
}

void ForwardKinematics::publishPose(double x, double y, double z)
{
    // create point stamped message
    geometry_msgs::PointStamped point_msg;
    point_msg.header.frame_id = "base_link";
    point_msg.header.stamp = ros::Time::now();
    point_msg.point.x = x;
    point_msg.point.y = y;
    point_msg.point.z = z;

    // publish pose message
    cartesian_pose_pub.publish(point_msg);
    // wait for 1 second
    ros::Duration(1.0).sleep();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "forward_kinematics");
    ros::NodeHandle nh;

    ForwardKinematics fk(nh);
    
    ros::spin();

    return 0;
}