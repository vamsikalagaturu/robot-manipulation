#include <youbot_orocos_kdl/forward_kinamatics.h>

ForwardKinematics::ForwardKinematics(ros::NodeHandle nh):
    nh_(nh)
{
    // get urdf from parameter server
    nh_.param("robot_description", robot_desc_string, std::string());

    // create a tree from the urdf
    treeFromUrdf();

    // get chain from base to tip
    urdf_tree.getChain("arm_link_0", "arm_link_5", urdf_chain);

    // create a tree from DH parameters
    chainFromDHParams();

    // get chain from base to tip
    my_tree.getChain("arm_link_0", "arm_link_5", my_chain);

    // define vector of multiple joint angles
    std::vector<double> joint_angles_1;
    joint_angles_1.push_back(0.1);
    joint_angles_1.push_back(0.5);
    joint_angles_1.push_back(-1.0);
    joint_angles_1.push_back(0.1);
    joint_angles_1.push_back(1.0);

    std::vector<double> joint_angles_2;
    joint_angles_2.push_back(2.0);
    joint_angles_2.push_back(1.0);
    joint_angles_2.push_back(-2.0);
    joint_angles_2.push_back(1.0);
    joint_angles_2.push_back(2.0);

    // push back joint angles to joint_angles_vector
    joint_angles_vector.push_back(joint_angles_1);
    joint_angles_vector.push_back(joint_angles_2);
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
            ROS_INFO("SUCCESS, thanks KDL!");
        }else{
            ROS_INFO("Error: could not calculate forward kinematics :(");
        }

        // extract pose from frame
        double x = ee_frame.p.x();
        double y = ee_frame.p.y();
        double z = ee_frame.p.z();

        // print pose
        ROS_INFO("Pose: x: %f, y: %f, z: %f", x, y, z);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "forward_kinematics");
    ros::NodeHandle nh;

    ForwardKinematics fk(nh);
    
    ros::spin();

    return 0;
}