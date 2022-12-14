#include <youbot_orocos_kdl/forward_kinamatics.h>

ForwardKinematics::ForwardKinematics(ros::NodeHandle nh):
    nh_(nh)
{
    // initialize joint state publisher
    joint_state_pub = nh_.advertise<sensor_msgs::JointState>("/joint_states", 1);

    // initialize cartesian pose publisher
    cartesian_position_pub = nh_.advertise<geometry_msgs::PointStamped>("cartesian_point", 1);

    // wait for 1 second to make sure the publishers are ready
    ROS_INFO("Waiting for 3 seconds to make sure the publishers and rviz are ready...");
    ros::Duration(3.0).sleep();

    nh_.getParam("joint_names", joint_names);

    nh_.getParam("joint_angles/default", joint_angles_default);

    // publish initial joint state with all joints at 0
    ROS_INFO("Publishing initial joint state");
    publishJointAngles(joint_angles_default);
    ros::Duration(1.0).sleep();

    // read 5 sets of joint angles from parameter server
    int num_test_sets;
    nh_.getParam("joint_angles/num_sets", num_test_sets);
    for (int i = 0; i < num_test_sets; i++)
    {
        std::vector<double> joint_angles;
        std::string joint_angles_string = "joint_angles/set" + boost::lexical_cast<std::string>(i);
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

    // create a tree from the urdf
    if (treeFromUrdf())
    {
        // create a chain between specified links
        nh_.getParam("chain_links", chain_links);
        urdf_tree.getChain(chain_links[0], chain_links[1], urdf_chain);

        // calculate cartesian pose for each joint angle vector
        ROS_INFO("Calculating cartesian pose with urdf chain");
        jointAnglesToCartesianPose(urdf_chain);
    }

    // create a tree from DH parameters
    chainFromDHParams();

    ROS_INFO("Calculating cartesian pose with DH chain");
    jointAnglesToCartesianPose(my_chain);

    // TODO: Use gui to change joint angles

    ROS_INFO("All operations completed, press Ctrl+C to exit.");
}

ForwardKinematics::~ForwardKinematics()
{
}

bool ForwardKinematics::treeFromUrdf()
{
    // get urdf from parameter server
    if (nh_.getParam("/robot_description", robot_desc_string))
    {
        ROS_INFO("Successfully got robot_description from parameter server");

        // parse urdf from robot_description string
        if (!kdl_parser::treeFromString(robot_desc_string, urdf_tree)){
            ROS_ERROR("Failed to construct kdl tree");
            return (false);
        }
        else
        {
            ROS_INFO("Successfully constructed kdl tree");
            return (true);
        }
    }
    else
    {
        ROS_ERROR("Failed to get robot_description from parameter server");
        return (false);
    }
}

void ForwardKinematics::chainFromDHParams()
{
    // get number of joints from parameter server
    int num_joints;
    nh_.getParam("dh_params/num_joints", num_joints);
    // add segments to the chain using DH parameters from parameter server for each joint
    for (int i = 1; i < num_joints+1; i++)
    {
        std::string joint_name = "arm_joint_" + boost::lexical_cast<std::string>(i);
        std::string joint_dh_string = "dh_params/" + joint_name;
        std::vector<double> joint_dh_params;
        if (nh_.hasParam(joint_dh_string))
        {
            // get dictionary of DH parameters for this joint
            XmlRpc::XmlRpcValue joint_dh_dict;
            nh_.getParam(joint_dh_string, joint_dh_dict);
            // get joint type as int from dictionary
            int joint_type = (int)joint_dh_dict["joint_type"];
            // get joint axis array from dictionary
            XmlRpc::XmlRpcValue joint_axis_array = joint_dh_dict["axis"];
            // get dh parameters array from dictionary
            XmlRpc::XmlRpcValue joint_dh_array = joint_dh_dict["dh"];
            // create a joint object from the joint type and joint axis as int
            KDL::Joint joint = KDL::Joint(KDL::Vector(0,0,0), KDL::Vector(
                                static_cast<int>(joint_axis_array[0]),
                                static_cast<int>(joint_axis_array[1]),
                                static_cast<int>(joint_axis_array[2])), KDL::Joint::JointType(joint_type));
            // create a segment object from the joint and dh parameters
            my_chain.addSegment(KDL::Segment(joint_name, joint, 
                                KDL::Frame::DH_Craig1989(static_cast<double>(joint_dh_array[0]), static_cast<double>(joint_dh_array[1]),
                                                         static_cast<double>(joint_dh_array[2]), static_cast<double>(joint_dh_array[3]))));
        }
        else 
        {
            ROS_ERROR("No DH parameters found for joint %s", joint_name.c_str());
        }
    }
}

void ForwardKinematics::jointAnglesToCartesianPose(KDL::Chain &chain)
{
    // create forward kinematics solver
    KDL::ChainFkSolverPos_recursive fksolver = KDL::ChainFkSolverPos_recursive(chain);

    // run forward kinematics for each joint angle vector
    for (int i = 0; i < joint_angles_vector.size(); i++)
    {
        // publish joint angles to joint_states topic
        publishJointAngles(joint_angles_vector[i]);

        // create joint array
        unsigned int nj = chain.getNrOfJoints();
        KDL::JntArray jointpositions = KDL::JntArray(nj);
        // print nj
        ROS_INFO("Number of joints: %d", nj);

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

        // extract position from frame
        double x = ee_frame.p.x();
        double y = ee_frame.p.y();
        double z = ee_frame.p.z();

        // Print the frame
        std::cout << "End-Effector frame\n";
        for (int i = 0; i < 4; i++){
            for (int j = 0; j < 4; j++) {
                double a = ee_frame(i, j);
                if (a < 0.001 && a > -0.001) {
                    a = 0.0;
                }
                std::cout << a << "\t";
            }
            std::cout << std::endl;
        }

        // print position
        ROS_INFO("Calculated Position: x: %f, y: %f, z: %f", x, y, z);

        // publish pose to rviz
        publishPosition(x, y, z);
    }
}

void ForwardKinematics::publishJointAngles(std::vector<double> joint_angles)
{
    // create joint state message
    sensor_msgs::JointState joint_state_msg;

    joint_state_msg.header.stamp = ros::Time::now();

    // assign joint names
    joint_state_msg.name = joint_names;

    // assign joint angles
    joint_state_msg.position = joint_angles;

    // publish joint state message
    joint_state_pub.publish(joint_state_msg);
}

void ForwardKinematics::publishPosition(double x, double y, double z)
{
    // create point stamped message
    geometry_msgs::PointStamped point_msg;
    point_msg.header.frame_id = "base_link";
    point_msg.header.stamp = ros::Time::now();
    point_msg.point.x = x;
    point_msg.point.y = y;
    point_msg.point.z = z;

    // publish pose message
    cartesian_position_pub.publish(point_msg);
    // wait for 2 seconds to visualize pose in rviz
    ros::Duration(2.0).sleep();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "forward_kinematics");
    ros::NodeHandle nh("~");

    ForwardKinematics fk(nh);
    
    while (ros::ok())
    {
        ros::spinOnce();
    }

    return 0;
}