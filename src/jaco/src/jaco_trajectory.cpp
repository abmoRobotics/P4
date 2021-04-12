#include <ros/ros.h>
#include <jaco_trajectory.h>

#include <actionlib/client/simple_action_client.h>
#include <tf_conversions/tf_eigen.h>


const double FINGER_MAX = 6400;

tf::Quaternion EulerZYZ_to_Quaternion(double tz1, double ty, double tz2)
{
    tf::Quaternion q;
    tf::Matrix3x3 rot;
    tf::Matrix3x3 rot_temp;
    rot.setIdentity();

    rot_temp.setEulerYPR(tz1, 0.0, 0.0);
    rot *= rot_temp;
    rot_temp.setEulerYPR(0.0, ty, 0.0);
    rot *= rot_temp;
    rot_temp.setEulerYPR(tz2, 0.0, 0.0);
    rot *= rot_temp;
    rot.getRotation(q);
    return q;
}


bool jaco_trajectory::gripper_action(double finger_turn){
moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    ROS_INFO("GRIPPER_ACTION");
    //ROS_INFO_STREAM(robot_connected_);
    if(robot_connected_ == false)
    {
        if (finger_turn>0.5*FINGER_MAX)
        {

          //gripper_group_->setNamedTarget("Close");
        }
        else
        {
          //gripper_group_->setNamedTarget("Open");
        }

        tripod_grip(finger_turn);
        
        bool success = (gripper_group_->plan(my_plan) == moveit_msgs::MoveItErrorCodes::SUCCESS);
        //gripper_group_->move();
        gripper_group_->execute(my_plan);
        return true;
    }

    if (finger_turn < 0)
    {
        finger_turn = 0.0;
    }
    else
    {
        finger_turn = std::min(finger_turn, FINGER_MAX);
    }

    kinova_msgs::SetFingersPositionGoal goal;
    goal.fingers.finger1 = finger_turn;
    goal.fingers.finger2 = goal.fingers.finger1;
    goal.fingers.finger3 = goal.fingers.finger1;
    finger_client_->sendGoal(goal);

    if (finger_client_->waitForResult(ros::Duration(5.0)))
    {
        finger_client_->getResult();
        return true;
    }
    else
    {
        finger_client_->cancelAllGoals();
        ROS_WARN_STREAM("The gripper action timed-out");
        return false;
    }
}

void jaco_trajectory::spherical_grip(double diameter){
    //Joint values found from palm to mid-finger
	double jointValue = (M_PI / 4.0) - asin((diameter / 44.0) - (29.0 / 22.0));
    //ROS_INFO_STREAM(jointValue);
	gripper_group_->setJointValueTarget("j2n6s300_joint_finger_1", jointValue);
	gripper_group_->setJointValueTarget("j2n6s300_joint_finger_2", jointValue);
	gripper_group_->setJointValueTarget("j2n6s300_joint_finger_3", jointValue);

}

void jaco_trajectory::pinch_grip(double diameter){
    //Joint values found from palm to end of finger
	double jointValue = (M_PI / 4.0) - asin((diameter / 87.0) - (2.0 / 3.0));
    //ROS_INFO_STREAM(jointValue);
	gripper_group_->setJointValueTarget("j2n6s300_joint_finger_1", jointValue);
	gripper_group_->setJointValueTarget("j2n6s300_joint_finger_2", jointValue);
	//gripper_group_->setJointValueTarget("j2n6s300_joint_finger_3", jointValue);

}

void jaco_trajectory::tripod_grip(double diameter){
    //Joint values found from palm to mid end of finger (Little before pinch)
	double jointValue = (M_PI / 4.0) - asin((diameter / 77.0) - (58.0 / 77.0));
    ROS_INFO_STREAM(diameter);
    ROS_INFO("TRIPOD");
	gripper_group_->setJointValueTarget("j2n6s300_joint_finger_1", jointValue);
	gripper_group_->setJointValueTarget("j2n6s300_joint_finger_2", jointValue);
	gripper_group_->setJointValueTarget("j2n6s300_joint_finger_3", jointValue);

}

void jaco_trajectory::pickup_object(){
    //once the object is know to be grasped
    //we remove obstacle from work scene
    co_.id = "cylinder";
    co_.operation = moveit_msgs::CollisionObject::REMOVE;
    pub_co_.publish(co_);

    //and then we declare it as an attached obstacle
    aco_.object.operation = moveit_msgs::CollisionObject::ADD;
    aco_.link_name = "j2n6s300_end_effector";
    aco_.touch_links.push_back("j2n6s300_end_effector");
    aco_.touch_links.push_back("j2n6s300_link_finger_1");
    aco_.touch_links.push_back("j2n6s300_link_finger_2");
    aco_.touch_links.push_back("j2n6s300_link_finger_3");
    aco_.touch_links.push_back("j2n6s300_link_finger_tip_1");
    aco_.touch_links.push_back("j2n6s300_link_finger_tip_2");
    aco_.touch_links.push_back("j2n6s300_link_finger_tip_3");
    pub_aco_.publish(aco_);
}

void jaco_trajectory::add_target()
{
    //remove target_cylinder
    co_.id = "cylinder";
    co_.operation = moveit_msgs::CollisionObject::REMOVE;
    pub_co_.publish(co_);

    //add target_cylinder
    co_.primitives.resize(1);
    co_.primitive_poses.resize(1);
    co_.primitives[0].type = shape_msgs::SolidPrimitive::CYLINDER;
    co_.primitives[0].dimensions.resize(geometric_shapes::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::CYLINDER>::value);
    co_.operation = moveit_msgs::CollisionObject::ADD;

    co_.primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT] = 0.13;
    co_.primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS] = 0.036;
    co_.primitive_poses[0].position.x = 0.7;
    co_.primitive_poses[0].position.y = 0.0;
    co_.primitive_poses[0].position.z = 0.13/2;
    pub_co_.publish(co_);
    planning_scene_msg_.world.collision_objects.push_back(co_);
    planning_scene_msg_.is_diff = true;
    pub_planning_scene_diff_.publish(planning_scene_msg_);
    aco_.object = co_;
    ros::WallDuration(0.1).sleep();
}

void jaco_trajectory::trajectory_plan(geometry_msgs::PoseStamped pose){
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    ROS_INFO("Starting");
    group_->clearPathConstraints();
    group_->setPoseTarget(pose);
    //group_->setNamedTarget("Home");
    
    //If plan succeded, then execute plan.
    bool success = (group_->plan(my_plan) == moveit_msgs::MoveItErrorCodes::SUCCESS);
    group_->execute(my_plan);
}

geometry_msgs::PoseStamped jaco_trajectory::generate_gripper_align_pose(geometry_msgs::Point targetpose_msg, double dist, double azimuth, double polar, double rot_gripper_z)
{
    geometry_msgs::PoseStamped pose_msg;
    //Set reference frame
    pose_msg.header.frame_id = "root";
    pose_msg.header.stamp = ros::Time::now();

    // computer pregrasp position w.r.t. location of grasp_pose in spherical coordinate. Orientation is w.r.t. fixed world (root) reference frame.
    double delta_x = -dist * cos(azimuth) * sin(polar);
    double delta_y = -dist * sin(azimuth) * sin(polar);
    double delta_z = -dist * cos(polar);

    // computer the orientation of gripper w.r.t. fixed world (root) reference frame. The gripper (z axis) should point(open) to the grasp_pose.
    tf::Quaternion q = EulerZYZ_to_Quaternion(azimuth, polar, rot_gripper_z);

    pose_msg.pose.position.x = targetpose_msg.x+ delta_x;
    pose_msg.pose.position.y = targetpose_msg.y+ delta_y;
    pose_msg.pose.position.z = targetpose_msg.z+ delta_z;
    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();
    pose_msg.pose.orientation.w = q.w();

    return pose_msg;
}

void jaco_trajectory::define_cartesian_pose()
{

    geometry_msgs::Point a;
    // Euler_ZYZ (-M_PI/4, M_PI/2, M_PI/2)
    a.x = 0.7;
    a.y = 0.0;
    a.z = 0.13/2;


    // generate_pregrasp_pose(double dist, double azimuth, double polar, double rot_gripper_z)
    grasp_pose_= generate_gripper_align_pose(a, 0.03999, 0, M_PI/2, M_PI/2);
    pregrasp_pose_ = generate_gripper_align_pose(a, 0.1, 0, M_PI/2, M_PI/2);
    postgrasp_pose_ = grasp_pose_;
    postgrasp_pose_.pose.position.z = grasp_pose_.pose.position.z + 0.05;
}

jaco_trajectory::~jaco_trajectory(){
    delete group_;
    delete gripper_group_;
}

// void jaco_trajectory::pos_callback(const jaco::obj_posConstPtr& msg){
//     for (int i = 0; i < sizeof(msg->object)/sizeof(msg->object[0]); i++)
//     {
//         obj_vec.push_back(msg->object[i]);
//         ROS_INFO("Vector for loop");
//         ROS_INFO_STREAM(obj_vec[i].radius);
//     }
    
    // ROS_INFO("pos_callback");
    // tf::Quaternion q; 

    // // define grasp pose
    // grasp_pose_.header.frame_id = "root";
    // grasp_pose_.header.stamp  = ros::Time::now();

    // // Euler_ZYZ (-M_PI/4, M_PI/2, M_PI/2) ---- WORKS
    // grasp_pose_.pose.position.x = msg->object[0].pos.x;
    // grasp_pose_.pose.position.y = msg->object[0].pos.y;
    // grasp_pose_.pose.position.z = msg->object[0].pos.z;

    // q = EulerZYZ_to_Quaternion(0, M_PI/2, M_PI/2);
    // //q = EulerZYZ_to_Quaternion(msg->orientation.x, msg->orientation.y, msg->orientation.z);
    // grasp_pose_.pose.orientation.x = q.x();
    // grasp_pose_.pose.orientation.y = q.y();
    // grasp_pose_.pose.orientation.z = q.z();
    // grasp_pose_.pose.orientation.w = q.w();

    // //ROS_INFO_STREAM(grasp_pose_.pose.orientation.z); // x, y, and z = 0.5

    // // generate_pregrasp_pose(double dist, double azimuth, double polar, double rot_gripper_z)
    // grasp_pose_= generate_gripper_align_pose(grasp_pose_, 0.03999, 0, M_PI/2, M_PI/2);
    // pregrasp_pose_ = generate_gripper_align_pose(grasp_pose_, 0.1, 0, M_PI/2, M_PI/2);
    // postgrasp_pose_ = grasp_pose_;
    // postgrasp_pose_.pose.position.z = grasp_pose_.pose.position.z + 0.05;

    // trajectory_plan(pregrasp_pose_);
    // trajectory_plan(grasp_pose_);
    // group_->setEndEffectorLink("j2n6s300_end_effector");
    // gripper_action(msg->object[0].radius);

//}

void jaco_trajectory::vision_data_callback(const vision::Detection_arrayConstPtr &msg){
    for (size_t i = 0; i < sizeof(msg->msg)/sizeof(msg->msg[0]); i++)
    {
        vision::Detection DetectionData;
        DetectionData = msg->msg[i];
        visionDataArray.msg.push_back(DetectionData);
    }
};

shapefitting::shape_data jaco_trajectory::get_shape_data(vision::Detection DetectionData){
shapefitting::shapefitting_positionGoal goal;
goal.input = DetectionData;
shape_data_client.sendGoal(goal);
shape_data_client.waitForResult(ros::Duration(2.0));

//Broadcast for new frames
tf2_ros::TransformBroadcaster tfb;
geometry_msgs::TransformStamped Transform_camera, Transform_obj;

    if (shape_data_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        ROS_INFO("SUCCESS");
        shapefitting::shape_data tf_Cam_Obj = shape_data_client.getResult()->object;

        //tf from end effector to camera
        Transform_camera.header.frame_id = "j2n6s300_end_effector";
        Transform_camera.child_frame_id = "Realsense";
        Transform_camera.transform.translation.x = 0.102; //Mulig fortegnsændring
        Transform_camera.transform.translation.y = 0;
        Transform_camera.transform.translation.z = -0.182;
        tf2::Quaternion q;
            q.setRPY(0, 0, M_PI/2);
        Transform_camera.transform.rotation.x = q.x();
        Transform_camera.transform.rotation.y = q.y();
        Transform_camera.transform.rotation.z = q.z();
        Transform_camera.transform.rotation.w = q.w();

        //tf from camera to object
        Transform_obj.header.frame_id = "Realsense";
        Transform_obj.child_frame_id = "ObjectOfInterest";
        Transform_obj.transform.translation.x = tf_Cam_Obj.pos.x;
        Transform_obj.transform.translation.y = tf_Cam_Obj.pos.y;
        Transform_obj.transform.translation.z = tf_Cam_Obj.pos.z;
        // tf2::Quaternion q;
        //     q.setRPY(0,0,0);
        // Transform_obj.transform.rotation.x = q.x();
        // Transform_obj.transform.rotation.y = q.y();
        // Transform_obj.transform.rotation.z = q.z();
        // Transform_obj.transform.rotation.w = q.w();
      
        //Change tf_Cam_Obj coordinates to world reference frame
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);

        geometry_msgs::TransformStamped object_Transform = tfBuffer.lookupTransform("world", "ObjectOfInterest",
                                                                    ros::Time::now(),ros::Duration(3.0));

        shapefitting::shape_data tf_World_Obj = tf_Cam_Obj;
        tf_World_Obj.pos.x = object_Transform.transform.translation.x;
        tf_World_Obj.pos.y = object_Transform.transform.translation.y;
        tf_World_Obj.pos.z = object_Transform.transform.translation.z;
        
        return tf_World_Obj;
        //return shape_data_client.getResult()->object;
    }
}

void jaco_trajectory::connect_itongue(){
    //Initiate connection with iTongue
    while (itongue_start_pub.getNumSubscribers() < 1 )
    {
        ROS_INFO("VENT");
    }
    
    ROS_INFO("her");
    jaco::sys_msg data;
    data.start_tci = 1;
    data.InHand = 1;
    data.ContactThreshold = 0.12;
    data.CuttingThreshold = 0.08;
    data.tci_port = "/dev/ttyUSB0";
    itongue_start_pub.publish(data);

}

void jaco_trajectory::itongue_callback(const jaco::RAWItongueOutConstPtr& msg){
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    geometry_msgs::PoseStamped currentpose;

    //Count amount of same sensor number
    if (old_Sensor == msg->Sensor)
    {
        Sensor_count ++;
    }
    else Sensor_count = 0;
    old_Sensor = msg->Sensor;

    // ROS_INFO_STREAM(msg->Sensor);
    // ROS_INFO_STREAM(current_robot_transformStamped.transform.translation.x );
    //ROS_INFO_STREAM(currentpose.pose.position.x);
    //ROS_INFO_STREAM(current_robot_transformStamped.transform.translation.y );
    //ROS_INFO_STREAM(currentpose.pose.position.y);
    //ROS_INFO_STREAM(current_robot_transformStamped.transform.translation.z );
    //ROS_INFO_STREAM(currentpose.pose.position.z);

    //To prevent noise in data, sensor count must be above 4.
    if(current_robot_transformStamped.transform.translation.x != 0 && Sensor_count > 4){
        currentpose.header = current_robot_transformStamped.header;
        currentpose.pose.orientation = current_robot_transformStamped.transform.rotation;
        currentpose.pose.position.x = current_robot_transformStamped.transform.translation.x;
        currentpose.pose.position.y = current_robot_transformStamped.transform.translation.y;
        currentpose.pose.position.z = current_robot_transformStamped.transform.translation.z;
        //Switch statement to move robot in relation to sensor
        switch (msg->Sensor)
        {
        case 17: //Z forwards  - away from oneself
        ROS_INFO("Z forwards  - away from oneself");
            currentpose.pose.position.z = currentpose.pose.position.z - 0.05;
            break;
        case 12: //Z backwards -- towards oneself
            currentpose.pose.position.z = currentpose.pose.position.z + 0.05;
            break; 
        case 11: // cross up-left
        ROS_INFO("cross up-left");
            currentpose.pose.position.y = currentpose.pose.position.y + 0.05;
            currentpose.pose.position.x = currentpose.pose.position.x - 0.05;
            break;
        case 8:// Y upwards
        ROS_INFO("Y upwards");
            currentpose.pose.position.y = currentpose.pose.position.y + 0.05;
            break;
        case 13: // Cross up-right
        ROS_INFO("Cross up-right");
            currentpose.pose.position.y = currentpose.pose.position.y + 0.05;
            currentpose.pose.position.x = currentpose.pose.position.x + 0.05;
            break;
        case 14: //x left
        ROS_INFO("x left");
            currentpose.pose.position.x = currentpose.pose.position.x - 0.05;
            break;
        case 15: //x right
        ROS_INFO("x right");
            currentpose.pose.position.x = currentpose.pose.position.x + 0.05;
            break;
        case 16: // Cross down-left
            currentpose.pose.position.y = currentpose.pose.position.y - 0.05;
            currentpose.pose.position.x = currentpose.pose.position.x - 0.05;
            break;
        case 9: // y downwards
            currentpose.pose.position.y = currentpose.pose.position.y - 0.05;
            break;
        case 18: // Cross down-right
            currentpose.pose.position.y = currentpose.pose.position.y - 0.05;
            currentpose.pose.position.x = currentpose.pose.position.x + 0.05;
            break;

        default:
            break;
        }
        // ROS_INFO_STREAM(currentpose.pose.position.x);
        // ROS_INFO_STREAM(currentpose.pose.position.y);
        // ROS_INFO_STREAM(currentpose.pose.position.z);
        group_->setPoseTarget(currentpose);
        group_->move();
        bool success = (group_->plan(my_plan) == moveit_msgs::MoveItErrorCodes::SUCCESS);
        group_->execute(my_plan);
    }
}

void jaco_trajectory::IF_full_auto_execute(const jaco::IF_fullAutoGoalConstPtr &goal){
    ROS_INFO("Action recieved");
    ROS_INFO_STREAM(goal->goalObject.X1);
    interface_result_.success = true;
    
    //Get position and shape of object
    shapeData = get_shape_data(goal->goalObject);
    
    //Check if object is a cylinder
    if (1==1)
    {
        pregrasp_pose_ = generate_gripper_align_pose(shapeData.pos, 0.1, 0, M_PI/2, M_PI/2);
        grasp_pose_= generate_gripper_align_pose(shapeData.pos, 0.03999, 0, M_PI/2, M_PI/2);
        //Generate and execute pregrasp trajectory
        trajectory_plan(pregrasp_pose_);
        //Generate and execute grasp trajectory
        trajectory_plan(grasp_pose_);
        //Close gripper
        spherical_grip(shapeData.radius*2);
    }
    

    interface_as_.setSucceeded(interface_result_);
};


jaco_trajectory::jaco_trajectory(ros::NodeHandle &nh): 
    nh_(nh),
    shape_data_client("get_shape",true),
    interface_as_(nh_, "IF_full_auto", boost::bind(&jaco_trajectory::IF_full_auto_execute, this, _1), false)
{
    interface_as_.start();
    
    planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));
    nh_.param<bool>("/robot_connected",robot_connected_,true);

    group_ = new moveit::planning_interface::MoveGroupInterface("arm");
    gripper_group_ = new moveit::planning_interface::MoveGroupInterface("gripper");

        /// Functions below are replaced or moved to the pos_callback function
    define_cartesian_pose();
    //  trajectory_plan(pregrasp_pose_);
    // trajectory_plan(grasp_pose_);
    group_->setEndEffectorLink("j2n6s300_end_effector"); //robot_type_ + "_end_effector" <---

    //  finger_client_ = new actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction>
    //          ("/j2n6s300_driver/fingers_action/finger_positions", false);
    //  while(robot_connected_ && !finger_client_->waitForServer(ros::Duration(5.0))){
    //    ROS_INFO("Waiting for the finger action server to come up");
    //  }moveit::planning_interface::MoveGroupInterface
    
    pub_aco_ = nh_.advertise<moveit_msgs::AttachedCollisionObject>("/attached_collision_object", 10);
    pub_co_ = nh_.advertise<moveit_msgs::CollisionObject>("/collision_object", 10);
    pub_planning_scene_diff_ = nh_.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    itongue_sub_ = nh.subscribe<jaco::RAWItongueOut>("/RAWItongueOut", 1, &jaco_trajectory::itongue_callback,this);

    // pos_sub = nh.subscribe<jaco::obj_pos>("/obj_pos", 1000, &jaco_trajectory::pos_callback, this); //EMIL
    itongue_start_pub = nh_.advertise<jaco::sys_msg>("/Sys_cmd",1);
    vision_data_sub = nh.subscribe<vision::Detection_array>("/Vision/ObjectDetection",1000,&jaco_trajectory::vision_data_callback,this);

    connect_itongue();

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    ros::Rate rate(10.0);
    while (nh_.ok()){
        //Publishes frame
        // Transform_camera.header.stamp = ros::Time::now();
        // tfb.sendTransform(Transform_camera);
     
     try{
       
       current_robot_transformStamped = tfBuffer.lookupTransform("world", "j2n6s300_end_effector",
                                ros::Time::now(),ros::Duration(3.0));
     }
     catch (tf2::TransformException &ex) {
       ROS_WARN("%s",ex.what());
       ros::Duration(1.0).sleep();
       continue;
     }
   }
}


void testemil(){
    ROS_INFO("BEGYNDT");
    vision::Detection DetectionData;
    DetectionData.Class = 1;
    DetectionData.X1 = 100;
    DetectionData.X2 = 200;
    DetectionData.Y1 = 100;
    DetectionData.Y2 = 200;

    shapefitting::shapefitting_positionGoal goal;
    goal.input = DetectionData;
    actionlib::SimpleActionClient<shapefitting::shapefitting_positionAction> shape_data_client("get_shape",true);
     ROS_INFO("1");
    shape_data_client.waitForServer();
     ROS_INFO("2");
    shape_data_client.sendGoal(goal);
    shape_data_client.waitForResult(ros::Duration(2.0));
    if (shape_data_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        ROS_INFO("SUCCESS");
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test");
    ros::NodeHandle node;

    ros::AsyncSpinner spinner(4);
    spinner.start();

    jaco_trajectory Jaco(node);
      //  testemil();
    //ros::spin();
    ros::waitForShutdown();
    return 0;
}
