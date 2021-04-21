#include <ros/ros.h>
#include <jaco_control.h>

#include <actionlib/client/simple_action_client.h>
#include <tf_conversions/tf_eigen.h>




const double FINGER_MAX = 6400;
std::string Camera_name;

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

// Værdier skal justeres
bool jaco_control::gripper_action(double finger_turn){
 //moveit::planning_interface::MoveGroupInterface::Plan my_plan;
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
        
        //bool success = (gripper_group_->plan(my_plan) == moveit_msgs::MoveItErrorCodes::SUCCESS);
            //gripper_group_->move();
        //gripper_group_->execute(my_plan);
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

// Værdier skal justeres
void jaco_control::spherical_grip(double diameter){
	//Joint values found from palm to mid-finger
	double jointValue = (M_PI / 4.0) - asin((diameter / 44.0) - (29.0 / 22.0));
	//ROS_INFO_STREAM(jointValue);

    //Send jointvalue to all fingers:
    finger_goal.fingers.finger1 = jointValue;
    finger_goal.fingers.finger2 = finger_goal.fingers.finger1;
    finger_goal.fingers.finger3 = finger_goal.fingers.finger1;
    finger_client_->sendGoal(finger_goal);

    if (finger_client_->waitForResult(ros::Duration(5.0))){
        finger_client_->getResult();
    } else {
        finger_client_->cancelAllGoals();
        ROS_WARN_STREAM("The gripper action timed-out");
    }

	//gripper_group_->setJointValueTarget("j2n6s300_joint_finger_1", jointValue);
	//gripper_group_->setJointValueTarget("j2n6s300_joint_finger_2", jointValue);
	//gripper_group_->setJointValueTarget("j2n6s300_joint_finger_3", jointValue);

}

// Værdier skal justeres
void jaco_control::pinch_grip(double diameter){
    //Joint values found from palm to end of finger
	double jointValue = (M_PI / 4.0) - asin((diameter / 87.0) - (2.0 / 3.0));
    //ROS_INFO_STREAM(jointValue);

    //Send jointvalue to all fingers:
    finger_goal.fingers.finger1 = jointValue;
    finger_goal.fingers.finger2 = finger_goal.fingers.finger1;
    finger_client_->sendGoal(finger_goal);

    if (finger_client_->waitForResult(ros::Duration(5.0))){
        finger_client_->getResult();
    } else {
        finger_client_->cancelAllGoals();
        ROS_WARN_STREAM("The gripper action timed-out");
    }

	//gripper_group_->setJointValueTarget("j2n6s300_joint_finger_1", jointValue);
	//gripper_group_->setJointValueTarget("j2n6s300_joint_finger_2", jointValue);

}

// Værdier skal justeres
void jaco_control::tripod_grip(double diameter){
    //Joint values found from palm to mid end of finger (Little before pinch)
	double jointValue = (M_PI / 4.0) - asin((diameter / 77.0) - (58.0 / 77.0));
    ROS_INFO_STREAM(diameter);
    ROS_INFO("TRIPOD");

    //Send jointvalue to all fingers:
    finger_goal.fingers.finger1 = jointValue;
    finger_goal.fingers.finger2 = finger_goal.fingers.finger1;
    finger_goal.fingers.finger3 = finger_goal.fingers.finger1;
    finger_client_->sendGoal(finger_goal);

    if (finger_client_->waitForResult(ros::Duration(5.0))){
        finger_client_->getResult();
    } else {
        finger_client_->cancelAllGoals();
        ROS_WARN_STREAM("The gripper action timed-out");
    }

	//gripper_group_->setJointValueTarget("j2n6s300_joint_finger_1", jointValue);
	//gripper_group_->setJointValueTarget("j2n6s300_joint_finger_2", jointValue);
	//gripper_group_->setJointValueTarget("j2n6s300_joint_finger_3", jointValue);

}

// Til simulering skal slettes
void jaco_control::pickup_object(){
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

// Til simulering skal slettes
void jaco_control::add_target()
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


// DONE
kinova::KinovaPose jaco_control::generate_gripper_align_pose(geometry_msgs::Point targetpose_msg, double dist, double azimuth, double polar, double rot_gripper_z)
{
    kinova::KinovaPose pose_msg;
    
    // computer pregrasp position w.r.t. location of grasp_pose in spherical coordinate. Orientation is w.r.t. fixed world (root) reference frame.
    double delta_x = -dist * cos(azimuth) * sin(polar);
    double delta_y = -dist * sin(azimuth) * sin(polar);
    double delta_z = -dist * cos(polar);

    // computer the orientation of gripper w.r.t. fixed world (root) reference frame. The gripper (z axis) should point(open) to the grasp_pose.
    tf::Quaternion q = EulerZYZ_to_Quaternion(azimuth, polar, rot_gripper_z);

    pose_msg.X = targetpose_msg.x + delta_x;
    pose_msg.Y = targetpose_msg.y+ delta_y;
    pose_msg.Z = targetpose_msg.z+ delta_z;
    //XYZ euler angles
    pose_msg.ThetaX = azimuth;
    pose_msg.ThetaY = polar;
    pose_msg.ThetaZ = rot_gripper_z;
    
    return pose_msg;
}


// DONE
void jaco_control::define_cartesian_pose()
{
    geometry_msgs::Point a;
    // Euler_ZYZ (-M_PI/4, M_PI/2, M_PI/2)
    a.x = 0.7;
    a.y = 0.0;
    a.z = 0.13/2;
    // generate_pregrasp_pose(double dist, double azimuth, double polar, double rot_gripper_z)
    grasp_pose_= generate_gripper_align_pose(a, 0.03999, 0, M_PI/2, M_PI/2);
    kinova_comm.setCartesianPosition(pregrasp_pose_);
    pregrasp_pose_ = generate_gripper_align_pose(a, 0.1, 0, M_PI/2, M_PI/2);
    kinova_comm.setCartesianPosition(grasp_pose_);
    //postgrasp_pose_ = grasp_pose_;
    //postgrasp_pose_.pose.position.z = grasp_pose_.pose.position.z + 0.05;
}

jaco_control::~jaco_control(){
    delete group_;
    delete gripper_group_;
}

// void jaco_control::pos_callback(const jaco::obj_posConstPtr& msg){
//     for (int i = 0; i < sizeof(msg->object)/sizeof(msg->object[0]); i++)
//     {
//         obj_vec.push_back(msg->object[i]);
//         ROS_INFO("Vector for loop");
//         ROS_INFO_STREAM(obj_vec[i].radius);
//     }
//     ROS_INFO("pos_callback");
//     tf::Quaternion q; 
//     // define grasp pose
//     grasp_pose_.header.frame_id = "root";
//     grasp_pose_.header.stamp  = ros::Time::now();
//     // Euler_ZYZ (-M_PI/4, M_PI/2, M_PI/2) ---- WORKS
//     grasp_pose_.pose.position.x = msg->object[0].pos.x;
//     grasp_pose_.pose.position.y = msg->object[0].pos.y;
//     grasp_pose_.pose.position.z = msg->object[0].pos.z;
//     q = EulerZYZ_to_Quaternion(0, M_PI/2, M_PI/2);
//     //q = EulerZYZ_to_Quaternion(msg->orientation.x, msg->orientation.y, msg->orientation.z);
//     grasp_pose_.pose.orientation.x = q.x();
//     grasp_pose_.pose.orientation.y = q.y();
//     grasp_pose_.pose.orientation.z = q.z();
//     grasp_pose_.pose.orientation.w = q.w();
//     //ROS_INFO_STREAM(grasp_pose_.pose.orientation.z); // x, y, and z = 0.5
//     // generate_pregrasp_pose(double dist, double azimuth, double polar, double rot_gripper_z)
//     grasp_pose_= generate_gripper_align_pose(grasp_pose_, 0.03999, 0, M_PI/2, M_PI/2);
//     pregrasp_pose_ = generate_gripper_align_pose(grasp_pose_, 0.1, 0, M_PI/2, M_PI/2);
//     postgrasp_pose_ = grasp_pose_;
//     postgrasp_pose_.pose.position.z = grasp_pose_.pose.position.z + 0.05;
//     trajectory_plan(pregrasp_pose_);
//     trajectory_plan(grasp_pose_);
//     group_->setEndEffectorLink("j2n6s300_end_effector");
//     gripper_action(msg->object[0].radius);
// }

// DONE
void jaco_control::vision_data_callback(const vision::Detection_arrayConstPtr &msg){
    for (size_t i = 0; i < sizeof(msg->msg)/sizeof(msg->msg[0]); i++)
    {
        vision::Detection DetectionData;
        DetectionData = msg->msg[i];
        visionDataArray.msg.push_back(DetectionData);
    }
};

// DONE
shapefitting::shape_data jaco_control::get_shape_data(vision::Detection DetectionData){
    shapefitting::shapefitting_positionGoal goal;
    goal.input = DetectionData;
    shape_data_client.sendGoal(goal);
    shape_data_client.waitForResult(ros::Duration(2.0));
    ROS_INFO("Get shape data function");

    if (shape_data_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        ROS_INFO("SUCCESS");
        shape_data_client.getResult()->object;
        //shapefitting::shape_data tf_Cam_Obj = shape_data_client.getResult()->object;

        //Initialize result  
        shapefitting::shapefitting_positionActionResult result;
        result.result.object = tf_Cam_Obj;
        
        //Set up frames:
        //tf from end effector to camera
        Transform_camera.header.stamp = ros::Time::now();
        Transform_camera.header.frame_id = "j2n6s300_end_effector";
        Transform_camera.child_frame_id = "Realsense_Camera";
        Transform_camera.transform.translation.x = 0.102; //Mulig fortegnsændring
        Transform_camera.transform.translation.y = 0;
        Transform_camera.transform.translation.z = -0.182;
        tf2::Quaternion q1;
            q1.setRPY(-0.26, 0, M_PI/2);
        Transform_camera.transform.rotation.x = q1.x();
        Transform_camera.transform.rotation.y = q1.y();
        Transform_camera.transform.rotation.z = q1.z();
        Transform_camera.transform.rotation.w = q1.w();

        //tf from camera to object
        Transform_obj.header.stamp = ros::Time::now();
        Transform_obj.header.frame_id = "Realsense_Camera";
        Transform_obj.child_frame_id = tf_Cam_Obj.object_class.data;
        Transform_obj.transform.translation.x = tf_Cam_Obj.pos.x;
        Transform_obj.transform.translation.y = tf_Cam_Obj.pos.y;
        Transform_obj.transform.translation.z = tf_Cam_Obj.pos.z;
        tf2::Quaternion q2;
            //q2.setRPY(0,0,0);
            q2.setRPY(tf_Cam_Obj.orientation.x, tf_Cam_Obj.orientation.y, tf_Cam_Obj.orientation.z);
        Transform_obj.transform.rotation.x = q2.x();
        Transform_obj.transform.rotation.y = q2.y();
        Transform_obj.transform.rotation.z = q2.z();
        Transform_obj.transform.rotation.w = q2.w();
 
        //From world to object data. tf from world to object is found by listener function.
        tf_World_Obj.result.object = tf_Cam_Obj;
        tf_World_Obj.result.object.pos.x = obj_ee_transformStamped.transform.translation.x;
        tf_World_Obj.result.object.pos.y = obj_ee_transformStamped.transform.translation.y;
        tf_World_Obj.result.object.pos.z = obj_ee_transformStamped.transform.translation.z;
        tf_World_Obj.result.object.orientation.x = obj_ee_transformStamped.transform.rotation.x;
        tf_World_Obj.result.object.orientation.y = obj_ee_transformStamped.transform.rotation.y;
        tf_World_Obj.result.object.orientation.z = obj_ee_transformStamped.transform.rotation.z;       

        return tf_World_Obj.result.object;
        //return shape_data_client.getResult()->object;
    }
}

// DONE
void jaco_control::connect_itongue(){
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

// DONE
void jaco_control::itongue_callback(const jaco::RAWItongueOutConstPtr& msg){

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
    kinova::KinovaPose velocity;
    velocity.ThetaX = 0;
    velocity.ThetaY = 0;
    velocity.ThetaZ = 0;
    velocity.X = 0;
    velocity.Y = 0;
    velocity.Z = 0;
    //To prevent noise in data, sensor count must be above 4.
    if(Sensor_count > 4){

        //Switch statement to move robot in relation to sensor
        switch (msg->Sensor)
        {
        case 17: //Z forwards  - away from oneself
        ROS_INFO("Z forwards  - away from oneself");
            velocity.Z = -0.05;
            break;
        case 12: //Z backwards -- towards oneself
            velocity.Z = 0.05;
            break; 
        case 11: // cross up-left
        ROS_INFO("cross up-left");
            velocity.Y  = 0.05;
            velocity.X  = -0.05;
            break;
        case 8:// Y upwards
        ROS_INFO("Y upwards");
            velocity.Y = 0.05;
            break;
        case 13: // Cross up-right
        ROS_INFO("Cross up-right");
            velocity.Y = 0.05;
            velocity.X = 0.05;
            break;
        case 14: //x left
        ROS_INFO("x left");
            velocity.X  = -0.05;
            break;
        case 15: //x right
        ROS_INFO("x right");
            velocity.X = 0.05;
            break;
        case 16: // Cross down-left
            velocity.Y = -0.05;
            velocity.X = -0.05;
            break;
        case 9: // y downwards
            velocity.Y = -0.05;
            break;
        case 18: // Cross down-right
            velocity.Y = -0.05;
            velocity.X = 0.05;
            break;

        default:
            break;
        }
        // ROS_INFO_STREAM(currentpose.pose.position.x);
        // ROS_INFO_STREAM(currentpose.pose.position.y);
        // ROS_INFO_STREAM(currentpose.pose.position.z);
        // group_->setPoseTarget(currentpose);
        // group_->move();
        // bool success = (group_->plan(my_plan) == moveit_msgs::MoveItErrorCodes::SUCCESS);
        // group_->execute(my_plan);
        kinova_comm.setCartesianVelocities(velocity);
    } else
    {
        // Set velocity to zero if no command is recived.
        velocity.X = 0;
        velocity.Y = 0;
        velocity.Z = 0;
        kinova_comm.setCartesianVelocities(velocity);
    }
    
}

// Skal konverteres
void jaco_control::IF_full_auto_execute(const jaco::IF_fullAutoGoalConstPtr &goal){
    ROS_INFO("Full-auto Action received");
    interface_result_.success = true;
    
    //Get position and shape of object by calling get_shape_data with goalObject.
    shapeData = get_shape_data(goal->goalObject);
    ROS_INFO("Shapedata received");

    //Check if object is a cylinder. Use shapeData to 

    if (1==1)
    {
        //Calculate grasp positions. Must be KinovaPose type.
        pregrasp_pose_ = generate_gripper_align_pose(shapeData.pos, 0.1, 0, M_PI/2, M_PI/2);
        grasp_pose_= generate_gripper_align_pose(shapeData.pos, 0.03999, 0, M_PI/2, M_PI/2);
        //Generate and execute pregrasp trajectory

		kinova_comm.setCartesianPosition(pregrasp_pose_, 0, false);
        
        kinova_comm.setCartesianPosition(grasp_pose_, 0, false);
		//kinova_api_.sendBasicTrajectory(pregrasp_pose_);

		//trajectory_plan(pregrasp_pose_); //

        //Generate and execute grasp trajectory

		spherical_grip(shapeData.radius*2);
	}
    interface_as_.setSucceeded(interface_result_);
};

jaco_control::jaco_control(ros::NodeHandle &nh): 
    nh_(nh),
    shape_data_client("get_shape",true),
    interface_as_(nh_, "IF_full_auto", boost::bind(&jaco_control::IF_full_auto_execute, this, _1), false),
    kinova_comm(nh_,mutexer,true,"j2n6s300")
{
    interface_as_.start();
    
    planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));
    nh_.param<bool>("/robot_connected",robot_connected_,true);

    //group_ = new moveit::planning_interface::MoveGroupInterface("arm");
    //gripper_group_ = new moveit::planning_interface::MoveGroupInterface("gripper");

        /// Functions below are replaced or moved to the pos_callback function
    //define_cartesian_pose();
    //  trajectory_plan(pregrasp_pose_);
    // trajectory_plan(grasp_pose_);
    
    //group_->setEndEffectorLink("j2n6s300_end_effector"); //robot_type_ + "_end_effector" <---
    //kinova::KinovaPose::CartesianInfo a;
    //kinova_comm.setCartesianPosition
    
    //kinova_comm.setCartesianVelocities()
     finger_client_ = new actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction>
             ("/j2n6s300_driver/fingers_action/finger_positions", false);
     while(robot_connected_ && !finger_client_->waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the finger action server to come up");
      }
        //moveit::planning_interface::MoveGroupInterface

    pub_aco_ = nh_.advertise<moveit_msgs::AttachedCollisionObject>("/attached_collision_object", 10);
    pub_co_ = nh_.advertise<moveit_msgs::CollisionObject>("/collision_object", 10);
    pub_planning_scene_diff_ = nh_.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    itongue_sub_ = nh.subscribe<jaco::RAWItongueOut>("/RAWItongueOut", 1, &jaco_control::itongue_callback,this);

    // pos_sub = nh.subscribe<jaco::obj_pos>("/obj_pos", 1000, &jaco_control::pos_callback, this); //EMIL
    itongue_start_pub = nh_.advertise<jaco::sys_msg>("/Sys_cmd",1);
    vision_data_sub = nh.subscribe<vision::Detection_array>("/Vision/ObjectDetection",1000,&jaco_control::vision_data_callback,this);

    //connect_itongue();

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    ros::Rate rate(10.0);


    
    

    while (nh_.ok()){      
        //testemil();      

        try{    
            //Send tf:
            static_broadcaster.sendTransform(Transform_camera);
            br.sendTransform(Transform_obj);  
            obj_ee_transformStamped = tfBuffer.lookupTransform("world", tf_Cam_Obj.object_class.data,
                                    ros::Time(0),ros::Duration(3.0));

            current_robot_transformStamped = tfBuffer.lookupTransform("world", "j2n6s300_end_effector",
                                    ros::Time(0),ros::Duration(3.0));
        }
        catch (tf2::TransformException &ex) {
             //First call might not pass through, as the frame may not have existed, when the transform is requested the transform may not exist yet and fails the first time.
             //After the first transform all the transforms exist and the transforms should work.
            //ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
    }
}
    
    
void jaco_control::testemil(){
    ROS_INFO("BEGYNDT");
    // vision::Detection DetectionData;
    // DetectionData.Class = 1;
    // DetectionData.X1 = 100;
    // DetectionData.X2 = 200;
    // DetectionData.Y1 = 100;
    // DetectionData.Y2 = 200;

    // shapefitting::shape_data test = jaco_trajectory::get_shape_data(DetectionData);

        
        tf_Cam_Obj.object_class.data = "Vin"; 
        tf_Cam_Obj.pos.x = 0.6;
        tf_Cam_Obj.pos.y = 0.6;
        tf_Cam_Obj.pos.z = 0.6;
        tf_Cam_Obj.orientation.x = -1.2;
        tf_Cam_Obj.orientation.y = 0;
        tf_Cam_Obj.orientation.z = 2.3;
        

        Transform_camera.header.stamp = ros::Time::now();
        Transform_camera.header.frame_id = "j2n6s300_end_effector";
        Transform_camera.child_frame_id = "Realsense_Camera";
        Transform_camera.transform.translation.x = 0.102; //Mulig fortegnsændring
        Transform_camera.transform.translation.y = 0;
        Transform_camera.transform.translation.z = -0.182;
        tf2::Quaternion q1;
            q1.setRPY(-0.26, 0, M_PI/2);
        Transform_camera.transform.rotation.x = q1.x();
        Transform_camera.transform.rotation.y = q1.y();
        Transform_camera.transform.rotation.z = q1.z();
        Transform_camera.transform.rotation.w = q1.w();

        //tf from camera to object
        Transform_obj.header.stamp = ros::Time::now();
        Transform_obj.header.frame_id = "Realsense_Camera";
        Transform_obj.child_frame_id = tf_Cam_Obj.object_class.data;
        Transform_obj.transform.translation.x = tf_Cam_Obj.pos.x;
        Transform_obj.transform.translation.y = tf_Cam_Obj.pos.y;
        Transform_obj.transform.translation.z = tf_Cam_Obj.pos.z;
        tf2::Quaternion q2;
            //q2.setRPY(0,0,0);
            q2.setRPY(tf_Cam_Obj.orientation.x, tf_Cam_Obj.orientation.y, tf_Cam_Obj.orientation.z);
        Transform_obj.transform.rotation.x = q2.x();
        Transform_obj.transform.rotation.y = q2.y();
        Transform_obj.transform.rotation.z = q2.z();
        Transform_obj.transform.rotation.w = q2.w();


    // shapefitting::shapefitting_positionGoal goal;
    // goal.input = DetectionData;
    // actionlib::SimpleActionClient<shapefitting::shapefitting_positionAction> shape_data_client("get_shape",true);
    //  ROS_INFO("1");
    // shape_data_client.waitForServer();
    //  ROS_INFO("2");
    // shape_data_client.sendGoal(goal);
    // shape_data_client.waitForResult(ros::Duration(2.0));
    // if (shape_data_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    //     ROS_INFO("SUCCESS");
    // }

    // shapefitting::shape_data Shape_Data_Test;
    // Shape_Data_Test.object_class.data = "Sodavand";
    // Shape_Data_Test.radius = 0.12;
    // Shape_Data_Test.pos.x = 0.2;
    // Shape_Data_Test.pos.y = -0.5;
    // Shape_Data_Test.pos.z = 1.3;
    // Shape_Data_Test.orientation.x = 0;
    // Shape_Data_Test.orientation.y = 1;
    // Shape_Data_Test.orientation.z = 0;

    // shapefitting::shapefitting_positionActionResult result;
    // result.result.object = Shape_Data_Test;
    
}


int main(int argc, char **argv)
{
    
    ros::init(argc, argv, "test");
    ros::NodeHandle node;

    ros::AsyncSpinner spinner(4);
    spinner.start();
    
    jaco_control Jaco(node);
    //ros::spin();
    ros::waitForShutdown();
    return 0;
}