#include <ros/ros.h>
#include <jaco_control.h>
//#include <customVector.h>
#include <actionlib/client/simple_action_client.h>
#include <tf_conversions/tf_eigen.h>

template <class T>
std::array<T,3> vector::dotProd(std::array<T,3> A,std::array<T,3> B){
    return A;
}

template <class T>
std::array<T,3> crossProd(std::array<T,3>,std::array<T,3>);


template <class T>
std::array<T,3> sub(std::array<T,3>,std::array<T,3>);

template <class T>
std::array<T,3> add(std::array<T,3>,std::array<T,3>);

template <class T>
std::array<T,3> vecProj(std::array<T,3>,std::array<T,3>);

template <class T>
std::array<T,3> pointToArray(geometry_msgs::Vector3,std::array<T,3>);

template <class T>
std::array<T,3> pointToArray(geometry_msgs::Point,std::array<T,3>);


bool debug_assistiveb = false;
void debug_assistive(std::string a){
if (debug_assistiveb)
{
    ROS_INFO_STREAM(a);
    std::array<double,3> A = {1,1,1};
    vector::dotProd(A,A);
}
}
bool debug_normalb = false;
void debug_normal(std::string a){
if (debug_normalb)
{
    ROS_INFO_STREAM(a);
}
}
bool debug_experimentalb = true;

void debug_experimental(std::string a){
if (debug_experimentalb)
    {
        ROS_INFO_STREAM(a);
    }
}

bool debug_itongueb = false;

void debug_itongue(std::string a){
    if (debug_itongueb)
    {
        ROS_INFO_STREAM(a);
    }
}




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

// double scaletest(double val, double max, double min){
//     double n = (val - min )/ (max-min);
//     return n;
// }


// Værdier skal justeres
void jaco_control::spherical_grip(double diameter){
	//Joint values found from palm to mid-finger
	// double jointValue = (M_PI / 4.0) - asin((diameter / 44.0) - (29.0 / 22.0));
	// //ROS_INFO_STREAM(jointValue);

    // //Send jointvalue to all fingers:
    // finger_goal.fingers.finger1 = jointValue;
    // finger_goal.fingers.finger2 = finger_goal.fingers.finger1;
    // finger_goal.fingers.finger3 = finger_goal.fingers.finger1;
    // finger_client_->sendGoal(finger_goal);

    // if (finger_client_->waitForResult(ros::Duration(5.0))){
    //     finger_client_->getResult();
    // } else {
    //     finger_client_->cancelAllGoals();
    //     ROS_WARN_STREAM("The gripper action timed-out");
    // }


    double fingerPercent = (diameter - 45) / (100-45);
    kinova::FingerAngles fingerAng;
    fingerAng.Finger1 = FINGER_MAX*fingerPercent;
    fingerAng.Finger2 = FINGER_MAX*fingerPercent;
    fingerAng.Finger3 = FINGER_MAX*fingerPercent;
    kinova_comm.printFingers(fingerAng);
    kinova_comm.setFingerPositions(fingerAng);
    

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
    if (robot_connected_)
    {
    //kinova_comm.setCartesianPosition(pregrasp_pose_);
    }
    pregrasp_pose_ = generate_gripper_align_pose(a, 0.1, 0, M_PI/2, M_PI/2);
    if (robot_connected_)
    {
    //kinova_comm.setCartesianPosition(grasp_pose_);
    }
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
    visionDataArray.msg.clear();
    if (msg->msg.size()>0)
    {
       for (size_t i = 0; i < sizeof(msg->msg)/sizeof(msg->msg[0]); i++)
        {
            vision::Detection DetectionData;
            DetectionData = msg->msg[i];
            visionDataArray.msg.push_back(DetectionData);
        }
    }
    
    
};

// DONE
shapefitting::shape_data jaco_control::get_shape_data(vision::Detection DetectionData){
    shapefitting::shapefitting_positionGoal goal;
    goal.input = DetectionData;
    shape_data_client.sendGoal(goal);
    shape_data_client.waitForResult(ros::Duration(2.0));
    ROS_INFO("Get shape data function1");

    if (shape_data_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        ROS_INFO("SUCCESS");
        //shape_data_client.getResult()->object;

        shapefitting::shape_data tf_Cam_Obj = shape_data_client.getResult()->object;
        //shapefitting::shape_data tf_Cam_Obj = shape_data_client.getResult()->object;



        //Initialize result  
        // shapefitting::shapefitting_positionActionResult result;
        // result.result.object = tf_Cam_Obj;
        
        //Set up frames:
        //tf from end effector to camera
        Transform_camera.header.stamp = ros::Time::now();
        Transform_camera.header.frame_id = "j2n7s300_end_effector";
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
        //ROS_INFO("VENT");
    }
    
    //ROS_INFO("her");
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

    kinova::KinovaPose velocity;
    geometry_msgs::Point velDir;
    velDir.x = 0;
    velDir.y = 0;
    velDir.z = 0;
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
        debug_itongue("Z forwards  - away from oneself");
            velocity.Z = -0.30;
            break;
        case 12: //Z backwards -- towards oneself
            velocity.Z = 0.30;
            break; 
        case 11: // cross up-left
        debug_itongue("cross up-left");
            velocity.Y  = 0.30;
            velocity.X  = -0.30;
            break;
        case 8:// Y upwards
        debug_itongue("Y upwards");
            velocity.Y = 0.30;
            break;
        case 13: // Cross up-right
        debug_itongue("Cross up-right");
            velocity.Y = 0.30;
            velocity.X = 0.30;
            break;
        case 14: //x left
        debug_itongue("x left");
            velocity.X  = -0.30;
            break;
        case 15: //x right
        debug_itongue("x right");
            velocity.X = 0.30;
            break;
        case 16: // Cross down-left
            velocity.Y = -0.30;
            velocity.X = -0.30;
            break;
        case 9: // y downwards
            velocity.Y = -0.30;
            break;
        case 18: // Cross down-right
            velocity.Y = -0.30;
            velocity.X = 0.30;
            break;
        case 1: //Twist wrist
            debug_itongue("twist 1");
            velocity.ThetaY = 0.7;
            break;
        case 2: //twist  wrist another way
            debug_itongue("Twist 2");
            velocity.ThetaY = -0.7;
            break;
        case 3: //twist  wrist another way
            debug_itongue("gripper");
            spherical_grip(100);
            break;
        case 4: //twist  wrist another way
            debug_itongue("gripper open");
            spherical_grip(45);
            break;
        default:
        return;
            break;
        }
        // ROS_INFO_STREAM(currentpose.pose.position.x)
        // ROS_INFO_STREAM(currentpose.pose.position.y);
        // ROS_INFO_STREAM(currentpose.pose.position.z);
        // group_->setPoseTarget(currentpose);
        // group_->move();
        // bool success = (group_->plan(my_plan) == moveit_msgs::MoveItErrorCodes::SUCCESS);
        // group_->execute(my_plan);
        kinova::KinovaPose ee_pose; //end effectpr åpse
        if (robot_connected_)
        {
            kinova_comm.getCartesianPosition(ee_pose);
            debug_normal("GAMLE VÆRDIER");
            geometry_msgs::Point newTraj;
            debug_normal(std::to_string(velocity.X));
            debug_normal(std::to_string(velocity.Y));
            debug_normal(std::to_string(velocity.Z));
            velDir.x = velocity.X;
            velDir.y = velocity.Y;
            velDir.z = velocity.Z;
            debug_experimental("Initial values : " + std::to_string(velocity.X) + " " + std::to_string(velocity.Y) + " " + std::to_string(velocity.Z) + " " + std::to_string(velocity.ThetaX) + " " + std::to_string(velocity.ThetaY) + " " + std::to_string(velocity.ThetaZ));
             if (!obj_ee_array.empty())
            {
                debug_normal(std::to_string(obj_ee_array.size()));
                newTraj = assistiveControl(velDir,obj_ee_array,current_robot_transformStamped);
                velocity.X = newTraj.x;
                velocity.Y = newTraj.y;
                velocity.Z = newTraj.z;
    
            }

            debug_experimental("Adjusted values : " + std::to_string(velocity.X) + " " + std::to_string(velocity.Y) + " " + std::to_string(velocity.Z) + " " + std::to_string(velocity.ThetaX) + " " + std::to_string(velocity.ThetaY) + " " + std::to_string(velocity.ThetaZ));
             kinova_comm.setCartesianVelocities(velocity);
            debug_normal("SEND  ");
            
        }
    } else
    {
        // Set velocity to zero if no command is recived.
        velocity.X = 0;
        velocity.Y = 0;
        velocity.Z = 0;
        if (robot_connected_)
        {
        //kinova_comm.setCartesianVelocities(velocity);
        }
    }
    
}

// Skal konverteres
void jaco_control::IF_full_auto_execute(const jaco::IF_fullAutoGoalConstPtr &goal){
    ROS_INFO("Full-auto Action received");
    interface_result_.success = true;
    
    //Get position and shape of object by calling get_shape_data with goalObject.
    //shapeData = get_shape_data(goal->goalObject);
    ROS_INFO("Shapedata received");

    //Check if object is a cylinder. Use shapeData to 

    if (1==1)
    {
        //Calculate grasp positions. Must be KinovaPose type.
        pregrasp_pose_ = generate_gripper_align_pose(shapeData.pos, 0.1, 0, M_PI/2, M_PI/2);
        grasp_pose_= generate_gripper_align_pose(shapeData.pos, 0.03999, 0, M_PI/2, M_PI/2);
        //Generate and execute pregrasp trajectory

        if (robot_connected_)
        {
            //kinova_comm.setCartesianPosition(pregrasp_pose_, 0, false);
        
            //kinova_comm.setCartesianPosition(grasp_pose_, 0, false);
        } else
        {
            ROS_INFO_STREAM("Robot not connected");
        }
        
        

		//kinova_api_.sendBasicTrajectory(pregrasp_pose_);

		//trajectory_plan(pregrasp_pose_); //

        //Generate and execute grasp trajectory

		spherical_grip(shapeData.radius*2);
	}
    interface_as_.setSucceeded(interface_result_);
};

void jaco_control::setCameraPos(){
        //tf from end effector to camera
        Transform_camera.header.stamp = ros::Time::now();
        Transform_camera.header.frame_id = "j2n6s300_end_effector";
        Transform_camera.child_frame_id = "Realsense_Camera";
        Transform_camera.transform.translation.x = 0; //Mulig fortegnsændring
        Transform_camera.transform.translation.y = 0.0967;
        Transform_camera.transform.translation.z = -0.1779;
        tf2::Quaternion q1;
            q1.setRPY(-0.21, 0, M_PI);
        Transform_camera.transform.rotation.x = q1.x();
        Transform_camera.transform.rotation.y = q1.y();
        Transform_camera.transform.rotation.z = q1.z();
        Transform_camera.transform.rotation.w = q1.w();
}

void jaco_control::shapefitting_activeCb(){
    debug_normal("Goal just went active");
}

void jaco_control::shapefitting_doneCb(const actionlib::SimpleClientGoalState& state, const shapefitting::shapefitting_simple_position_arrayResultConstPtr& result){
    // Clear previous object position data
    if (result->object[0].pos.x > -10)
    {
        tf_cam_to_object.clear();
    geometry_msgs::TransformStamped Transform_obj;
    debug_normal("shapefitting_doneCB");
    
    // Map every object to the camera, since the position is measured from the camera
    for (size_t i = 0; i < result->object.size(); i++)
    {
        debug_normal("shapefitting_doneCB_LOOP");
        //tf from camera to object
        Transform_obj.header.stamp = ros::Time::now();
        Transform_obj.header.frame_id = "Realsense_Camera";
        Transform_obj.child_frame_id = result->object[i].object_class.data + "object";
        Transform_obj.transform.translation.x = result->object[i].pos.x;
        Transform_obj.transform.translation.y = result->object[i].pos.y;
        Transform_obj.transform.translation.z = result->object[i].pos.z;
        tf2::Quaternion q2;
            //q2.setRPY(0,0,0);
        q2.setRPY(result->object[i].orientation.x, result->object[i].orientation.y, result->object[i].orientation.z);
        Transform_obj.transform.rotation.x = q2.x();
        Transform_obj.transform.rotation.y = q2.y();
        Transform_obj.transform.rotation.z = q2.z();
        Transform_obj.transform.rotation.w = q2.w();

        tf_cam_to_object.push_back(Transform_obj);
    }
    }
    
    
    
    
    
}

void jaco_control::doStuff(){}

typedef actionlib::SimpleActionClient<shapefitting::shapefitting_position_arrayAction> Client;

jaco_control::jaco_control(ros::NodeHandle &nh): 
    nh_(nh),
    shape_data_client("get_shape",true),
    interface_as_(nh_, "IF_full_auto", boost::bind(&jaco_control::IF_full_auto_execute, this, _1), false),
    kinova_comm(nh_,mutexer,true,"j2n6s300"),
    kinova_arm(kinova_comm, nh_, "j2n6s300", "j2n6s300"),
    pose_server(kinova_comm, nh_, "j2n6s300", "j2n6s300"),
    angles_server(kinova_comm, nh_),
    fingers_server(kinova_comm, nh_),
    joint_trajectory_controller(kinova_comm, nh_),
    shapefitting_ac("get_simple_shape_array",true)
{
    
    kinova_comm.startAPI();


   
    
   // interface_as_.start();
    ROS_INFO("Waiting for shapefitting action server to start.");
    //shapefitting_ac.waitForServer();
    ROS_INFO("ShapeFitting action server started");
    
    planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));
    //nh_.param<bool>("/robot_connected",robot_connected_,true);
    robot_connected_ = true;
    //group_ = new moveit::planning_interface::MoveGroupInterface("arm");
    //gripper_group_ = new moveit::planning_interface::MoveGroupInterface("gripper");

        /// Functions below are replaced or moved to the pos_callback function
    //define_cartesian_pose();
    //  trajectory_plan(pregrasp_pose_);
    // trajectory_plan(grasp_pose_);
    
    //group_->setEndEffectorLink("j2n6s300_end_effector"); //robot_type_ + "_end_effector" <---
    //kinova::KinovaPose::CartesianInfo a;

    //  finger_client_ = new actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction>
    //          ("/j2n6s300_driver/fingers_action/finger_positions", false);
    //  while(robot_connected_ && !finger_client_->waitForServer(ros::Duration(5.0))){
    //     ROS_INFO("Waiting for the finger action server to come up");
    //   }
        //moveit::planning_interface::MoveGroupInterface

    pub_aco_ = nh_.advertise<moveit_msgs::AttachedCollisionObject>("/attached_collision_object", 10);
    pub_co_ = nh_.advertise<moveit_msgs::CollisionObject>("/collision_object", 10);
    pub_planning_scene_diff_ = nh_.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    itongue_sub_ = nh.subscribe<jaco::RAWItongueOut>("/RAWItongueOut", 1, &jaco_control::itongue_callback,this);

    // pos_sub = nh.subscribe<jaco::obj_pos>("/obj_pos", 1000, &jaco_control::pos_callback, this); //EMIL
    itongue_start_pub = nh_.advertise<jaco::sys_msg>("/Sys_cmd",1);
    vision_data_sub = nh.subscribe<vision::Detection_array>("/Vision/ObjectDetection",1000,&jaco_control::vision_data_callback,this);
    ROS_INFO("Connecting itongue");
    connect_itongue();
    ROS_INFO("Itongue connected");
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    // spherical_grip(100);
    // sleep(3);
    // spherical_grip(45);
    testemil();
    ros::Rate rate(10.0);



    while (nh_.ok()){     
        
        vision::Detection data;
        
        // data.X1 = 0.492;
        // data.X2 = 0.634;
        // data.Y1 = 0.207;
        // data.Y2 = 0.823;

            
            
    //MANUEL
        // shapefitting::shapefitting_position_arrayGoal goal;
        
        // data.X1 = 0.492;
        // data.X2 = 0.634;
        // data.Y1 = 0.3;
        // data.Y2 = 0.8;
        // data.Class = 10;
        // goal.input.msg.push_back(data);
        

        //Update camera with respect to end effector
        setCameraPos();

    //AUTOMATISK
        shapefitting::shapefitting_simple_position_arrayGoal goal;

        for (vision::Detection data : visionDataArray.msg){
            goal.input.msg.push_back(data);
        }


        
        actionlib::SimpleClientGoalState shapefitting_ac_state = shapefitting_ac.getState();
   
        
        if (shapefitting_ac_state.isDone())
        {
                //shapefitting_ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb)
            if (visionDataArray.msg.size() > 0)
            {
                    shapefitting_ac.sendGoal(goal,
                    boost::bind(&jaco_control::shapefitting_doneCb,this,_1,_2),
                        boost::bind(&jaco_control::shapefitting_activeCb,this),
            //actionlib::SimpleActionClient<shapefitting::shapefitting_position_arrayAction>::SimpleActiveCallback(),
            actionlib::SimpleActionClient<shapefitting::shapefitting_simple_position_arrayAction>::SimpleFeedbackCallback());
            }
        }

        // if (visionDataArray.msg.size() > 0)
        // {
        // //            shapefitting_ac.sendGoal(goal,
        // //            boost::bind(&jaco_control::shapefitting_doneCb,this,_1,_2),
        // //             boost::bind(&jaco_control::shapefitting_activeCb,this),
        // //  //actionlib::SimpleActionClient<shapefitting::shapefitting_position_arrayAction>::SimpleActiveCallback(),
        // //  actionlib::SimpleActionClient<shapefitting::shapefitting_position_arrayAction>::SimpleFeedbackCallback());
        // }
        


        //ROS_INFO("NHOK");
        try{    
            //Send transforms to /tf:
            static_broadcaster.sendTransform(Transform_camera);
            for (geometry_msgs::TransformStamped camData : tf_cam_to_object){
                //ROS_INFO_STREAM("JAJA");
                br.sendTransform(camData);
            }


            current_robot_transformStamped = tfBuffer.lookupTransform("world", "j2n6s300_end_effector",
                                    ros::Time(0),ros::Duration(3.0));
            
            // Transform each camData into world frame and save
            
            tf_cam_to_object[0].header.stamp = ros::Time::now();
            for (geometry_msgs::TransformStamped camData : tf_cam_to_object){
                // ROS_INFO_STREAM(camData.transform.translation.x);
                // ROS_INFO_STREAM(camData.transform.translation.y);
                // ROS_INFO_STREAM(camData.transform.translation.z);
                obj_ee_array.clear();
                obj_ee_array.push_back(tfBuffer.lookupTransform("world", camData.child_frame_id,
                                    ros::Time(0),ros::Duration(3.0)));
            }

        }
        catch (tf2::TransformException &ex) {
             //First call might not pass through, as the frame may not have existed, when the transform is requested the transform may not exist yet and fails the first time.
             //After the first transform all the transforms exist and the transforms should work.
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        //Check if previous goal is finished
        //if Finished send new goal using sendGoal(goal,doneCb,activeCb,feedbackCb)
    }
}

geometry_msgs::Point jaco_control::EndEffDirVec(geometry_msgs::Point iTongueDirection)
{
    geometry_msgs::Point EndEffDirVec, EndEffDirNormVec;
    EndEffDirVec.x = iTongueDirection.x;
    EndEffDirVec.y = iTongueDirection.y;
    EndEffDirVec.z = iTongueDirection.z;
    
    double lenghtA = sqrt((std::pow(EndEffDirVec.x,2))+(std::pow(EndEffDirVec.y,2))+(std::pow(EndEffDirVec.z,2)));
    EndEffDirNormVec.x = EndEffDirVec.x/lenghtA;
    EndEffDirNormVec.y = EndEffDirVec.y/lenghtA;
    EndEffDirNormVec.z = EndEffDirVec.z/lenghtA;

    return EndEffDirNormVec;

} //Enheds retnings vektor (skal normaliseres)

std::vector<jaco_control::ObjectInScene> jaco_control::ObjDirectionVectors(std::vector<geometry_msgs::TransformStamped> &objectsIn, geometry_msgs::TransformStamped &endEffPoseIn){
        debug_assistive("OBJDIRRECTION 11");
    geometry_msgs::TransformStamped endEffPose = endEffPoseIn;
    std::vector<jaco_control::ObjectInScene> objectDataVec;
     debug_assistive("OBJDIRRECTION 1");
     debug_experimental(std::to_string(objectsIn.size()));
    std::vector<geometry_msgs::TransformStamped> objects = objectsIn;
 
    debug_experimental("OBJDIRRECTION 12");
    debug_assistive(std::to_string(objects.size()));
    for (geometry_msgs::TransformStamped obj : objects){
          debug_assistive("OBJDIRRECTION 2");

        jaco_control::ObjectInScene objectData;
        geometry_msgs::Point vec;
        //Bergn vektor mellem to punkter
        vec.x = obj.transform.translation.x - endEffPose.transform.translation.x;
        vec.y = obj.transform.translation.y - endEffPose.transform.translation.y;
        vec.z = obj.transform.translation.z - endEffPose.transform.translation.z;
        //Beregn afstand
        objectData.dist = std::sqrt(std::pow(vec.x,2) + std::pow(vec.y,2) + std::pow(vec.z,2));
        
        //Normaliser retningsvector
        vec.x = vec.x/objectData.dist;
        vec.y = vec.y/objectData.dist;
        vec.z = vec.z/objectData.dist;

        //Save vector
        objectData.directionVector = vec;
        objectData.position.x = obj.transform.translation.x;
        objectData.position.y = obj.transform.translation.y;
        objectData.position.z = obj.transform.translation.z;
        //Push til vector

        objectDataVec.push_back(objectData);
    }
    
    return objectDataVec;

}

geometry_msgs::Point jaco_control::assistiveControl(geometry_msgs::Point &iTongueDirIn, std::vector<geometry_msgs::TransformStamped> &objectsIn, geometry_msgs::TransformStamped &endEffPoseIn)
{

    if (std::abs(iTongueDirIn.x) + std::abs(iTongueDirIn.y) + std::abs(iTongueDirIn.z))
    {
        /* code */
    
    

    std::vector<geometry_msgs::TransformStamped> objects = objectsIn;
    geometry_msgs::Point iTongueDir = iTongueDirIn;
    geometry_msgs::TransformStamped endEffPose = endEffPoseIn;
    //debug_experimental("Initial direction: " + std::to_string(iTongueDirIn.x) + " "+ std::to_string(iTongueDirIn.y) + " " + std::to_string(iTongueDirIn.z));
    if (!objects.empty())
    {
    debug_assistive("1");
    std::vector<float> Assist; 
    debug_assistive(std::to_string(objects.size()));
    geometry_msgs::Point EndEffDir = EndEffDirVec(iTongueDir);
 debug_assistive("11");
    
    

    std::vector<jaco_control::ObjectInScene> ObjDirVec = ObjDirectionVectors(objects,endEffPose);
    
        /* code */
   
        debug_assistive("2");
    for(size_t i = 0; i < ObjDirVec.size(); i++)
    {
        debug_experimental("i1: " + std::to_string(iTongueDir.x) + " obj1 " + std::to_string(ObjDirVec[i].directionVector.x) + " i2 " + std::to_string(iTongueDir.y) + " obj2 " + std::to_string(ObjDirVec[i].directionVector.y)  + " i3 " + std::to_string(iTongueDir.z) + " obj3 " + std::to_string(ObjDirVec[i].directionVector.z) + " sum " + std::to_string(((iTongueDir.x * ObjDirVec[i].directionVector.x) + (iTongueDir.y * ObjDirVec[i].directionVector.y) + (iTongueDir.z * ObjDirVec[i].directionVector.z) )/(std::sqrt((std::pow(iTongueDir.x,2)+(std::pow(iTongueDir.y,2)+(std::pow(iTongueDir.z,2))))))));
        
        double dotProd = ((iTongueDir.x * ObjDirVec[i].directionVector.x) + (iTongueDir.y * ObjDirVec[i].directionVector.y) + (iTongueDir.z * ObjDirVec[i].directionVector.z));
        double leniTongueVec = (std::sqrt((std::pow(iTongueDir.x,2)+(std::pow(iTongueDir.y,2)+(std::pow(iTongueDir.z,2))))));
        double Angle = acos(dotProd/leniTongueVec);
        //Angle = std::min(Angle,M_PI-Angle);
        double dist = ObjDirVec[i].dist; 
        //Find på nogle parametre
        //double Assitability =  std::max((double)dist,0.2) * Angle; // Skal måske justeres
        double Assitability = (std::pow(dist,2) * Angle) + ( 0.3 * dist ) + ( 0.03 * std::pow(Angle,4));
        Assist.push_back(Assitability); 
        debug_experimental("Adjustability: " + std::to_string(Assitability) + " dist: " + std::to_string(dist) + " Angle: " + std::to_string(Angle) + " New value: " + std::to_string(std::max((double)dist,0.2)*Angle));
    }
        debug_assistive("3");
        int id ;
        
    debug_assistive(std::to_string(objects.size()));
    if (objects.size() > 1)
    {
       id = *min_element(Assist.begin(), Assist.end());//HER
    } else
    {
        id = 0;
    }
    
    
    
    debug_assistive(std::to_string(id));
        debug_assistive("31");
    double thresh_auto = 0.1;
    double thresh_semi = 0.15;
     geometry_msgs::Point newTraj;
     debug_assistive(std::to_string(Assist.size()));
    if (Assist[id] < thresh_auto) // full auto den har du lavet
    {

            debug_assistive("4");
        debug_experimental("Full auto");
        std::cout << "Going towards object " << id << std::endl;
        newTraj.x = ObjDirVec[id].directionVector.x;
        newTraj.y = ObjDirVec[id].directionVector.y;
        newTraj.z = ObjDirVec[id].directionVector.z;

    }
    else if (Assist[id] > thresh_auto && Assist[id] < thresh_semi) // semi auto 
    {
         debug_experimental("Semi auto");
            debug_assistive("5");
        // beregn percent assistance
        double p_manual = Assist[id]-thresh_auto/(thresh_semi-thresh_auto);
        double p_assist = 1-p_manual;

        // Vægt manual hastighed 
        newTraj.x = EndEffDir.x*p_manual;
        newTraj.y = EndEffDir.y*p_manual;
        newTraj.z = EndEffDir.z*p_manual;

        // Vægt auto hastighed
        newTraj.x = newTraj.x + ObjDirVec[id].directionVector.x*p_assist;
        newTraj.y = newTraj.y + ObjDirVec[id].directionVector.y*p_assist;
        newTraj.z = newTraj.z + ObjDirVec[id].directionVector.z*p_assist;
        
    }else // full manual
    {
         debug_experimental("NOT auto");
        std::cout << "not close enough to assist" << std::endl;
        newTraj.x = EndEffDir.x;
        newTraj.y = EndEffDir.y;
        newTraj.z = EndEffDir.z;
    }
    
        debug_assistive("6");
    // Juster vektorer afhængigt af hastighed. 
    double vel = 0.3; // m/s
    newTraj.x = newTraj.x * vel;
    newTraj.y = newTraj.y * vel;
    newTraj.z = newTraj.z * vel;
    //debug_experimental("Adjusted direction: " + std::to_string(newTraj.x) + " " + std::to_string(newTraj.y) + " " + std::to_string(newTraj.z));
    debug_assistive("IM IN");

     return newTraj;
    
    }
    }
    geometry_msgs::Point newTraj;
    newTraj.x = iTongueDirIn.x;
    newTraj.y = iTongueDirIn.y;
    newTraj.z = iTongueDirIn.z;
    return newTraj;
}

// geometry_msgs::Point jaco_control::trajVel(ObjectInScene obj, geometry_msgs::Pose endEffPose){
//     std::array<double,3> startPos {endEffPose.position.x, endEffPose.position.y, endEffPose.position.z};
//     std::array<double,3> endPos {obj.position.x, obj.position.y, obj.position.z};
//     double vel = 5.0; // m/s
//     double tf = obj.dist/vel;
    
    
//     std::array<double,3> a0 = startPos; // start pos
//     std::array<double,3> a1 {0,0,0} ; // start vel
//     std::array<double,3> a2;
//     std::array<double,3> a3;
//     for (size_t i = 0; i < startPos.size(); i++)
//     {
//         a2[i] = 3/(tf*tf)*(endPos[i]-startPos[i]);
//     }
//     for (size_t i = 0; i < startPos.size(); i++)
//     {
//         a3[i] = -2/(tf*tf)*(endPos[i]-startPos[i]);
//     }
    
//     double tf = 0.5;
//     // calculate pos 0.5 sec in the future
//     std::array<double,3> futurePos;
//     for (size_t i = 0; i < futurePos.size(); i++)
//     {
//         futurePos[i] = a0[i]+a1[i]*tf+a2[i]*std::pow(tf,2)+a3[i]*std::pow(tf,3);
//     }
    
//     // calculate velocity
    
// }
    

void jaco_control::testemil(){
    
    geometry_msgs::TransformStamped testPos;
    testPos.transform.translation.x = 0.26846;
    testPos.transform.translation.y = -0.6144;
    testPos.transform.translation.z = 0.2311;
    testPos.transform.rotation.w = 0.05287;
    testPos.transform.rotation.x = 0.81934;
    testPos.transform.rotation.y = -0.57;
    testPos.transform.rotation.z = 0.015761;
    testPos.header.frame_id = "world";
    testPos.child_frame_id = "object";
    testPos.header.stamp = ros::Time::now();
    
    tf_cam_to_object.push_back(testPos);
}





int main(int argc, char **argv)
{
    
    ros::init(argc, argv, "kinova_arm_driver");
    ros::NodeHandle node("~");
    ros::AsyncSpinner spinner(4);
    spinner.start();
    jaco_control Jaco(node);
    //Jaco.doStuff();
    //ros::spin();
    ros::waitForShutdown();
    return 0;
}
