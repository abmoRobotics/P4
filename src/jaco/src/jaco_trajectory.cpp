#include <ros/ros.h>
#include <jaco_trajectory.h>

#include <actionlib/client/simple_action_client.h>
//#include <kinova_msgs/SetFingersPositionAction.h>
#include <tf_conversions/tf_eigen.h>


//moveit::planning_interface::MoveGroupInterface group_("arm");
//moveit::planning_interface::MoveGroupInterface gripper_group_("gripper");

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

	double jointValue = (M_PI / 4.0) - asin((diameter / 44.0) - (29.0 / 22.0));
    ROS_INFO_STREAM(jointValue);
	gripper_group_->setJointValueTarget("j2n6s300_joint_finger_1", jointValue);
	gripper_group_->setJointValueTarget("j2n6s300_joint_finger_2", jointValue);
	gripper_group_->setJointValueTarget("j2n6s300_joint_finger_3", jointValue);

}


void jaco_trajectory::pinch_grip(double diameter){

	double jointValue = (M_PI / 4.0) - asin((diameter / 87.0) - (2.0 / 3.0));
    ROS_INFO_STREAM(jointValue);
	gripper_group_->setJointValueTarget("j2n6s300_joint_finger_1", jointValue);
	gripper_group_->setJointValueTarget("j2n6s300_joint_finger_2", jointValue);
	//gripper_group_->setJointValueTarget("j2n6s300_joint_finger_3", jointValue);

}
void jaco_trajectory::tripod_grip(double diameter){

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
    // can_pose_.pose.position.x = co_.primitive_poses[0].position.x;
    // can_pose_.pose.position.y = co_.primitive_poses[0].position.y;
    // can_pose_.pose.position.z = co_.primitive_poses[0].position.z;
    pub_co_.publish(co_);
    planning_scene_msg_.world.collision_objects.push_back(co_);
    planning_scene_msg_.is_diff = true;
    pub_planning_scene_diff_.publish(planning_scene_msg_);
    aco_.object = co_;
    ros::WallDuration(0.1).sleep();
}

void jaco_trajectory::generate_trajectory(geometry_msgs::PoseStamped pose){
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    ROS_INFO("Starting");
    group_->clearPathConstraints();
    group_->setPoseTarget(pose);
    //group_->setNamedTarget("Home");
    
 bool   success = (group_->plan(my_plan) == moveit_msgs::MoveItErrorCodes::SUCCESS);

    group_->execute(my_plan);
}

geometry_msgs::PoseStamped jaco_trajectory::generate_gripper_align_pose(geometry_msgs::PoseStamped targetpose_msg, double dist, double azimuth, double polar, double rot_gripper_z)
{
    geometry_msgs::PoseStamped pose_msg;

    pose_msg.header.frame_id = "root";
    pose_msg.header.stamp = ros::Time::now();

    // computer pregrasp position w.r.t. location of grasp_pose in spherical coordinate. Orientation is w.r.t. fixed world (root) reference frame.
    double delta_x = -dist * cos(azimuth) * sin(polar);
    double delta_y = -dist * sin(azimuth) * sin(polar);
    double delta_z = -dist * cos(polar);

    // computer the orientation of gripper w.r.t. fixed world (root) reference frame. The gripper (z axis) should point(open) to the grasp_pose.
    tf::Quaternion q = EulerZYZ_to_Quaternion(azimuth, polar, rot_gripper_z);

    pose_msg.pose.position.x = targetpose_msg.pose.position.x+ delta_x;
    pose_msg.pose.position.y = targetpose_msg.pose.position.y+ delta_y;
    pose_msg.pose.position.z = targetpose_msg.pose.position.z+ delta_z;
    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();
    pose_msg.pose.orientation.w = q.w();


    return pose_msg;

}

void jaco_trajectory::define_cartesian_pose()
{
    tf::Quaternion q;

    // // define start pose before grasp
    // start_pose_.header.frame_id = "root";
    // start_pose_.header.stamp = ros::Time::now();
    // start_pose_.pose.position.x = 0.5;
    // start_pose_.pose.position.y = -0.5;
    // start_pose_.pose.position.z = 0.5;

    // q = EulerZYZ_to_Quaternion(-M_PI/4, M_PI/2, M_PI);
    // start_pose_.pose.orientation.x = q.x();
    // start_pose_.pose.orientation.y = q.y();
    // start_pose_.pose.orientation.z = q.z();
    // start_pose_.pose.orientation.w = q.w();

    // define grasp pose
    grasp_pose_.header.frame_id = "root";
    grasp_pose_.header.stamp  = ros::Time::now();

    // Euler_ZYZ (-M_PI/4, M_PI/2, M_PI/2)
    grasp_pose_.pose.position.x = 0.7;
    grasp_pose_.pose.position.y = 0.0;
    grasp_pose_.pose.position.z = 0.13/2;

    q = EulerZYZ_to_Quaternion(0, M_PI/2, M_PI/2);
    grasp_pose_.pose.orientation.x = q.x();
    grasp_pose_.pose.orientation.y = q.y();
    grasp_pose_.pose.orientation.z = q.z();
    grasp_pose_.pose.orientation.w = q.w();

    // generate_pregrasp_pose(double dist, double azimuth, double polar, double rot_gripper_z)
    grasp_pose_= generate_gripper_align_pose(grasp_pose_, 0.03999, 0, M_PI/2, M_PI/2);
    pregrasp_pose_ = generate_gripper_align_pose(grasp_pose_, 0.1, 0, M_PI/2, M_PI/2);
    postgrasp_pose_ = grasp_pose_;
    postgrasp_pose_.pose.position.z = grasp_pose_.pose.position.z + 0.05;
}



jaco_trajectory::~jaco_trajectory(){
    delete group_;
    delete gripper_group_;
}

void jaco_trajectory::pos_callback(const jaco::obj_posConstPtr& msg){
    for (int i = 0; i < sizeof(msg->object)/sizeof(msg->object[0]); i++)
    {
        obj_vec.push_back(msg->object[i]);
        ROS_INFO("Vector for loop");
        ROS_INFO_STREAM(obj_vec[i].radius);
    }
    


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

    // generate_trajectory(pregrasp_pose_);
    // generate_trajectory(grasp_pose_);
    // group_->setEndEffectorLink("j2n6s300_end_effector");
    // gripper_action(msg->object[0].radius);

}

void jaco_trajectory::itongue_callback(const jaco::RAWItongueOutConstPtr& msg){
   // planning_scene_monitor_->waitForCurrentRobotState(ros::Time::now());
     moveit::planning_interface::MoveGroupInterface::Plan my_plan;
   //  geometry_msgs::PoseStamped currentpose = group_->getCurrentPose();
   geometry_msgs::PoseStamped currentpose;
//    ros::topic::waitForMessage('/tf', node, ros::Duration(2));

//         tf2_ros::Buffer tfBuffer;
//   tf2_ros::TransformListener tfListener(tfBuffer);
//  try{
       
//        current_robot_transformStamped = tfBuffer.lookupTransform("world", "j2n6s300_end_effector",
//                                 ros::Time::now(),ros::Duration(3.0));
//      }
//      catch (tf2::TransformException &ex) {
//        ROS_WARN("%s",ex.what());
//        ros::Duration(1.0).sleep();
//      }

    if (old_Sensor == msg->Sensor)
    {
        Sensor_count ++;
    }
    else Sensor_count = 0;
    

    ROS_INFO_STREAM(msg->Sensor);
    ROS_INFO_STREAM(current_robot_transformStamped.transform.translation.x );
    ROS_INFO_STREAM(currentpose.pose.position.x);
    ROS_INFO_STREAM(current_robot_transformStamped.transform.translation.y );
    ROS_INFO_STREAM(currentpose.pose.position.y);
    ROS_INFO_STREAM(current_robot_transformStamped.transform.translation.z );
    ROS_INFO_STREAM(currentpose.pose.position.z);


    if(current_robot_transformStamped.transform.translation.x != 0 && Sensor_count > 4){
        currentpose.header = current_robot_transformStamped.header;
        currentpose.pose.orientation=current_robot_transformStamped.transform.rotation;
        currentpose.pose.position.x = current_robot_transformStamped.transform.translation.x;
        currentpose.pose.position.y = current_robot_transformStamped.transform.translation.y;
        currentpose.pose.position.z = current_robot_transformStamped.transform.translation.z;
        switch (msg->Sensor)
        {
        case 17: //Z forwards  - away from oneself
        ROS_INFO("Z forwards  - away from oneself");
            currentpose.pose.position.z = currentpose.pose.position.z - 0.050000;
            break;
        case 12: //Z backwards -- towards oneself
            currentpose.pose.position.z = currentpose.pose.position.z + 0.050000;
            break; 
        case 11: // cross up-left
        ROS_INFO("cross up-left");
            currentpose.pose.position.y = currentpose.pose.position.y + 0.050000;
            currentpose.pose.position.x = currentpose.pose.position.x - 0.050000;
            break;
        case 8:// Y upwards
        ROS_INFO("Y upwards");
            currentpose.pose.position.y = currentpose.pose.position.y + 0.050000;
            break;
        case 13: // Cross up-right
        ROS_INFO("Cross up-right");
            currentpose.pose.position.y = currentpose.pose.position.y + 0.050000;
            currentpose.pose.position.x = currentpose.pose.position.x + 0.050000;
            break;
        case 14: //x left
        ROS_INFO("x left");
            currentpose.pose.position.x = currentpose.pose.position.x - 0.050000;
            break;
        case 15: //x right
        ROS_INFO("x right");
            currentpose.pose.position.x = currentpose.pose.position.x + 0.050000;
            break;
        case 16: // Cross down-left
            currentpose.pose.position.y = currentpose.pose.position.y - 0.050000;
            currentpose.pose.position.x = currentpose.pose.position.x - 0.050000;
            break;
        case 9: // y downwards
            currentpose.pose.position.y = currentpose.pose.position.y - 0.050000;
            break;
        case 18: // Cross down-right
            currentpose.pose.position.y = currentpose.pose.position.y - 0.050000;
            currentpose.pose.position.x = currentpose.pose.position.x + 0.050000;
            break;

        default:
            break;
        }
        ROS_INFO_STREAM(currentpose.pose.position.x);
        ROS_INFO_STREAM(currentpose.pose.position.y);
        ROS_INFO_STREAM(currentpose.pose.position.z);
        group_->setPoseTarget(currentpose);
        group_->move();
        bool success = (group_->plan(my_plan) == moveit_msgs::MoveItErrorCodes::SUCCESS);
        group_->execute(my_plan);
    }
}

void jaco_trajectory::tf_callback(const tf::tfMessageConstPtr& msg1){
    ROS_INFO_STREAM(msg1->transforms[6].header.frame_id);

    

}




void jaco_trajectory::itongue_control(int test){
moveit::planning_interface::MoveGroupInterface::Plan my_plan;
//geometry_msgs::PoseStamped currentpose = group_->getCurrentPose();
//robot_state::RobotStatePtr fdgsfgdgfs = group_->getCurrentState();
// switch (test)
// {
// case 1:
//     currentpose.pose.position.x = currentpose.pose.position.x + 0.01;
//     break;
// case 2:
//     currentpose.pose.position.x = currentpose.pose.position.x - 0.01;
//     break;
//  case 3:
//     currentpose.pose.position.y = currentpose.pose.position.y + 0.01;
//     break;
// case 4:
//     currentpose.pose.position.y = currentpose.pose.position.y - 0.01;
//     break;
// case 5:
//     currentpose.pose.position.z = currentpose.pose.position.z + 0.01;
//     break;
// case 6:
//     currentpose.pose.position.z = currentpose.pose.position.z - 0.01;
//     break;
// default:
//     break;
// }

    
    
    
//     group_->setPoseTarget(currentpose);
//     bool success = (group_->plan(my_plan) == moveit_msgs::MoveItErrorCodes::SUCCESS);
//     group_->execute(my_plan);

}


jaco_trajectory::jaco_trajectory(ros::NodeHandle &nh): nh_(nh){

     
    // // Before we can load the planner, we need two objects, a RobotModel and a PlanningScene.
    // robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    // robot_model_ = robot_model_loader.getModel();

    // // construct a `PlanningScene` that maintains the state of the world (including the robot).
    // planning_scene_.reset(new planning_scene::PlanningScene(robot_model_));
    planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));
    nh_.param<bool>("/robot_connected",robot_connected_,true);
    //nh_.param<std::string>("/robot_type",robot_type_,"j2n6s300"); 
    
    group_ = new moveit::planning_interface::MoveGroupInterface("arm");
    gripper_group_ = new moveit::planning_interface::MoveGroupInterface("gripper");
    //pub_planning_scene_diff_ = nh_.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    
        /// Functions below are replaced or moved to the pos_callback function
    //define_cartesian_pose();
    // generate_trajectory(pregrasp_pose_);
    // generate_trajectory(grasp_pose_);
    // group_->setEndEffectorLink("j2n6s300_end_effector"); //robot_type_ + "_end_effector" <---
    // robot_state::RobotStatePtr current_state;
    // current_state = group_->getCurrentState();
    

    //  finger_client_ = new actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction>
    //          ("/j2n6s300_driver/fingers_action/finger_positions", false);
    //  while(robot_connected_ && !finger_client_->waitForServer(ros::Duration(5.0))){
    //    ROS_INFO("Waiting for the finger action server to come up");
    //  }moveit::planning_interface::MoveGroupInterface
    
    pub_aco_ = nh_.advertise<moveit_msgs::AttachedCollisionObject>("/attached_collision_object", 10);
    pub_co_ = nh_.advertise<moveit_msgs::CollisionObject>("/collision_object", 10);
    pub_planning_scene_diff_ = nh_.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    itongue_sub_ = nh.subscribe<jaco::RAWItongueOut>("/RAWItongueOut", 1, &jaco_trajectory::itongue_callback,this);
    //tf_sub = nh.subscribe<tf::tfMessage>("/tf", 1, &jaco_trajectory::tf_callback, this);

    //itongue_sub_ = nh.subscribe<jaco::RAWItongueOut>("/RAWItongueOut", 1, &jaco_trajectory::itongue_callback,this);
    //tf_sub = nh.subscribe<tf::tfMessage>("/tf", 1, &jaco_trajectory::tf_callback, this);
    pos_sub = nh.subscribe<jaco::obj_pos>("/obj_pos", 1000, &jaco_trajectory::pos_callback, this); //EMIL

    // tf2::toMsg(tf2::Transform::getIdentity(), joint_global_frame_pose_stamped.pose);
    // tf2::toMsg(tf2::Transform::getIdentity(), joint_pose_stamped.pose);

    //tf::TransformListener listener;
    //tf::StampedTransform transform;

//    listener.lookupTransform("j2n6s300_link_6", ros::Time(0), transform);
    //listener.transformPose("j2n6s300_link_6", ros::Time(0), transform);
    // add_target();
    // gripper_action(0.5*FINGER_MAX);
    // pickup_object();



    // for (size_t i = 0; i < 20; i++)
    // {
    //     itongue_control(1);
    //      //ros::WallDuration(0.01).sleep();
    // }
    // for (size_t i = 0; i < 20; i++)
    // {
    //     itongue_control(2);
    //      //ros::WallDuration(0.01).sleep();
    // }
    // for (size_t i = 0; i < 20; i++)
    // {
    //     itongue_control(3);
    //      //ros::WallDuration(0.01).sleep();10| eelchair for the first time, please observe the following precautions:1.Practice using the Itongueon a computer or tablet before using it with a wheelchair.2.Ask your wheelchair service technician to reduce the speed settings of your wheelchair.3.Practice using the Itongueto control the wheelchair indoors and with others present.4.Be aware that speaking while in control of a wheelchair may cause the wheelchair to move unexpectedly.Menu (Key-board)
    // }
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    ros::Rate rate(10.0);
    while (nh_.ok()){
     
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


int main(int argc, char **argv)
{
    ros::init(argc, argv, "test");
    ros::NodeHandle node;
    ros::AsyncSpinner spinner(4);
    spinner.start();

    jaco_trajectory Jaco(node);

    // ros::Rate rate(10.0);
    // while(ros::ok()){

    // }
        //     Jaco.joint_pose_stamped.header.frame_id = "j2n6s300_link_6";
        // Jaco.joint_pose_stamped.header.stamp = ros::Time();
        // Jaco.tf_.transform(Jaco.joint_pose_stamped, Jaco.joint_global_frame_pose_stamped, "j2n6s300_link_base");
        // ROS_INFO_STREAM(Jaco.joint_global_frame_pose_stamped.pose.position.x);
                /////// Pass frame from tf ///////

    //Jaco.group_->getCurrentPose();
    //ros::spin();
    ros::waitForShutdown();
    return 0;
}
