
// MoveIt!
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <math.h> 
#include <geometric_shapes/solid_primitive_dims.h>

#include <jaco/RAWItongueOut.h>
#include <jaco/position.h>
#include <jaco/obj_pos.h>
#include <jaco/sys_msg.h>
#include <geometry_msgs/Pose.h>
#include <kinova_msgs/SetFingersPositionAction.h>

#include <tf/transform_listener.h>
#include <tf/tfMessage.h>
//#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <vector>
#include <shapefitting/shapefitting_positionAction.h>
#include <vision/Detection.h>
#include <vision/Detection_array.h>
#include <actionlib/client/simple_action_client.h>
#include <shapefitting/s_position.h>

class jaco_trajectory
{
private:
    void generate_trajectory(geometry_msgs::PoseStamped pose);
    void spherical_grip(double diameter);
    //void hook_grip();
    void pinch_grip(double diameter);
    void tripod_grip(double diameter);
    void pickup_object();
    void add_target();
    void define_cartesian_pose();
    void itongue_callback(const jaco::RAWItongueOutConstPtr &msg);
    void pos_callback(const jaco::obj_posConstPtr &msg); //Shape fitting
    void connect_itongue();
    void vision_data();
    void get_shape_data(vision::Detection DetectionData);
    void vision_data_callback(const vision::Detection_arrayConstPtr &msg);
    //void semi_autonomous_control(); Not implemented
    //void full_autonomous_control();

    geometry_msgs::PoseStamped generate_gripper_align_pose(geometry_msgs::PoseStamped targetpose_msg, double dist, double azimuth, double polar, double rot_gripper_z);
    actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction>* finger_client_;
    
    moveit::planning_interface::MoveGroupInterface* gripper_group_;
    robot_model::RobotModelPtr robot_model_;
    planning_scene::PlanningScenePtr planning_scene_;
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
    ros::NodeHandle nh_;
    ros::Publisher pub_planning_scene_diff_;
    bool robot_connected_;
    ros::Publisher pub_co_;
    ros::Publisher pub_aco_;
    
    moveit_msgs::CollisionObject co_;
    moveit_msgs::AttachedCollisionObject aco_;
    moveit_msgs::PlanningScene planning_scene_msg_;

    geometry_msgs::PoseStamped grasp_pose_;
    geometry_msgs::PoseStamped pregrasp_pose_;
    geometry_msgs::PoseStamped postgrasp_pose_;
    geometry_msgs::PoseStamped object_pose_;
    ros::Subscriber itongue_sub_;
    ros::Subscriber tf_sub;
    geometry_msgs::TransformStamped current_robot_transformStamped;

    vision::Detection_array visionDataArray;
    shapefitting::s_position shapeData;
 
    int old_Sensor = 0;
    int Sensor_count;
    //Shape fitting:
    ros::Subscriber pos_sub;
    ros::Subscriber vision_data_sub;
    ros::Publisher talker_pub;
    ros::Publisher itongue_start_pub;
    actionlib::SimpleActionClient<shapefitting::shapefitting_positionAction> shape_data_client;


public: 
    bool gripper_action(double finger_turn);
    jaco_trajectory(ros::NodeHandle &nh);
    moveit::planning_interface::MoveGroupInterface* group_;
    geometry_msgs::PoseStamped joint_global_frame_pose_stamped;
    geometry_msgs::PoseStamped joint_pose_stamped;
    tf2_ros::Buffer tf_;
    std::vector<jaco::position> obj_vec;

    
    ~jaco_trajectory();
};

// jaco_trajectory::jaco_trajectory(/* args */)
// {
// }

// jaco_trajectory::~jaco_trajectory()
// {
// }
