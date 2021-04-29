
// MoveIt!
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <array>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <math.h> 
#include <geometric_shapes/solid_primitive_dims.h>
#include <string>
#include <jaco/RAWItongueOut.h>
#include <jaco/position.h>
#include <jaco/obj_pos.h>
#include <jaco/sys_msg.h>
#include <geometry_msgs/Pose.h>
#include <kinova_msgs/SetFingersPositionAction.h>

#include <tf/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

#include <vector>
#include <shapefitting/shapefitting_positionAction.h>
#include <vision/Detection.h>
#include <vision/Detection_array.h>

#include <actionlib/client/simple_action_client.h>
#include <shapefitting/shapefitting_position_arrayAction.h>
#include <shapefitting/shapefitting_simple_position_arrayAction.h>

#include <shapefitting/shape_data.h>
#include <jaco/IF_fullAutoAction.h>
#include <actionlib/server/simple_action_server.h>

#include <kinova_driver/kinova_comm.h>
#include <kinova/KinovaTypes.h>
#include <kinova_driver/kinova_ros_types.h>
#include "kinova_driver/kinova_api.h"
#include <kinova_msgs/AddPoseToCartesianTrajectory.h>
class jaco_control
{
private:
    //void trajectory_plan(geometry_msgs::PoseStamped pose);
    void spherical_grip(double diameter);
    //void hook_grip();
    void pinch_grip(double diameter);
    void tripod_grip(double diameter);
    void pickup_object();
    void add_target();
    // Test function delete later
    void define_cartesian_pose();
    // Define the presgrasp postion 
    //void define_pregrasp_pose(geometry_msgs::Point pose_point);
    // Define the grasp position
    //void define_grasp_pose(geometry_msgs::Point pose_point);
    void itongue_callback(const jaco::RAWItongueOutConstPtr &msg);
    void pos_callback(const jaco::obj_posConstPtr &msg); //Shape fitting
    void connect_itongue();
    void vision_data();
    //Get shape data from shapefitting node
    shapefitting::shape_data get_shape_data(vision::Detection DetectionData);
    void vision_data_callback(const vision::Detection_arrayConstPtr &msg);
    void IF_full_auto_execute(const jaco::IF_fullAutoGoalConstPtr &goal);
    //void semi_autonomous_control(); Not implemented
    //void full_autonomous_control();
    void setCameraPos();
    void testemil();
    void shapefitting_doneCb(const actionlib::SimpleClientGoalState& state, const shapefitting::shapefitting_simple_position_arrayResultConstPtr& result);
    void shapefitting_activeCb();

    kinova::KinovaPose generate_gripper_align_pose(geometry_msgs::Point targetpose_msg, double dist, double azimuth, double polar, double rot_gripper_z);
    actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction>* finger_client_;
    moveit::planning_interface::MoveGroupInterface* gripper_group_;
    robot_model::RobotModelPtr robot_model_;
    planning_scene::PlanningScenePtr planning_scene_;
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<jaco::IF_fullAutoAction> interface_as_;
    jaco::IF_fullAutoFeedback interface_feedback_;
    jaco::IF_fullAutoResult interface_result_;
    ros::Publisher pub_planning_scene_diff_;
    bool robot_connected_;
    ros::Publisher pub_co_;
    ros::Publisher pub_aco_;
    
    moveit_msgs::CollisionObject co_;
    moveit_msgs::AttachedCollisionObject aco_;
    moveit_msgs::PlanningScene planning_scene_msg_;

    
    kinova::KinovaPose grasp_pose_;
    kinova::KinovaPose pregrasp_pose_;
    kinova::KinovaPose postgrasp_pose_;
    geometry_msgs::PoseStamped object_pose_;
    ros::Subscriber itongue_sub_;
    ros::Subscriber tf_sub;
    geometry_msgs::TransformStamped current_robot_transformStamped;

    vision::Detection_array visionDataArray;
    shapefitting::shape_data shapeData;

    std::list<std::vector<geometry_msgs::Point>> Placement;

    //Frames
    shapefitting::shape_data tf_Cam_Obj;
    shapefitting::shapefitting_positionActionResult tf_World_Obj;
    tf2_ros::StaticTransformBroadcaster static_broadcaster;
    tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped obj_ee_transformStamped;
    std::vector<geometry_msgs::TransformStamped> obj_ee_array;
    geometry_msgs::TransformStamped Transform_camera, Transform_obj, object_Transform;
   
    actionlib::SimpleActionClient<shapefitting::shapefitting_simple_position_arrayAction> shapefitting_ac;

    
    struct ObjectInScene{
        double dist;                          // Afstand til objekt
        geometry_msgs::Point directionVector;       //Vektorer med retning til de forskellige objekter (normaliseret)
        geometry_msgs::Point position;
    };
    

    

    //Semi automation

    // Input bevægelsesretning fra itongue
    geometry_msgs::Point EndEffDirVec(geometry_msgs::Point iTongueDirection); //Enheds retnings vektor (skal normaliseres)

    std::vector<ObjectInScene> ObjDirectionVectors(std::vector<shapefitting::shape_data> objects, geometry_msgs::Pose endEffPose);
    
    // Function that determines level of automation
    geometry_msgs::Point assistiveControl(geometry_msgs::Point iTongueDir, std::vector<shapefitting::shape_data> objects, geometry_msgs::Pose endEffPose);

    // Eventuelt PID controller til at styre retning

    // Beregn trajectory retuner hastighed og retning på hastighed

    // geometry_msgs::Point trajVel(ObjectInScene obj, geometry_msgs::Pose endEffPose);
    
    

    std::vector<geometry_msgs::TransformStamped> tf_cam_to_object;
    
    int old_Sensor = 0;
    int Sensor_count;
    //Shape fitting:
    ros::Subscriber pos_sub;
    ros::Subscriber vision_data_sub;
    ros::Publisher talker_pub;
    ros::Publisher itongue_start_pub;
    actionlib::SimpleActionClient<shapefitting::shapefitting_positionAction> shape_data_client;

    //Physical robot
    
    boost::recursive_mutex mutexer;
    // kinova::KinovaComm kinova_comm; 

    kinova_msgs::SetFingersPositionGoal finger_goal;
    kinova::KinovaAPI kinova_api_;
    


public: 
    bool gripper_action(double finger_turn);
    jaco_control(ros::NodeHandle &nh);
    moveit::planning_interface::MoveGroupInterface* group_;
    geometry_msgs::PoseStamped joint_global_frame_pose_stamped;
    geometry_msgs::PoseStamped joint_pose_stamped;
    tf2_ros::Buffer tf_;
    void doStuff();
    //std::vector<jaco::position> obj_vec;

    
    ~jaco_control();
};

// jaco_trajectory::jaco_trajectory(/* args */)
// {
// }

// jaco_trajectory::~jaco_trajectory()
// {
// }
