//  #include <ros/ros.h>
// #include <tf2_ros/transform_listener.h>
//  #include <geometry_msgs/TransformStamped.h>
//  #include <vision/detect_srv.h>

// using namespace std;
// //vector<vision::Detection> obj;

// class Server{

// public:
// vision::Detection obj;

// bool service_callback(vision::detect_srv::Request &req,
//                       vision::detect_srv::Response &res){
//   ROS_INFO("SERVICE CALLBACK");
//   //res.msg = obj;

//   obj.Class = 1;
//   obj.X1 = 10;
//   obj.X2 = 20;
//   obj.Y1 = 15;
//   obj.Y2 = 45;
//   res.msg.push_back(obj);
 
//   return 1;
// }

// };

// int main(int argc, char** argv){
//   ros::init(argc, argv, "my_tf2_listener");
//   ros::NodeHandle node;
//   Server server_obj;
//   ros::ServiceServer server1 = node.advertiseService("detection_service", &Server::service_callback, &server_obj);

//   ros::spin();
// }


#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <turtlesim/Pose.h>

std::string turtle_name;



void poseCallback(const turtlesim::PoseConstPtr& msg){
  static tf2_ros::TransformBroadcaster br;
	geometry_msgs::TransformStamped transformStamped;
  
	transformStamped.header.stamp = ros::Time::now();
	transformStamped.header.frame_id = "world";
	transformStamped.child_frame_id = turtle_name;
	transformStamped.transform.translation.x = msg->x;
	transformStamped.transform.translation.y = msg->y;
	transformStamped.transform.translation.z = 0.0;
	tf2::Quaternion q;
        q.setRPY(0, 0, msg->theta);
	transformStamped.transform.rotation.x = q.x();
	transformStamped.transform.rotation.y = q.y();
	transformStamped.transform.rotation.z = q.z();
	transformStamped.transform.rotation.w = q.w();

	br.sendTransform(transformStamped);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "broadcast");

  ros::NodeHandle node;
  //ros::NodeHandle private_node("~");
  if (! node.hasParam("turtle"))
  {
    if (argc != 2){ROS_ERROR("need turtle name as argument"); return -1;};
    turtle_name = argv[1];
  }
  else
  {
    node.getParam("turtle", turtle_name);
  }
    
  // ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe(turtle_name+"/pose", 10, &poseCallback);

  ros::spin();
  return 0;
};

