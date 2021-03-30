 #include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
 #include <geometry_msgs/TransformStamped.h>
 #include <vision/detect_srv.h>

using namespace std;
//vector<vision::Detection> obj;

class Server{

public:
vision::Detection obj;

bool service_callback(vision::detect_srv::Request &req,
                      vision::detect_srv::Response &res){
  ROS_INFO("SERVICE CALLBACK");
  //res.msg = obj;

  obj.Class = 1;
  obj.X1 = 10;
  obj.X2 = 20;
  obj.Y1 = 15;
  obj.Y2 = 45;
  res.msg.push_back(obj);
 
  return 1;
}

};

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf2_listener");
  ros::NodeHandle node;
  Server server_obj;
  ros::ServiceServer server1 = node.advertiseService("detection_service", &Server::service_callback, &server_obj);

  ros::spin();
}
