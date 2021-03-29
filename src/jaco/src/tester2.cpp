 #include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
 #include <geometry_msgs/TransformStamped.h>



int main(int argc, char** argv){
ros::init(argc, argv, "my_tf2_listener");
ros::NodeHandle node;
 
tf2_ros::Buffer tfBuffer;
tf2_ros::TransformListener tfListener(tfBuffer);

ros::Rate rate(10.0);
while (node.ok()){
  geometry_msgs::TransformStamped transformStamped;
  try{
       
    transformStamped = tfBuffer.lookupTransform("j2n6s300_end_effector", "world",
                                ros::Time::now(),ros::Duration(3.0));
      ROS_INFO_STREAM(transformStamped.transform.translation.x);
      ROS_INFO_STREAM(transformStamped.transform.translation.y);
      ROS_INFO_STREAM(transformStamped.transform.translation.z);
    }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    ros::Duration(1.0).sleep();
    continue;
  }
 

 
     

    rate.sleep();
  }
  return 0;
};

//  int main(int argc, char** argv){
// ros::init(argc, argv, "my_tf2_listener");
//    ros::NodeHandle node;
 
   
//   tf2_ros::Buffer tfBuffer;
//   tf2_ros::TransformListener tfListener(tfBuffer);

//   ros::Rate rate(10.0);
//    while (node.ok()){
//      geometry_msgs::TransformStamped transformStamped;
//      try{
       
//        transformStamped = tfBuffer.lookupTransform("j2n6s300_end_effector", "world",
//                                 ros::Time::now(),ros::Duration(3.0));
//         ROS_INFO_STREAM(transformStamped.transform.translation.x);
//         ROS_INFO_STREAM(transformStamped.transform.translation.y);
//         ROS_INFO_STREAM(transformStamped.transform.translation.z);
//      }
//      catch (tf2::TransformException &ex) {
//        ROS_WARN("%s",ex.what());
//        ros::Duration(1.0).sleep();
//        continue;
//      }
 

 
     

//     rate.sleep();
//   }
//   return 0;
//  };
