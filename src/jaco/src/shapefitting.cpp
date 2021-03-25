#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <jaco_trajectory.h>



int main(int argc, char **argv){
    ros::init(argc, argv, "talker");
    ros::NodeHandle node;
    ros::AsyncSpinner spinner(1);
    spinner.start();
 while(ros::ok()){
    ros::Publisher talker_pub = node.advertise<jaco::position>("position", 10); 
    jaco::position msg;
    //Create object. Define the values to advertise. 
        msg.radius = 7.0 *2; 
        msg.posx.x = 0.5;
        msg.posy.y = 0.1;
        msg.posz.z = 0.07;
        msg.orientation.x = 0.5;
        msg.orientation.y = 0.5;
        msg.orientation.z = 0.5;

    talker_pub.publish(msg);
 }
    ros::waitForShutdown();
    return 0;
}

