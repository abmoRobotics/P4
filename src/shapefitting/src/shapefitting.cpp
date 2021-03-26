#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <jaco/position.h>
#include <jaco/obj_pos.h>

ros::Publisher talker_pub;

int main(int argc, char **argv){
    ros::init(argc, argv, "talker");
    ros::NodeHandle node;
    ros::AsyncSpinner spinner(1);
    spinner.start();
 while(ros::ok()){
    ros::Publisher talker_pub = node.advertise<jaco::obj_pos>("position", 10); 
    jaco::obj_pos msg;
    jaco::position data;

    //Create object. Define the values to advertise. 
        data.radius = 7.0 *2; 
        data.pos.x = 0.5;
        data.pos.y = 0.1;
        data.pos.z = 0.07;
        data.orientation.x = 0.5;
        data.orientation.y = 0.5;
        data.orientation.z = 0.5;
        msg.object.push_back(data);

        data.radius = 6.0 *2; 
        data.pos.x = 0.6;
        data.pos.y = 0.2;
        data.pos.z = 0.08;
        data.orientation.x = 0.6;
        data.orientation.y = 0.6;
        data.orientation.z = 0.6;
        msg.object.push_back(data);

    talker_pub.publish(msg);
 }
    ros::waitForShutdown();
    return 0;
}

