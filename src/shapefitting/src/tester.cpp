
#include <ros/ros.h>
int main(int argc, char **argv){
     ros::init(argc, argv, "talker");
    ros::NodeHandle node;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    return 0;
}