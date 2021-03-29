 #include <ros/ros.h>
 #include <std_msgs/Int16.h>
 //#include <joint_command_interface.h>

// Scenario 4 from overleaf
 int main(int argc, char** argv){
    ros::init(argc, argv, "interface_full_automatic");
    ros::NodeHandle node;
    ros::Publisher talker_pub = node.advertise<std_msgs::Int16>("interface_full_automatic", 10); 
    ros::Rate loop_rate(10);
    while(ros::ok()){
        std_msgs::Int16 msg;
        msg.data = 18;

        talker_pub.publish(msg);
    }
    ros::spin();
    return 0;
 };
