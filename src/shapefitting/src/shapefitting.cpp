#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <jaco/position.h>
#include <jaco/obj_pos.h>
#include <std_msgs/String.h>
#include <actionlib/server/simple_action_server.h>
#include <shapefitting/shapefitting_positionAction.h>

ros::Publisher talker_pub;

typedef actionlib::SimpleActionServer<shapefitting::shapefitting_positionAction> ShapeFittingActionServer;

void execute(const shapefitting::shapefitting_positionGoalConstPtr goal, ShapeFittingActionServer* as){
    as->setSucceeded();
}

int main(int argc, char **argv){
    ros::NodeHandle node;


    ShapeFittingActionServer server(node,"get_shape", boost::bind(&execute,_1,&server),false);

    
    server.start();
    ros::spin();
    // ros::Publisher talker_pub = node.advertise<jaco::obj_pos>("position", 10); 
    // ros::Rate loop_rate(10);
    // while(ros::ok()){
        
    //     jaco::obj_pos msg;
    //     jaco::position data;

    //     //Create object. Define the values to advertise. 
    //         data.object_class.data = "test1";
    //         data.radius = 7.0 *2;
    //         data.pos.x = 0.5;
    //         data.pos.y = 0.1;
    //         data.pos.z = 0.07;
    //         data.orientation.x = 0.5;
    //         data.orientation.y = 0.5;
    //         data.orientation.z = 0.5;
    //         msg.object.push_back(data);

    //         data.object_class.data = "test2";
    //         data.radius = 6.0 *2; 
    //         data.pos.x = 0.6;
    //         data.pos.y = 0.2;
    //         data.pos.z = 0.08;
    //         data.orientation.x = 0.6;
    //         data.orientation.y = 0.6;
    //         data.orientation.z = 0.6;
    //         msg.object.push_back(data);

    //     talker_pub.publish(msg);
    // }
    // ros::waitForShutdown();
    return 0;
}



