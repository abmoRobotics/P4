#include <ros/ros.h>
#include <iostream>
#include <vision/detect_srv.h>
#include <vision/Detection_array.h>
#include <actionlib/client/simple_action_client.h>
#include <jaco/IF_fullAutoAction.h>
#include <jaco/RAWItongueOut.h>
#include <vision/Detection.h>
#include <time.h>


// videofeed ind camera 1 og camera 2
// Visuelt interface af iTongue
// Tegnet bounding boxes på videofeed
// evt.


using namespace std;
int iTongue_sensor{0};
vector<vision::Detection> visionDataArray;

void iTongue_callback(const jaco::RAWItongueOutConstPtr &msg)
{

    iTongue_sensor = msg->Sensor;
}

void vision_callback(const vision::Detection_arrayConstPtr &msg)
{
    visionDataArray.clear();

    for (int i = 0; i < msg->msg.size(); i++)
    {
        vision::Detection visionData;
        visionData = msg->msg[i];
        visionDataArray.push_back(visionData);
    }
}

void videoFeed_window(){
    
}

int main(int argc, char *argv[])
{

    ros::init(argc, argv, "interface_screen");
    ros::AsyncSpinner spinner(0);
    spinner.start();

    ros::NodeHandle n;
    ros::Subscriber iTongue_sub = n.subscribe<jaco::RAWItongueOut>("RAWItongueOut", 1, iTongue_callback);
    ros::Subscriber vision_sub = n.subscribe<vision::Detection_array>("/Vision/ObjectDetection", 10, vision_callback);
    ros::Subscriber 





    
    while (ros::ok())
    {
        
        loop_rate.sleep();
    }
    ros::spinOnce();
    ros::waitForShutdown();
    return 0;
}
