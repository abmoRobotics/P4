#include <ros/ros.h>
#include <iostream>
#include <vision/detect_srv.h>
#include <vision/Detection_array.h>
#include <actionlib/client/simple_action_client.h>
#include <jaco/IF_fullAutoAction.h>
#include <jaco/RAWItongueOut.h>
#include <vision/Detection.h>
#include <time.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
// videofeed ind camera 1 og camera 2
// Visuelt interface af iTongue
// Tegnet bounding boxes på videofeed
// evt.


using namespace std;
int iTongue_sensor{0};
vector<vision::Detection> visionDataArray;
cv::Mat camera1Image = cv::Mat::zeros(640, 480, CV_8UC3);

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


void video_stream(const sensor_msgs::ImageConstPtr &msg){
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    camera1Image = cv_ptr->image;
    //cv::cvtColor(camera1Image, camera1Image, cv::COLOR_BGR2RGB);
    //cv::resize(camera1Image, camera1Image, cv::Size(camera1Image.cols * 2, camera1Image.rows * 2));
}

cv::Mat Keypad = cv::Mat::zeros(720, 360, CV_8UC3);
//bool hasRunOnce {false};

void videoFeed_window(){
    
    
    cv::Size elipseSize = cv::Size(60,35);
    vector<cv::Point> keypadPoints = {cv::Point(90, 40), cv::Point(2*90, 40), cv::Point(3*90, 40), cv::Point(90, 140), cv::Point(2*90, 140),
                                    cv::Point(3*90, 140), cv::Point(50, 240), cv::Point(50+86, 240), cv::Point(310-86, 240), cv::Point(360-50, 240),
                                    cv::Point(90, 400), cv::Point(90*3, 400), cv::Point(90, 600), cv::Point(90*3, 600), cv::Point(90*2, 400),
                                    cv::Point(90*2, 600), cv::Point(90, 500), cv::Point(90*3, 500)};

    vector<std::string> objectVector = {"150clSoda", "50clSoda", "Juice", "Minimaelk", "Roedvin", "RoseVin", "Saftevand", "Skummetmaelk", "Termokrus"};
    cv::Scalar Ycolor = cv::Scalar(0,255,255);
    int radius {35};
    

    //if (hasRunOnce == false){
    for (size_t i = 0; i < keypadPoints.size()-4; i++)
    {
        cv::circle(Keypad, keypadPoints[i], radius, Ycolor, cv::FILLED);
    }

    for (size_t i = 14; i < keypadPoints.size()-2; i++) //stå op
    {
        cv::ellipse(Keypad, keypadPoints[i], elipseSize, 90, 0, 360, Ycolor, cv::FILLED);
    }

    for (size_t i = 16; i < keypadPoints.size(); i++) //lig ned
    {
        cv::ellipse(Keypad,keypadPoints[i],elipseSize, 0, 0, 360, Ycolor, cv::FILLED);
    }
    //hasRunOnce = true;
    //}
    
    
    for (int i = 0; i < visionDataArray.size(); i++)
    {
        cv::Point pt1, pt2, textPoint;
        pt1.x = visionDataArray[i].X1 * camera1Image.cols;
        pt1.y = visionDataArray[i].Y1 * camera1Image.rows;
        pt2.x = visionDataArray[i].X2 * camera1Image.cols;
        pt2.y = visionDataArray[i].Y2 * camera1Image.rows;
        cv::Scalar color;
        if(visionDataArray[i].Class = 0) color = cv::Scalar(13,17,241); // Blue
        else if(visionDataArray[i].Class = 1) color = cv::Scalar(249,58,56); // Red
        else if(visionDataArray[i].Class = 2) color = cv::Scalar(80,224,212); // Cyan
        else if(visionDataArray[i].Class = 3) color = cv::Scalar(154,7,203); // Dark purple
        else if(visionDataArray[i].Class = 4) color = cv::Scalar(19,140,65); // Green
        else if(visionDataArray[i].Class = 5) color = cv::Scalar(30,9,65); // Dark blue
        else if(visionDataArray[i].Class = 6) color = cv::Scalar(225,60,233); //Pink
        else if(visionDataArray[i].Class = 7) color = cv::Scalar(255,255,0); //Yellow
        else if(visionDataArray[i].Class = 8) color = cv::Scalar(240,108,11); //Orange
        cv::rectangle(camera1Image, pt1, pt2, color, 4);
        textPoint.x = pt1.x;
        textPoint.y = pt1.y - 10;
         cv::putText(camera1Image, objectVector[(visionDataArray[i].Class / 10)], textPoint, cv::FONT_HERSHEY_SIMPLEX, 1, color);
     }
    

vector<int> iTongueMap {3, 2, 1, 6, 5, 4, 10, 9, 8, 7, 13, 11, 18, 16, 12, 17, 15, 14};

cv::Scalar Rcolor = cv::Scalar(0,0,255);
if(iTongue_sensor > 0 && iTongue_sensor <= 18)
{
    auto it = find(iTongueMap.begin(), iTongueMap.end(), iTongue_sensor);
    int index = std::distance(iTongueMap.begin(), it);
    cv::circle(Keypad, keypadPoints[index], 8, Rcolor, cv::FILLED);
}

    cv::imshow("Keypad", Keypad);
    cv::moveWindow("Keypad", 20,20);
    cv::imshow("Display", camera1Image);
    cv::moveWindow("Display", 430, 20);
    cv::waitKey(100);
}



int main(int argc, char *argv[])
{
cv::namedWindow("Display");
cv::namedWindow("Keypad");
    ros::init(argc, argv, "interface_screen");
    ros::AsyncSpinner spinner(0);
    spinner.start();

    ros::NodeHandle n;
    ros::Subscriber iTongue_sub = n.subscribe<jaco::RAWItongueOut>("RAWItongueOut", 1, iTongue_callback);
    ros::Subscriber vision_sub = n.subscribe<vision::Detection_array>("/Vision/ObjectDetection", 10, vision_callback);
    ros::Subscriber video_sub = n.subscribe<sensor_msgs::Image>("/Imagepub/RGB", 1, video_stream);
    
    while (ros::ok())
    {
        videoFeed_window();
    }
    ros::spinOnce();
    ros::waitForShutdown();
    cv::destroyAllWindows();
    return 0;
}
