#include <ros/ros.h>
#include <iostream>
#include <vision/detect_srv.h>
#include <vision/Detection_array.h>
#include <actionlib/client/simple_action_client.h>
#include <jaco/IF_fullAutoAction.h> 
#include <jaco/RAWItongueOut.h>
#include <time.h>

using namespace std;
vector<vision::Detection> objects;
int itongue_sensor;
int menu_selection = 0;
vector<vision::Detection> visionDataArray;

void full_automatic(){
    system("clear");
   // int length = sizeof(visionDataArray.msg)/sizeof(visionDataArray.msg[0]);
    
    //ROS_INFO_STREAM(visionDataArray.size());
        for (size_t i = 0; i < visionDataArray.size(); i++){
            cout << "Object no." << i  << endl;
        }
    
    cout << "Choose an Object: " << endl;
    jaco::IF_fullAutoActionPtr acPrt;
    actionlib::SimpleActionClient<jaco::IF_fullAutoAction> aclient("IF_full_auto", true); 
    jaco::IF_fullAutoGoal goal;
    for (size_t i = 0; i < visionDataArray.size(); i++) {
        if (itongue_sensor == i+1){
        goal.goalObject = visionDataArray[i];
        aclient.waitForServer();
        aclient.sendGoal(goal);
        aclient.waitForResult();
    }   }
    

    if (aclient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        cout << "Robot have reached the object." << endl;
    }

}

//Automation begynder ved en bestemt distance til et objekt.
void semi_automatic(){

}

void itongue_callback(const jaco::RAWItongueOutConstPtr& msg){

    static int old_Sensor, Sensor_count;
    if (old_Sensor == msg->Sensor){
        Sensor_count ++;
    }
    else Sensor_count = 0;

    old_Sensor = msg->Sensor;

    // To get the sensor activated
    if (Sensor_count > 4){
        itongue_sensor = old_Sensor;
    }    
    else itongue_sensor = 0;

}


void vision_callback(const vision::Detection_arrayConstPtr& msg){
    visionDataArray.clear();
    for (size_t i = 0; i < sizeof(msg->msg)/sizeof(msg->msg[0]); i++)
    {
        vision::Detection DetectionData;
        DetectionData = msg->msg[i];
        visionDataArray.push_back(DetectionData);
    }
}


void menu(){

   // int selection = 1;

    // while (selection != 0){
        system("clear");
        cout << "_____________________________" << endl;
        cout << "1. Full Automatic Control Mode" << endl;
        cout << "2. Semi Automatic Control Mode" << endl;
        cout << "Select Control Mode" << endl;
        cout << "----------------------------" << endl;
        if (itongue_sensor == 1)
        {
            menu_selection = 1;
            system("clear");
            sleep(2);
            itongue_sensor = 0;
            
         } else if (itongue_sensor == 2)
        {
           menu_selection = 2;
            system("clear");
            sleep(2);
            itongue_sensor = 0;
        }
        
        

        // cin >> selection;

        // switch (selection)
        // {
        // case 1:
        //     full_automatic();
        //     break;
        // case 2:
        //     semi_automatic();
        //     break;

        // default:
        //     break;
        // }
    //     selection = 0;
    // }
    

}


int main(int argc, char *argv[]){
    ros::init(argc, argv, "interface");
    ros::AsyncSpinner spinner(0);
    spinner.start();
        
    ros::NodeHandle nh;
    ros::Subscriber itongue_sub = nh.subscribe<jaco::RAWItongueOut>("/RAWItongueOut", 1, itongue_callback);
    ros::Subscriber vision_sub = nh.subscribe<vision::Detection_array>("/Vision/ObjectDetection", 10, vision_callback);
    // jaco::IF_fullAutoActionPtr acPrt;
    // actionlib::SimpleActionClient<jaco::IF_fullAutoAction> aclient("IF_full_auto", true); 
    // jaco::IF_fullAutoGoal goal;
    // goal.goalObject.X1 = 101;
    //     ROS_INFO("1");
    // aclient.waitForServer();
    // ROS_INFO("2");
    // aclient.sendGoal(goal);
    // aclient.waitForResult();
    // ROS_INFO("DONE");
    // time_t start = time(NULL);
    // time_t now = time(NULL);
    //Run starting menu for 20 seconds
    // while ((now - start) <= 20){
    //     menu();
    //     time_t now = time(NULL);
    // }    

    // while ((now - start) >= 21){
    //     ros::NodeHandle nh;
    // ros::Subscriber itongue_sub = nh.subscribe<jaco::RAWItongueOut>("/RAWItongueOut", 1, itongue_callback);
        
    // }
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        /* code */
        //menu() - hvis vi ikke har valgt kategori
        if (menu_selection == 0)
        {
            menu();
        }
        
        if (menu_selection == 1){
            full_automatic();
        }

        // if (menu_selection == 2){
        //     semi_automatic();
        // }

        //FUll_automatic() hvis vi har valgt en kategori
        loop_rate.sleep();
    }
    


    ros::spinOnce();
    
    ros::waitForShutdown();
    //ros::spin();
    return 0;
}