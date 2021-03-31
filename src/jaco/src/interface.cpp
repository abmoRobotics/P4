#include <ros/ros.h>
#include <iostream>
#include <vision/detect_srv.h>
#include <actionlib/client/simple_action_client.h>
#include <jaco/IF_fullAutoAction.h> 

using namespace std;
vector<vision::Detection> objects;

void full_automatic(){
    ros::NodeHandle nh;
    ros::ServiceClient dclient = nh.serviceClient<vision::detect_srv>("detection_service");
    vision::detect_srv srv_detection;
    
    if (dclient.call(srv_detection)){
        int l = srv_detection.response.msg.size();
        for (size_t i = 0; i < l ; i++){
            cout << "Object no." << i << "\n" << srv_detection.response.msg[i] << endl;
        }
    }
    
    cout << "Choose an Object: " << endl;
    int select_object;
    cin >> select_object;
    jaco::IF_fullAutoActionPtr acPrt;
    actionlib::SimpleActionClient<jaco::IF_fullAutoAction> aclient("IF_full_auto", true); 
    jaco::IF_fullAutoGoal goal;
    goal.goalObject = srv_detection.response.msg[select_object];
    aclient.sendGoal(goal);
    
    aclient.waitForResult();
    if (aclient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        cout << "Robot have reached the object." << endl;
    }

    menu();

}

//Automation begynder ved en bestemt distance til et objekt.
void semi_automatic(){

}


void menu(){

    int selection = 1;

    while (selection != 0){
        system("clear");
        cout << "_____________________________" << endl;
        cout << "1. Full Automatic Control Mode" << endl;
        cout << "2. Semi Automatic Control Mode" << endl;
        cout << "Select Control Mode" << endl;
        cout << "----------------------------" << endl;
        cin >> selection;

        switch (selection)
        {
        case 1:
            full_automatic();
            break;
        case 2:
            semi_automatic();
            break;

        default:
            break;
        }
        selection = 0;
    }
    

}


int main(int argc, char *argv[]){
    ros::init(argc, argv, "interface");
    ros::AsyncSpinner spinner(0);
    spinner.start();
    menu();
    ros::waitForShutdown();
    //ros::spin();
    return 0;
}