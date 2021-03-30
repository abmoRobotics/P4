#include <ros/ros.h>
#include <iostream>
#include <vision/Detection.h>

using namespace std;

ros::NodeHandle nh;

vector<> Classes;

void fullcauto_callback(const vision::Detection_arrayConstPtr& msg){


}
// Få objekter fra JJ, udvælg et, send dette objekt til Emil, send Emils data til pos_callback i jaco_trajectory.
void full_automatic(){
//Subscribe to JJ Detection.msg. Get Class and print.

//Service

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
        cout << "\n----------------------------" << endl;
        cout << "Select Option" << endl;
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


ros::Subscriber fullauto_sub = nh.subscribe<vision::Detection_array>("/Vision/ObjectDetection", 10, fullauto_callback());

void main(int argc, char *argv[]){
    ros::init(argc, argv, "interface");
    ros::AsyncSpinner spinner(0);
    spinner.start();
    menu();
    ros::waitForShutdown();
    //ros::spin();
}