#include <ros/ros.h>
#include <iostream>
//#include <vision/Detection.h>
#include <actionlib/client/simple_action_client.h>
//#include <shapefitting/shapefitting_positionAction.h> 

using namespace std;

ros::NodeHandle nh;

vector<int16_t> objects;

void fullauto_callback(const vision::Detection_arrayConstPtr& msg){
    for (size_t i = 0; i < 10; i++){
        objects.push_back(msg);
    }
}




// Få objekter fra JJ, udvælg et, send dette objekt til Emil, send Emils data til pos_callback i jaco_trajectory.
void full_automatic(){
    cout << "Choose an object: " << endl;
    for (size_t i = 0; i < sizeof(objects)/sizeof(objects[0]); i++){
        //cout << i << objects[i].class << endl;
    }
    int obj_select;
    cin >> obj_select;
    //Send object to ShapeFitting
    actionlib::SimpleActionClient<shapefitting::shapefitting_positionAction> shapefitting_client("get_shape", true); 
    //Check input in actionclient
    shapefitting_client.waitForServer();
    //shapefitting::shapefitting_positionInput input;
    shapefitting_client.sendGoal(obj_select);

    //Action for trajectory and grasping
    

    //Feedback on distance to object



    //Goal  = object grasped?


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


ros::Subscriber fullauto_sub = nh.subscribe<vision::Detection_array>("/Vision/ObjectDetection", 10, fullauto_callback);


actionlib::SimpleActionClient<shapefitting::shapefitting_positionAction> shapefitting_client("get_shape", true); 
//Check input in actionclient
shapefitting_client.waitForServer();




void main(int argc, char *argv[]){
    ros::init(argc, argv, "interface");
    ros::AsyncSpinner spinner(0);
    spinner.start();
    menu();
    ros::waitForShutdown();
    //ros::spin();
}