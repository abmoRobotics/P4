#include <shapefitting_simple.h>

typedef actionlib::SimpleActionServer<shapefitting::shapefitting_simple_position_arrayAction> ShapeSimpleArrayActionServer;

using namespace cv;
using namespace rs2;

Mat Depth_Image;
bool runonce = false;

void array(const shapefitting::shapefitting_simple_position_arrayGoalConstPtr goal, ShapeSimpleArrayActionServer* as){
    // Initiate Return variables
    shapefitting::shapefitting_simple_position_arrayResult resultreturn;
    shapefitting::shape_data result_temp;

    Mat depth_mat = Depth_Image;
   
    int i = 0;
    // For each element to detect
    for (auto element : goal->input.msg)
    {
        int ROI_X = ((element.X1 + element.X2)/2)*depth_mat.cols;
        int ROI_Y = ((element.Y1 + element.Y2)/2)*depth_mat.rows;

        double depth = GetDepthAt(ROI_X, ROI_Y, depth_mat);

        double X, Y, Z;

        // Convert offset coordinates, to put (0,0) in the center
        int Xc = -((depth_mat.cols/2) - ROI_X);
        int Yc = -((depth_mat.rows/2) - ROI_Y);
        
        // D435i depth FOV: 86*57
        // D435i RGB FOV: 64*41
        // Calculate maximum distance from centre at 1 meter, for x and y
        double Xat1 = 0.624869352; //tan(64 / 2)
        double Yat1 = 0.373884679; //tan(41 / 2)

        // Normalize X and Y picture coordinates.
        double NormX = (double)Xc / (double)(depth_mat.cols / 2);
        double NormY = (double)Yc / (double)(depth_mat.rows / 2);

        // Get distance at x,y coordinate(Normal distance to plain containing camera sensor)
        double distNorm = depth;

        switch (element.Class)
        {
        case 0: // 150clSoda
            result_temp.radius = 0.035;
            Z = distNorm+result_temp.radius;
            result_temp.object_class.data = "150clSoda";
            break;
        case 10: // 50clSoda
            result_temp.radius = 0.04;
            Z = distNorm+result_temp.radius;
            result_temp.object_class.data = "50clSoda";
            break;
        case 20: // Juice
            result_temp.radius = 0.03;
            Z = distNorm+result_temp.radius;
            result_temp.object_class.data = "Juice";
            break;
        case 30: // Minimælk
            result_temp.radius = 0.033;
            Z = distNorm+result_temp.radius;
            result_temp.object_class.data = "Minimælk";
            break;
        case 40: // Rødvin
            result_temp.radius = 0.0375;
            Z = distNorm+result_temp.radius;
            result_temp.object_class.data = "Rødvin";
            break;
        case 50: // Rosevin
            result_temp.radius = 0.035;
            Z = distNorm+result_temp.radius;
            result_temp.object_class.data = "Rosevin";
            break;
        case 60: // Saftevand
            result_temp.radius = 0.035;
            Z = distNorm+result_temp.radius;
            result_temp.object_class.data = "Saftevand";
            break;
        case 70: // Skummetmælk
            result_temp.radius = 0.033;
            Z = distNorm+result_temp.radius;
            result_temp.object_class.data = "Skummetmælk";
            break;
        case 80: // Termokrus
            result_temp.radius = 0.0378;
            Z = distNorm+result_temp.radius;
            result_temp.object_class.data = "Termokrus";
            break;
        default: // Default værdi
            result_temp.radius = 0.03;
            Z = distNorm+0.03;
            result_temp.object_class.data = "FEJL";
            break;
        }

        // Set X, Y, Z coordinates
        X = NormX * Xat1 *distNorm;
        Y = NormY * Yat1 *distNorm;

        // Put coordinates in Data, and pushback into list
        result_temp.pos.x = X;
        result_temp.pos.y = Y;
        result_temp.pos.z = Z;
        ROS_INFO_STREAM(Z);

        result_temp.orientation.x = 0;
        result_temp.orientation.y = 0;
        result_temp.orientation.z = 0;

        result_temp.object_index = element.Class;

        resultreturn.object.push_back(result_temp);
    }
    ROS_INFO_STREAM("DONE");
    as->setSucceeded(resultreturn);
    
}

void UpdatePointCloud(const jaco::DepthImageConstPtr &msg){

    int i = 0;
    
    if(!runonce){
        cv::Mat Initialiser(Size(msg->width, msg->height), CV_64F);
        Depth_Image = Initialiser;
        runonce = true;
    }
    

    for (int x = 0; x < msg->width; x++)
    {
        for (int y = 0; y < msg->height; y++)
        {
            Depth_Image.at<double>(cv::Point(x, y)) = msg->data.at(i);
            i++;
        }
        
    }
    
}

int main(int argc, char **argv){

    ros::init(argc, argv, "get_shape");
    ros::NodeHandle node;
    
    ROS_INFO("Shape fitting up and running");

    ShapeSimpleArrayActionServer server(node,"get_simple_shape_array", boost::bind(&array,_1,&server),false);

    ros::Subscriber PointCloudSubscriber;

    PointCloudSubscriber = node.subscribe<jaco::DepthImage>("/Imagepub/Depth",1,&UpdatePointCloud);

    server.start();
    ros::spin();
    
    return 0;
}

Mat Overlap(Mat depth, Mat filter) {

    Mat Out = depth; //Return variable
    
    // For all pixels in depth-image
    for (int x = 0; x < depth.cols; x++) {
        for (int y = 0; y < depth.rows; y++) {
            // If pixel white, put in depth
            if (filter.at<uchar>(cv::Point(x, y)) > 125) {
                Out.at<double>(cv::Point(x, y)) = depth.at<double>(cv::Point(x,y));
            }
            // Otherwise, put invalid depth(0)
            else { 
                Out.at<double>(cv::Point(x, y)) = 0;
            }
        }
    }

    return Out;
}

static cv::Mat frame_to_mat(const rs2::frame& f)
{
    using namespace cv;
    using namespace rs2;

    auto vf = f.as<video_frame>();
    const int w = vf.get_width();
    const int h = vf.get_height();

    if (f.get_profile().format() == RS2_FORMAT_BGR8)
    {
        return Mat(Size(w, h), CV_8UC3, (void*)f.get_data(), Mat::AUTO_STEP);
    }
    else if (f.get_profile().format() == RS2_FORMAT_RGB8)
    {
        auto r_rgb = Mat(Size(w, h), CV_8UC3, (void*)f.get_data(), Mat::AUTO_STEP);
        Mat r_bgr;
        cvtColor(r_rgb, r_bgr, COLOR_RGB2BGR);
        return r_bgr;
    }
    else if (f.get_profile().format() == RS2_FORMAT_Z16)
    {
        return Mat(Size(w, h), CV_16UC1, (void*)f.get_data(), Mat::AUTO_STEP);
    }
    else if (f.get_profile().format() == RS2_FORMAT_Y8)
    {
        return Mat(Size(w, h), CV_8UC1, (void*)f.get_data(), Mat::AUTO_STEP);
    }
    else if (f.get_profile().format() == RS2_FORMAT_DISPARITY32)
    {
        return Mat(Size(w, h), CV_32FC1, (void*)f.get_data(), Mat::AUTO_STEP);
    }

    throw std::runtime_error("Frame format is not supported yet!");
}

// Converts depth frame to a matrix of doubles with distances in meters
static cv::Mat depth_frame_to_meters( const rs2::depth_frame & f )
{
    cv::Mat dm = frame_to_mat(f);
    dm.convertTo( dm, CV_64F );
    dm = dm * f.get_units();
    return dm;
}

rs2::pipeline InitiateRealsense(){
    // Create config object, and enable stream of depth data.
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 360, RS2_FORMAT_Z16, 90);
    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Start streaming with default recommended configuration
    pipe.start(cfg);

    return pipe;
}

cv::Mat GetDepthData(rs2::pipeline *pipe){
// Wait for next set of frames from the camera
    frameset data = pipe->wait_for_frames();
    // Import depth
    depth_frame depth = data.get_depth_frame(); 
    // Convert frame to Mat with distances
    Mat depth_mat = depth_frame_to_meters(depth);

    // Manually align depth data to RGB data
    // D435i depth FOV: 86*57
    // D435i RGB FOV: 64*41
    double w = 0.560; // DO NOT ASK! Det er gæt-magi ¯\_( ͡❛ ͜ʖ ͡❛)_/¯
    double h = 0.665; // Forholdet imellem FOV heights

    cv::Rect roi;
    roi.x = depth_mat.cols * ((1-w)/2);
    roi.y = depth_mat.rows * ((1-h)/2);
    roi.width = depth_mat.cols * w;
    roi.height = depth_mat.rows * h;

    std::cout << roi.width << " " << roi.height << std::endl;
    
    depth_mat = depth_mat(roi);

    resize(depth_mat, depth_mat,Size(425,239));

    return depth_mat;
}

cv::Mat IsolateROI(Mat depth_mat, double X1, double X2, double Y1, double Y2){
    //Black image
    Mat Blob = Mat::zeros(Size(depth_mat.cols,depth_mat.rows),CV_8UC1); 
    //Put white square at region of interest
    // rectangle(Blob, Point(depth_mat.cols*0.125+(X1*0.75*depth_mat.cols)-0.05,Y1*depth_mat.rows-0.05), 
    //                 Point(depth_mat.cols*0.125+(X2*0.75*depth_mat.cols)+0.05,Y2*depth_mat.rows+0.05),
    //                 Scalar(255),FILLED); 

    rectangle(Blob, Point(X1*depth_mat.cols,Y1*depth_mat.rows), 
                    Point(X2*depth_mat.cols,Y2*depth_mat.rows),
                    Scalar(255),FILLED); 

    //Keep only depths where Blob was white
    depth_mat = Overlap(depth_mat, Blob);
    return depth_mat;
}

double GetDepthAt(int x, int y, cv::Mat frame){

    double depth = 0;
    double temp = 0;
    int count = 0;

    for (int X = -6; X < 6; X++)
    {
        for (int Y = -6; Y < 6; Y++)
        {   
            temp = frame.at<double>(cv::Point(X+x, Y+y));
            if (temp > 0 && temp < 2.0)
            {
                depth = depth+frame.at<double>(cv::Point(X+x, Y+y));
                count++;
            }
        }
                
            
    }
    std::cout << depth/count << std::endl;
    return depth/count;
}

