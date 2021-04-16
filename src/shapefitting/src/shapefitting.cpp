#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <jaco/position.h>
#include <jaco/obj_pos.h>

#include <vision/Detection.h>

#include <std_msgs/String.h>
#include <actionlib/server/simple_action_server.h>
#include <shapefitting/shapefitting_positionAction.h>

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <opencv2/opencv.hpp>   // Include OpenCV API
#include <array>                // For arrays
#include <iostream>             // For cout
#include <fstream>              // For File-stream
#include <sstream>
#include <exception>

cv::Mat hej; 
// For Shape fitting
#include <CGAL/property_map.h>
#include <CGAL/IO/read_xyz_points.h>
#include <CGAL/Point_with_normal_3.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Shape_detection_3/Efficient_RANSAC.h>
#include <CGAL/Shape_detection_3.h>

// For calculating Point normals
#include <CGAL/compute_average_spacing.h>
#include <CGAL/pca_estimate_normals.h>
#include <CGAL/mst_orient_normals.h>
#include <CGAL/property_map.h>
#include <list>

typedef actionlib::SimpleActionServer<shapefitting::shapefitting_positionAction> ShapeFittingActionServer;

// Type declarations for shape fitting
typedef CGAL::Exact_predicates_inexact_constructions_kernel  Kernel;
typedef std::pair<Kernel::Point_3, Kernel::Vector_3>         Point_with_normal;
typedef std::vector<Point_with_normal>                       Pwn_vector;
typedef CGAL::First_of_pair_property_map<Point_with_normal>  Point_map;
typedef CGAL::Second_of_pair_property_map<Point_with_normal> Normal_map;

typedef CGAL::Shape_detection_3::Efficient_RANSAC_traits
        <Kernel,Pwn_vector,Point_map, Normal_map>           Traits;
typedef CGAL::Shape_detection_3::Efficient_RANSAC<Traits>   Efficient_ransac;
typedef CGAL::Shape_detection_3::Plane<Traits>              Plane; 
typedef CGAL::Shape_detection_3::Cylinder<Traits>           Cylinder;

// Type declarations for calculating point normals
typedef CGAL::Sequential_tag                                Concurrency_tag;
typedef std::list<Point_with_normal>                        Pwn_list;

// For printing type of cylinder
typedef Kernel::FT                                          FT;

using namespace cv;
using namespace rs2;


Mat Overlap(Mat depth, Mat filter);
Pwn_list DepthMat_to_Pwn_list(Mat DepthMat);
Pwn_vector List2Vector(Pwn_list list);
static cv::Mat depth_frame_to_meters( const rs2::depth_frame & f );
static cv::Mat frame_to_mat(const rs2::frame& f);

void execute(const shapefitting::shapefitting_positionGoalConstPtr goal, ShapeFittingActionServer* as){
    // Create config object, and enable stream of depth data.
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_DEPTH);
    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Start streaming with default recommended configuration
    pipe.start(cfg);
    // Point cloud for storring received data
    pointcloud pc;
    // Define align object. Will be used to align depth to color.
    rs2::align align_to_color(RS2_STREAM_COLOR);

// Initiate Return variables
    shapefitting::shapefitting_positionActionResult resultreturn;


// Import new data, and create Color and depth data
    // Wait for next set of frames from the camera
    frameset data = pipe.wait_for_frames();
    // Align depth-map to RGB channel
    data = align_to_color.process(data);
    // Import depth
    depth_frame depth = data.get_depth_frame();
    // Convert frame to Mat with distances
    Mat depth_mat = depth_frame_to_meters(depth);   //Function in cv-helpers.hpp
    // Create color_mat from input
    // Mat color_mat = frame_to_mat(data.get_color_frame());   //Currently not used for anything
    
// Isolate needed values, by use of the classification
    int16_t TopLeftX = goal->input.X1;
    int16_t TopLeftY = goal->input.Y1;
    int16_t RightButtomX = goal->input.X2;
    int16_t RightButtomY = goal->input.Y2;
    // Isolate distances on region of interest

    Mat Blob = Mat::zeros(Size(depth_mat.cols,depth_mat.rows),CV_8UC1);
    rectangle(Blob, Point(TopLeftX,TopLeftY), Point(RightButtomX,RightButtomY),Scalar(255),FILLED);
    depth_mat = Overlap(depth_mat, Blob);

    // Convert depth_mat to Pwn_list:
    Pwn_list points = DepthMat_to_Pwn_list(depth_mat);

// Calculate point normals, for use in Ransac
    const int nb_neighbors = 18; // K-nearest neighbors = 3 rings
    

    if(points.size() > 1){
        CGAL::pca_estimate_normals<Concurrency_tag>(
            points.begin(),
            points.end(),
            CGAL::First_of_pair_property_map<Point_with_normal>(),
            CGAL::Second_of_pair_property_map<Point_with_normal>(),
            nb_neighbors);
                
        // Convert Points from list to vector
        Pwn_vector vector = List2Vector(points);
        
    // Perform Shape detection using Ransac
        // Instantiate shape detection engine.
        Efficient_ransac ransac;

        // Provide input data.
        ransac.set_input(vector);

        // Register cylinder shapes via template method.
        ransac.add_shape_factory<Cylinder>();

        // Set Ransac parameters
        Efficient_ransac::Parameters parameters;
        parameters.min_points = 600;            //Min amount of points within each detected cylinder
        parameters.probability = 0.005;
        parameters.epsilon = 0.002;             //Maximum acceptable euclidian distance between a point and a shape
        parameters.cluster_epsilon = 0.005;     //Maximum acceptable euclidian distance between points which are assumed to be neighbors

        // Detect registered shapes with the customized parameters.
        ransac.detect(parameters);
                
        // Print number of detected shapes and unassigned points.
        std::ostringstream InfoStream;
        
        InfoStream << ransac.shapes().end() - ransac.shapes().begin()
            << " detected shapes, "
            << ransac.number_of_unassigned_points()
            << " unassigned points, "
            << points.size()
            << " available points.";
        
        std::string INFO = InfoStream.str();
        ROS_INFO_STREAM(INFO);

        // Print details of shape
        Efficient_ransac::Shape_range shapes = ransac.shapes();         //Shapes detected by ransac
        Efficient_ransac::Shape_range::iterator it = shapes.begin();    //Itterator for going through all points
        if (shapes.size() == 0){
            ROS_WARN("No shapes detected");
            as->setAborted();
        } else {  

            while (it != shapes.end()) {
                // Get parameters depending on the detected shape.
                if (Cylinder* cyl = dynamic_cast<Cylinder*>(it->get())) {
                    Kernel::Line_3 axis = cyl->axis();
                    FT radius = cyl->radius();
                    //Print result
                    std::cout << "Cylinder with center "
                        << axis.point() << " axis " << axis.direction() << " and radius " << radius << std::endl;
                        
                    resultreturn.result.object.pos.x = axis.point().x();
                    resultreturn.result.object.pos.y = axis.point().y();
                    resultreturn.result.object.pos.z = axis.point().z();

                    resultreturn.result.object.orientation.x = axis.direction().dx();
                    resultreturn.result.object.orientation.y = axis.direction().dy();
                    resultreturn.result.object.orientation.z = axis.direction().dz();

                    resultreturn.result.object.radius = radius;
                                    
                }
                // Proceed with the next detected shape.
                it++;
            }
            as->setSucceeded();
        }
    } else {
        ROS_ERROR("Point cloud empty");
        as->setAborted();
    }

    
}

int main(int argc, char **argv){

    ros::init(argc, argv, "get_shape");
    ros::NodeHandle node;
    
    ROS_INFO("Shape fittingup and running");

    ShapeFittingActionServer server(node,"get_shape", boost::bind(&execute,_1,&server),false);

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

Pwn_vector List2Vector(Pwn_list list) {
    Pwn_vector result; //Return variable
    
    // For every entry in list
    for (Point_with_normal const c : list) {
        // Pushback element into return-vector
        result.push_back(c);
    }

    return result;
}

Pwn_list DepthMat_to_Pwn_list(Mat DepthMat)
{
    Pwn_list Out; //Return Data
    Point_with_normal Data; //Data to pushback in Pwn_list
  
    // For visualization with Mathlab function
    //std::ofstream myfile;
    //myfile.open("DataFile.txt");

    // For all pixels in depth-image(+= other than 1 to export fewer points -> Fewer calculations -> faster runtime)
    for (int x = 0; x < DepthMat.cols; x+=4) {
        for (int y = 0; y < DepthMat.rows; y+=4) {
            // If data is legal(Not 0) and below 3(Sometimes noise is present) - Export depth
            if (DepthMat.at<double>(cv::Point(x, y)) != 0 ) {//&& DepthMat.at<double>(cv::Point(x, y)) < 3
                double X, Y, Z;

                // Convert offset coordinates, to put (0,0) in the center
                int Xc = -((DepthMat.cols/2) - x);
                int Yc = -((DepthMat.rows/2) - y);
                
                // D435i depth FOV: 86*57
                // D435i RGB FOV: 64*41
                // Calculate maximum distance from centre at 1 meter, for x and y
                double Xat1 = 0.624869352; //tan(64 / 2)
                double Yat1 = 0.373884679; //tan(41 / 2)

                // Normalize X and Y picture coordinates.
                double NormX = (double)Xc / (double)(DepthMat.cols / 2);
                double NormY = (double)Yc / (double)(DepthMat.rows / 2);

                // Get distance at x,y coordinate(Normal distance to plain containing camera sensor)
                double distNorm = DepthMat.at<double>(cv::Point(x, y));

                // Set X, Y, Z coordinates
                X = NormX * Xat1 *distNorm;
                Y = NormY * Yat1 *distNorm;
                Z = distNorm;

                // Put coordinates in Data, and pushback into list
                Data.first = {X,Y,Z}; // Position
                Data.second = {0,0,0};// Normalvektor til punktet
                Out.push_back(Data);
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


