#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <jaco/position.h>
#include <jaco/obj_pos.h>

#include <vision/Detection.h>

#include <std_msgs/String.h>
#include <actionlib/server/simple_action_server.h>
#include <shapefitting/shapefitting_positionAction.h>
#include <shapefitting/shapefitting_position_arrayAction.h>

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <opencv2/opencv.hpp>   // Include OpenCV API
#include <array>                // For arrays
#include <iostream>             // For cout
#include <fstream>              // For File-stream
#include <sstream>
#include <exception>

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

using namespace cv;
using namespace rs2;

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

Mat Overlap(Mat depth, Mat filter);
Pwn_list DepthMat_to_Pwn_list(Mat DepthMat);
Pwn_vector List2Vector(Pwn_list list);
static cv::Mat depth_frame_to_meters( const rs2::depth_frame & f );
static cv::Mat frame_to_mat(const rs2::frame& f);
Pwn_vector VectorTest();
rs2::pipeline InitiateRealsense();
cv::Mat GetDepthData(rs2::pipeline *pipe);
cv::Mat IsolateROI(Mat depth_mat, double X1, double X2, double Y1, double Y2);
Pwn_vector EstimateNormals(Pwn_list points, int nb_neighbors);
Efficient_ransac::Shape_range PerformShapeDetection(Efficient_ransac *ransac, Pwn_vector input);

