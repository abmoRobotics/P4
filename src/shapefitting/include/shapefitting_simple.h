#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <jaco/position.h>
#include <jaco/obj_pos.h>

#include <vision/Detection.h>

#include <std_msgs/String.h>
#include <actionlib/server/simple_action_server.h>
#include <shapefitting/shape_data.h>
#include <shapefitting/shapefitting_simple_position_arrayAction.h>

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <opencv2/opencv.hpp>   // Include OpenCV API
#include <array>                // For arrays
#include <iostream>             // For cout
#include <fstream>              // For File-stream
#include <sstream>
#include <exception>

                    

using namespace cv;
using namespace rs2;

// Type declarations for shape fitting


Mat Overlap(Mat depth, Mat filter);
static cv::Mat depth_frame_to_meters( const rs2::depth_frame & f );
static cv::Mat frame_to_mat(const rs2::frame& f);
rs2::pipeline InitiateRealsense();
cv::Mat GetDepthData(rs2::pipeline *pipe);
cv::Mat IsolateROI(Mat depth_mat, double X1, double X2, double Y1, double Y2);
double GetDepthAt(int x, int y, cv::Mat frame);
