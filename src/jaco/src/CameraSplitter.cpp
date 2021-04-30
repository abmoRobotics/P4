#include <ros/ros.h>
#include <iostream>
#include <vision/detect_srv.h>
#include <vision/Detection_array.h>
#include <actionlib/client/simple_action_client.h>
#include <jaco/RAWItongueOut.h>
#include <vision/Detection.h>
#include <time.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/image_encodings.h>
#include <librealsense2/rs.hpp> 

static const std::string OPENCV_WINDOW = "Image window";
int counter{0};
sensor_msgs::Image imageCb(const cv::Mat& img)
{
    cv_bridge::CvImage img_bridge;
    sensor_msgs::Image rosImg;

    img_bridge = cv_bridge::CvImage();
    std_msgs::Header header; // empty header
    header.seq = counter; // user defined counter
    header.stamp = ros::Time::now(); // time
    // Draw an example circle on the video stream
    
    img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, img);
    img_bridge.toImageMsg(rosImg); // from cv_bridge to sensor_msgs::Image
    // Update GUI Window
    //cv::imshow("OPENCV_WINDOW", img);
    cv::waitKey(3);

    // Output modified video stream
    counter++;
    return rosImg;
}

rs2::pipeline InitiateRealsense()
{
    // Create config object, and enable stream of depth data.
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_DEPTH, 480, 270, RS2_FORMAT_Z16, 60);
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
    
    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Start streaming with default recommended configuration
    std::cout << "START\n"; 
    pipe.start(cfg);
    std::cout << "SLUT\n";

    return pipe;
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

cv::Mat RealSenseCallBackRGB(rs2::pipeline *pipe)
{
    rs2::frameset data = pipe->wait_for_frames();
    // Import RGB
    cv::Mat RGB = frame_to_mat(data.get_color_frame());

    return RGB;

}

static cv::Mat depth_frame_to_meters( const rs2::depth_frame & f )
{
    cv::Mat dm = frame_to_mat(f);
    dm.convertTo( dm, CV_64F );
    dm = dm * f.get_units();
    return dm;
}

cv::Mat RealSenseCallBackDepth(rs2::pipeline *pipe, rs2::align *align_color){

    rs2::frameset data = pipe->wait_for_frames();
    //Allign to color
    data = align_color->process(data);
    // Import depth
    rs2::depth_frame depth = data.get_depth_frame();
    
    // Convert frame to Mat with distances
    cv::Mat depth_mat = depth_frame_to_meters(depth);

    return depth_mat;
}

int main(int argc, char **argv)
{
    
    ros::init(argc, argv, "CameraNode");
    ros::NodeHandle nh_;
    
    ros::Publisher image_pub = nh_.advertise<sensor_msgs::Image>("/Imagepub/RGB", 1);

    rs2::pipeline pipe = InitiateRealsense();
    while(ros::ok())
    {
    cv::Mat RealsenseRGB = RealSenseCallBackRGB(&pipe);
    
    image_pub.publish(imageCb(RealsenseRGB));
    }
    
    ros::spinOnce();
    cv::destroyWindow(OPENCV_WINDOW);
    return 0;
}


// cv::Mat RealsenseRGB = RealSenseCallBackRGB(&pipe);
// cv::Mat RealsenseDepth = RealSenseCallBackDepth(&pipe, &align_to_color);