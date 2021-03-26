#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <math.h>
std::string read_path = "runs/detect/john/0";
cv::Mat pic = cv::imread(read_path);