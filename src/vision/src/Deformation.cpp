#include <opencv2\core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <math.h>

void Background()
{
	cv::VideoCapture video = cv::VideoCapture(0);

	cv::Mat frame, background, output, reference, display;

	video >> background;

	std::vector<cv::Mat> pictures;
	cv::cvtColor(background, background, cv::COLOR_RGB2GRAY);
	int i{ 0 };
	cv::Point rectPoints[4];
	cv::Point rectPoints2[4];
	rectPoints[0].x = 200;
	rectPoints[0].y = 50;
	rectPoints[1].x = 400;
	rectPoints[1].y = 50;
	rectPoints[3].x = 200;
	rectPoints[3].y = 400;
	rectPoints[2].x = 400;
	rectPoints[2].y = 400;

	int space{ 5 };

	rectPoints2[0].x = rectPoints[0].x + space;
	rectPoints2[0].y = rectPoints[0].y + space;
	rectPoints2[1].x = rectPoints[1].x - space;
	rectPoints2[1].y = rectPoints[1].y + space;
	rectPoints2[3].x = rectPoints[3].x + space;
	rectPoints2[3].y = rectPoints[3].y - space;
	rectPoints2[2].x = rectPoints[2].x - space;
	rectPoints2[2].y = rectPoints[2].y - space;
	while (1) {
		
			video >> frame;
			display = frame;
			cv::cvtColor(frame, frame, cv::COLOR_RGB2GRAY);

			int t = 2;

			pictures.push_back(frame);

			output = cv::Mat::zeros(frame.rows, frame.cols, CV_8U);


			if (i >= t)
			{
				reference = pictures[i - t];
				for (size_t x = rectPoints[0].x; x < rectPoints[2].x; x++)
				{
					for (size_t y = rectPoints[0].y; y < rectPoints[3].y; y++)
					{
						output.at<uchar>(cv::Point(x, y)) = abs(frame.at<uchar>(cv::Point(x, y)) - reference.at<uchar>(cv::Point(x, y)));
					}
				}

			}
			for (size_t x = rectPoints2[0].x; x < rectPoints2[2].x; x++)
			{
				for (size_t y = rectPoints2[0].y; y < rectPoints2[3].y; y++)
				{
					output.at<uchar>(cv::Point(x, y)) = 0;
				}
			}
			cv::threshold(output, output, 30, 250, cv::THRESH_BINARY);
			cv::Scalar color(0, 255, 0);
			std::string txt{ "" };
			bool IsDeforming{ false };
			int Area = cv::countNonZero(output);
			std::cout << "Area: " << Area << std::endl;
			cv::Point pt1;
			pt1.x = 10;
			pt1.y = 100;
			if (Area > 300)
			{ 
				IsDeforming = true;
				txt = "DEFORMED";
			}
			else
			{
				IsDeforming = false;
				txt = "NOT DEFORMED";
			}
			cv::putText(display, txt, pt1, cv::FONT_HERSHEY_COMPLEX, 0.4, color);
			for (size_t i = 0; i < 4; i++)
			{
				cv::line(display, rectPoints[i], rectPoints[(i + 1) % 4], color, 1);
			}
			for (size_t i = 0; i < 4; i++)
			{
				cv::line(display, rectPoints2[i], rectPoints2[(i + 1) % 4], color, 1);
			}
			
			i++;

			cv::imshow("Image", display);
			cv::imshow("Output", output);
			cv::waitKey(20);
		
	}

}