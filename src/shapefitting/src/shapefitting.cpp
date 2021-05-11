#include <shapefitting.h>


typedef actionlib::SimpleActionServer<shapefitting::shapefitting_positionAction> ShapeFittingActionServer;
typedef actionlib::SimpleActionServer<shapefitting::shapefitting_position_arrayAction> ShapeFittingArrayActionServer;

using namespace cv;
using namespace rs2;

Mat Depth_Image;
bool runonce = false;

void array(const shapefitting::shapefitting_position_arrayGoalConstPtr goal, ShapeFittingArrayActionServer* as){
    // Initiate Return variables
    shapefitting::shapefitting_position_arrayResult resultreturn;
    shapefitting::shape_data result_temp;

    // Get depth data from pipeline
    Mat depth_mat = Depth_Image;

    int i = 0;
    // For each element to detect
    for (auto element : goal->input.msg)
    {
        // Isolate distances on region of interest
        Mat ROI = IsolateROI(depth_mat, element.X1, element.X2, element.Y1, element.Y2);

        // Convert depth_mat to Pwn_list:
        Pwn_list points = DepthMat_to_Pwn_list(ROI);

        if(points.size() > 1){
            //Estimate point normals
            Pwn_vector vector = EstimateNormals(points, 18);

            // Instantiate shape detection engine.
            Efficient_ransac ransac;

            //Shapes detected by ransac
            Efficient_ransac::Shape_range shapes = PerformShapeDetection(&ransac, vector);
            Efficient_ransac::Shape_range::iterator it = shapes.begin();    //Itterator for going through all points
            if (shapes.size() == 0){
                std::ostringstream InfoStream;
                InfoStream  << "Class " << element.Class << " not detected."; //if shape X detected
                std::string INFO = InfoStream.str();
                ROS_INFO_STREAM(INFO);
            } else {  
                while (it != shapes.end()) {
                    // Get parameters depending on the detected shape.
                    if (Cylinder* cyl = dynamic_cast<Cylinder*>(it->get())) {
                        Kernel::Line_3 axis = cyl->axis();
                        FT radius = cyl->radius();
                        //Print result
                        std::ostringstream InfoStream;
                        InfoStream 
                            << "Class "
                            << element.Class
                            << " is cylinder with center "
                            << axis.point() 
                            << " axis " 
                            << axis.direction() 
                            << " and radius " 
                            << radius;
                        std::string INFO = InfoStream.str();
                        ROS_INFO_STREAM(INFO);    
                        result_temp.pos.x = axis.point().x();
                        result_temp.pos.y = axis.point().y();
                        result_temp.pos.z = axis.point().z();

                        result_temp.orientation.x = axis.direction().dx();
                        result_temp.orientation.y = axis.direction().dy();
                        result_temp.orientation.z = axis.direction().dz();

                        result_temp.radius = radius;

                        resultreturn.object.push_back(result_temp);
                                                       
                    }
                    // Proceed with the next detected shape.
                    
                    it++;
                }
            i++;
            }
        } else {
            ROS_WARN("Pointcloud empty");            
        }        
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

    ShapeFittingArrayActionServer server2(node,"get_shape_array", boost::bind(&array,_1,&server2),false);

    ros::Subscriber PointCloudSubscriber;
    PointCloudSubscriber = node.subscribe<jaco::DepthImage>("/Imagepub/Depth",1,&UpdatePointCloud);

    server2.start();
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
  
    // // For visualization with Mathlab function
    //  std::ofstream myfile;
    //  myfile.open("test.txt");

    // For all pixels in depth-image(+= other than 1 to export fewer points -> Fewer calculations -> faster runtime)
    for (int x = 0; x < DepthMat.cols; x++) {
        for (int y = 0; y < DepthMat.rows; y++) {
            // If data is legal(Not 0) and below 3(Sometimes noise is present) - Export depth
            if (DepthMat.at<double>(cv::Point(x, y)) != 0 && DepthMat.at<double>(cv::Point(x, y)) < 2.5 ) {//&& DepthMat.at<double>(cv::Point(x, y)) < 3
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

                //  myfile << x << " " << y << " " << Data.first << "\n" ;
            }
        }
    }

    // myfile.close();

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

Pwn_vector VectorTest() {
    Pwn_vector result; //Return variable
    std::ifstream infile("/media/emil/USB/Samples/2.txt");
    double a, b, c, d, e, f;
    Point_with_normal PWN;

    // For every entry in list
    while (infile >> a >> b >> c >> d >> e >> f) {
        
        PWN.first = { a, b, c };
        PWN.second = { d, e, f };
        result.push_back(PWN);
    }

    return result;
}

rs2::pipeline InitiateRealsense(){
    // Create config object, and enable stream of depth data.
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_DEPTH, 480, 270, RS2_FORMAT_Z16, 60);
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
    double w = 0.777;
    double h = 0.734;

    cv::Rect roi;
    roi.x = depth_mat.cols * ((1-w)/2);
    roi.y = depth_mat.rows * ((1-h)/2);
    roi.width = depth_mat.cols * w;
    roi.height = depth_mat.rows * h;
    
    depth_mat = depth_mat(roi);

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
    //cv::imshow("s", Blob);
    return depth_mat;
}

Pwn_vector EstimateNormals(Pwn_list points, int nb_neighbors){
    
    CGAL::pca_estimate_normals<Concurrency_tag>(
            points.begin(),
            points.end(),
            CGAL::First_of_pair_property_map<Point_with_normal>(),
            CGAL::Second_of_pair_property_map<Point_with_normal>(),
            nb_neighbors);
    
        // Convert Points from list to vector
        Pwn_vector vector = List2Vector(points);

        return vector;
}

Efficient_ransac::Shape_range PerformShapeDetection(Efficient_ransac *ransac, Pwn_vector input){
// Provide input data.
    ransac->set_input(input);

    // Register cylinder shapes via template method.
    ransac->add_shape_factory<Cylinder>();

    // Set Ransac parameters
    CGAL::Shape_detection_3::Efficient_RANSAC<Traits>::Parameters parameters;
    parameters.probability = 0.0000005;          // Sets probability to miss the largest primitive at each iteration.
    parameters.min_points = 0.5*input.size();   // Min amount of points within each detected cylinder
    parameters.epsilon = 0.005;                  // Maximum acceptable euclidian distance between a point and a shape
    parameters.cluster_epsilon = 0.01;           // Maximum acceptable euclidian distance between points which are assumed to be neighbors
    parameters.normal_threshold = 0.85;          // Sets maximum normal deviation. // 0.9 < dot(surface_normal, point_normal); 

    // Detect registered shapes with the customized parameters.
    ransac->detect(parameters);
    
    // Print number of detected shapes and unassigned points.
    std::ostringstream InfoStream;
    
    InfoStream << ransac->shapes().end() - ransac->shapes().begin()
        << " detected shapes, "
        << ransac->number_of_unassigned_points()
        << " unassigned points, "
        << input.size()
        << " available points.";
    
    std::string INFO = InfoStream.str();
    ROS_INFO_STREAM(INFO);

    Efficient_ransac::Shape_range shapes = ransac->shapes();

    return shapes;
 }

