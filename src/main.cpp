

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <vector>
#include <stdio.h>
#include <stack>
#include <ctime>


#include <surface_reconstruction/vo_system.h>


#include <opencv2/opencv.hpp>
#include "opencv2/imgproc/imgproc.hpp"

//#include "stdafx.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>


#define _USE_MATH_DEFINES

/////ROS IMAGE SUBSCRIBER
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>


#include <fstream>
#include <iomanip>    // Needed for stream modifiers fixed and set precision


#include <Eigen/Dense>
using namespace Eigen;
#include <opencv2/core/eigen.hpp>


//using namespace CVD;
using namespace std;

//chadir
#include <unistd.h>
// reading a text file
#include <iostream>
#include <fstream>
#include <string>
//directorio
#include <dirent.h>

#include <boost/thread/thread.hpp>
#include <iostream>
#include <stdio.h>
#include <boost/filesystem.hpp>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include<sensor_msgs/Imu.h>


#include <thread>


//pragma omp
#include <omp.h>
//#pragma omp

//// file storage
#include <time.h>
#include "opencv2/opencv.hpp"

using namespace cv;
int main(int argc, char** argv)
{

    ros::init(argc, argv, "camera_image");
    ros::start();

//    srand ( (unsigned)time(0) );
//    omp_set_dynamic(0);
//    omp_set_nested(1);

    //Launch the Visual odometry system
   vo_system vo_system_object;


    ros::spin();
    ros::shutdown();
    return 0;
}