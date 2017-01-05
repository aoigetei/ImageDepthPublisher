#define NOMINMAX


/******************************************************
NOTE: Requires ROS to run

Gets the depth data from the ZED Camera and publishes
to the quad/depth topic
*******************************************************/

#include <iomanip>
#include <signal.h>
#include <iostream>
#include <limits>
#include <thread>

#include <ros/ros.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "std_msgs/Float32MultiArray.h"

#include <zed/Camera.hpp>

#ifndef _SL_JETSON_   // defined in zed/utils/GlobalDefines.hpp --> Detect if we are running under a Jetson TK1 or TX1
#include <opencv2/core/utility.hpp>
#endif

using namespace sl::zed;
using namespace std;

typedef struct SaveParamStruct {
    sl::POINT_CLOUD_FORMAT PC_Format;
    sl::DEPTH_FORMAT Depth_Format;
    std::string saveName;
    bool askSavePC;
    bool askSaveDepth;
    bool stop_signal;
} SaveParam;

sl::zed::Camera * zed_ptr;
SaveParam *param;


int main(int argc, char **argv) {

    ros::init(argc, argv, "main");
    ros::NodeHandle nh;

    std_msgs::Float32MultiArray array;

    sl::zed::Camera* zed = new sl::zed::Camera(sl::zed::VGA);
    int nbFrames = 0;
    sl::zed::MODE depth_mode = sl::zed::MODE::PERFORMANCE;

    //Publisher for the depth data - tell it to buffer up to 100 frames
    ros::Publisher depth_pub = nh.advertise<std_msgs::Float32MultiArray>("quad/depth", 1000);

    //Set rate to 10Hz
    ros::Rate rate(10);

    sl::zed::InitParams parameters;
    parameters.mode = depth_mode;
    parameters.unit = sl::zed::UNIT::METER;
    parameters.verbose = 1;


    sl::zed::ERRCODE err = zed->init(parameters);
    zed_ptr = zed;
    //ERRCODE display
    cout << errcode2str(err) << endl;

    //Quit if an error occurred
    if (err != sl::zed::ERRCODE::SUCCESS) {
        delete zed;
        return 1;
    }


    int width = zed_ptr->getImageSize().width;
    int height = zed_ptr->getImageSize().height;
    cv::Size size(width, height); // image size
    cv::Mat depthDisplay(size, CV_8UC1); // normalized depth to display
    cv::Mat imageDisplay(size, CV_8UC1);



    int depth_clamp = 15;
    zed_ptr->setDepthClampValue(depth_clamp);

    int mode_PC = 0;
    int mode_Depth = 0;

    param = new SaveParam();
    param->askSavePC = false;
    param->askSaveDepth = false;
    param->stop_signal = false;
    param->PC_Format = static_cast<sl::POINT_CLOUD_FORMAT> (mode_PC);
    param->Depth_Format = static_cast<sl::DEPTH_FORMAT> (mode_Depth);

    while(ros::ok()){

      zed_ptr->grab(sl::zed::SENSING_MODE::FILL, 1, 1);

	//retrieve the raw matrix with the depth values from the ZED
	sl::zed::Mat rawMat = zed_ptr->retrieveMeasure(sl::zed::MEASURE::DEPTH);

	//use these lines if you want to display the raw depth from the ZED
	slMat2cvMat(rawMat).copyTo(depthDisplay);
	
	//get the matrix of the normalized depth image
	sl::zed::Mat norm_rawMat = zed_ptr->normalizeMeasure(sl::zed::MEASURE::DEPTH);
	slMat2cvMat(norm_rawMat).copyTo(imageDisplay);

	array.data.clear();

	//Printout our depth array
	for(int i=0; i < depthDisplay.rows; i++){
		for(int j=0; j < depthDisplay.cols; j++){
			array.data.push_back(depthDisplay.at<float>(i, j));
		}
	}


	//publish the depth array for the current frame
	depth_pub.publish(array);
    }

    delete zed_ptr;

    return 0;
}
