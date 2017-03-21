#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "cvdrawingutils.h"
#include <geometry_msgs/Pose.h>
#include "aruco.h"

using namespace std;
using namespace cv;
using namespace aruco;



bool visualise = false;
string TheIntrinsicFile("");
CameraParameters TheCameraParameters;
float TheMarkerSize = -1;



// function prototypes
bool readArguments( int argc, char *argv[]);



void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  // publisher
  ros::NodeHandle np; 
  ros::Publisher pose_pub = np.advertise<geometry_msgs::Pose>("aruco/pose", 1); // publisher;

  // aruco variables
  MarkerDetector MDetector;
  vector<Marker> TheMarkers;
  double ThresParam1, ThresParam2;
  int iThresParam1, iThresParam2;
  cv::Mat rot_mat(3,3, cv::DataType<float>::type);
  const float p_off = CV_PI;
  const float r_off = CV_PI/2;
  const float y_off = CV_PI/2;

  cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
  if(!image.data)
  {
  	ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
	return;
  }

  //camera parameters if passed
  if(TheIntrinsicFile != "")
  {
	try
	{
		TheCameraParameters.readFromXMLFile(TheIntrinsicFile);
		TheCameraParameters.resize(image.size());
	}
	catch ( cv::Exception &e )
	{
		const char* err_msg = e.what();
		ROS_ERROR("%s\n", err_msg);
		return;
	}		
  }

  // Detection of markers in the image
  MDetector.detect(image, TheMarkers, TheCameraParameters, TheMarkerSize);

  if(TheMarkers.size()<0)
  {
	ROS_INFO("no markers found!\n");
	return;
  }

  if(TheCameraParameters.isValid() && visualise == true) 
  {
	for(size_t i=0; i<TheMarkers.size(); i++)
	{
		CvDrawingUtils::draw3dCube(image, TheMarkers[i], TheCameraParameters);
		CvDrawingUtils::draw3dAxis(image, TheMarkers[i], TheCameraParameters);		
	}
  }
  else
  {
    if(!TheCameraParameters.isValid())
	ROS_ERROR("camera parameters not valid!\n");
  }

  for(size_t i=0; i<TheMarkers.size(); i++)
  {
	TheMarkers[i].draw(image, cv::Scalar(0,200,0), 1, true);
	ROS_INFO("id: %d\n", TheMarkers[i].id);

	try
	{
		TheMarkers[i].calculateExtrinsics(TheMarkerSize, TheCameraParameters);
	}
	catch ( cv::Exception &e )
	{
		const char* err_msg = e.what();
		ROS_ERROR("%s\n", err_msg);
	}
  }

  float x_t, y_t, z_t;
  float roll, yaw, pitch;

  x_t = -TheMarkers[0].Tvec.at<Vec3f>(0,0)[0];
  y_t = -TheMarkers[0].Tvec.at<Vec3f>(0,0)[1];
  z_t = -TheMarkers[0].Tvec.at<Vec3f>(0,0)[2];
	
  cv::Rodrigues(TheMarkers[0].Rvec, rot_mat);

  pitch = -atan2(rot_mat.at<float>(2,0), rot_mat.at<float>(2,1));
  yaw = acos(rot_mat.at<float>(2,2));
  roll = -atan2(rot_mat.at<float>(0,2), rot_mat.at<float>(1,2));

  ROS_INFO("pitch: %f yaw: %f roll: %f\n", (pitch-p_off)*(180/CV_PI), (yaw-y_off)*(180/CV_PI), (roll-r_off)*(180/CV_PI));

  if(visualise)
  {
	cv::resize(image, image, cv::Size(), 0.5, 0.5);
	cv::imshow("view", image);
    	cv::waitKey(30);
  }

  /* now publish the poses */

  geometry_msgs::Pose pose_msg;

  pose_msg.position.x = x_t;
  pose_msg.position.y = y_t;
  pose_msg.position.z = z_t;

  pose_pub.publish(pose_msg); // publish coordinates
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");

  /* read arguments */
  if(readArguments(argc, argv) == false)
  {
	return -1;  
  }

  ros::NodeHandle nl; // listener

  if(visualise)
  {
  	// create gui
  	cv::namedWindow("view");
  	cv::startWindowThread();
  }

  image_transport::ImageTransport it(nl);
  image_transport::Subscriber sub = it.subscribe("camera/image", 1, imageCallback);

  ros::spin();

  if(visualise)
  {
  	// destroy gui
	cv::destroyWindow("view");
  }
}

bool readArguments( int argc, char *argv[] )
{

	vector<string> filename;

	if(argc<2)
	{
		ROS_ERROR("Not enough arguments, usage:\n");
		ROS_INFO("[.xml or .yml] path to the intrinstics file\n [marker size in meters (0.17)]\n [-v (optional)] to create gui\n");
		return false;
	}

	if(argc>=2)
		TheIntrinsicFile = argv[1];
	
	if(argc>=3)
		TheMarkerSize = atof(argv[2]);
	
	if(argc == 3)
		ROS_INFO("You need marker size for 3d info...\n");

	if(argc >= 4)
		visualise = true;
	else
		ROS_INFO("Use -v to view the image [optional]\n");

	return true;
}
