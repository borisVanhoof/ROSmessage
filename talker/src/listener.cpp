#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <geometry_msgs/Pose.h>

using namespace std;

void help(void)
{
  // how i see the end result of the aruco node
  // -v to visualise the received image
  cout << "Usage: talker -v [optional]" << endl;
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  // publisher
  ros::NodeHandle np; 
  ros::Publisher pose_pub = np.advertise<geometry_msgs::Pose>("aruco/pose", 1); // publisher;

  try
  {
    cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
    cv::waitKey(30);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }

  /* add aruco algorithm here */
  /* for now send dummy data */ 

  float x=1, y=2, z=3;

  geometry_msgs::Pose pose_msg;

  pose_msg.position.x = x;
  pose_msg.position.y = y;
  pose_msg.position.z = z;

  pose_pub.publish(pose_msg); // publish coordinates
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");

  ros::NodeHandle nl; // listener

  cv::namedWindow("view");
  cv::startWindowThread();

  image_transport::ImageTransport it(nl);
  image_transport::Subscriber sub = it.subscribe("camera/image", 1, imageCallback);

  ros::spin();
  cv::destroyWindow("view");
}
