#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <geometry_msgs/Pose.h>

using namespace std;

void callBackFunction(const geometry_msgs::Pose& msg)
{
  ROS_INFO("I heard: [%f %f %f]\n", msg.position.x, msg.position.y, msg.position.z);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_publisher");

  ros::NodeHandle np; //publisher
  ros::NodeHandle nl; //listener

  ros::Subscriber sub = nl.subscribe("aruco/pose", 5, callBackFunction);

  image_transport::ImageTransport it(np);
  image_transport::Publisher pub = it.advertise("camera/image", 1);
  cv::Mat image = cv::imread("test.jpg", CV_LOAD_IMAGE_COLOR);
  if(!image.data)
  {
	cout << "could not open test.jpg" << endl;
	return -1;
  }

  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

  ros::Rate loop_rate(30);

  while (ros::ok())
  {
    pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}
