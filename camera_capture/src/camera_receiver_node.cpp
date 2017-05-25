#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <boost/shared_ptr.hpp>
#include <iostream>

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from <%s> to <bgr8>", msg->encoding.c_str());
  }
   //std::cout << (cv_ptr->image).cols << std::endl;
   //std::cout << (cv_ptr->image).rows << std::endl;
  cv::imshow("view", cv_ptr->image);
  cv::waitKey(3);
}

int main(int argc, char** argv)
{
  //std::string hostname = "128.217.227.41";
  //std::string hostname = "192.168.1.13";
  //std::string hostname = "wvuirl-X202E";
  //std::string hostname = "192.168.1.173";
  std::string hostname = "192.168.1.54";
  //std::string hostname = "192.168.1.61";

  std::map<std::string, std::string> remappings;
  
  remappings["__master"] = "http://" + hostname + ":11311";
  remappings["__hostname"] = hostname;
  
  ros::init(remappings, "video_receiver_node");

  ros::NodeHandle nh;

  cv::namedWindow("view");
  cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber subscriber = it.subscribe("camera/image", 640*480, imageCallback, ros::VoidPtr(), image_transport::TransportHints("theora"));
//.tcpNoDelay().maxDatagramSize(1000)
  ros::spin();
  cv::destroyWindow("view");
  return 0;
}
