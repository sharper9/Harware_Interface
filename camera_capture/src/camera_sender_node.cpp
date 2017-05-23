#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "video_sender_node");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher publisher = it.advertise("camera/image", 1);

  cv::VideoCapture capture(0);

  if (!capture.isOpened())
  {
    ROS_ERROR("Video Device can not be opened");
    return 1;
  }

  cv::Mat frame;
  sensor_msgs::ImagePtr msg;

  ros::Rate loop_rate(5);
  while(nh.ok())
  {
    capture >> frame;

    if (!frame.empty())
    {
      msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
      publisher.publish(msg);
      cv::waitKey(1);
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
}
