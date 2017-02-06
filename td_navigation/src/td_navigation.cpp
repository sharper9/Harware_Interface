#include <ros/ros.h>

#include <boost/scoped_ptr.hpp>

int main(int argc, char **argv)
{
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
           ros::console::notifyLoggerLevelsChanged();
        }
    std::string node_type = "";
    if(ros::param::get("node_type", node_type)==false) node_type = "td_navigation";
    ROS_INFO("%s Start", node_type.c_str());
    ros::init(argc, argv, node_type);
    ROS_INFO(" - ros::init complete");
    ros::NodeHandle nh;
    ROS_INFO(" - node handle created");

    ros::Rate loop_rate(50);

    while(nh.ok())
    {
        //            std_msgs::String msg;

        //            rosDataSub.

        //            ROS_INFO("%s", msg.data.c_str());

        //            //make message

        //            rosDataPub.publish(msg);

        //            ros::spinOnce();


        //            ++count;
        ros::spinOnce();
        loop_rate.sleep();
    }

    ROS_DEBUG("td_navigation Closing");
    return 0;
}




//{
//      ros::NodeHandle n;

//      //ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

//      ros::Rate loop_rate(5);

//      int count = 0;
//        while (ros::ok())
//        {
//
//        }

//      return 0;
//}
