#ifndef TD_NAVIGATION_HPP__
#define TD_NAVIGATION_HPP__

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <hw_interface/base_interface.hpp>

#include <boost/scoped_ptr.hpp>
#include <math.h>

#include <hw_interface_plugin_timedomain/Range_Request.h>
#include <hw_interface_plugin_timedomain/RCM_Range_Info.h>
#include <td_navigation/Average_angle.h>
#include <td_navigation/Localize.h>

#include <td_navigation/radio_nav.hpp>

namespace td_navigation
{
  //class that let's us get variables from callbacks mostly
  class worker
  {
  public:
    worker();

    bool confirmed;
    int count;
    int selector;

    int average_length;
    double dStation;
    int rad_L;
    int rad_R;

    double rad104_DistL;
    double rad104_DistR;

    double rad105_DistL;
    double rad105_DistR;

    double headings[];
    double bearings[];
    double x[];
    double y[];

    double x_sum = 0;
    double y_sum = 0;
    double head_sum = 0;
    double bear_sum = 0;
    double h_dev_sum = 0;
    double b_dev_sum = 0;

    bool send_and_recieve(int& to, hw_interface_plugin_timedomain::Range_Request& rr, ros::Publisher& rad_pub);
    void radCallBack(const hw_interface_plugin_timedomain::RCM_Range_Info::ConstPtr &msg);
    bool srvCallBack(td_navigation::Localize::Request &req, td_navigation::Localize::Response &res);
    void subtract_oldest();
    void set_current_heading(double heading);
    void set_current_bearing(double bearing);
    void set_current_pos_x(double pos_x);
    void set_current_pos_y(double pos_y);
    void update_sums(double bot_x, double bot_y);
    double get_avg_heading();
    double get_avg_bearing();
    double get_avg_x();
    double get_avg_y();


  };

}

#endif //TD_NAVIGATION_HPP__
