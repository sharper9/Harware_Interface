#ifndef TD_NAVIGATION_HPP__
#define TD_NAVIGATION_HPP__

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <hw_interface/base_interface.hpp>
#include <vector>

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

    std::vector<double> headings;
    std::vector<double> bearings;
    std::vector<double> x;
    std::vector<double> y;

    bool send_and_recieve(int& to, hw_interface_plugin_timedomain::Range_Request& rr, ros::Publisher& rad_pub);
    void radCallBack(const hw_interface_plugin_timedomain::RCM_Range_Info::ConstPtr &msg);
    bool srvCallBack(td_navigation::Localize::Request &req, td_navigation::Localize::Response &res);
    void set_current_heading(double heading);
    void set_current_bearing(double bearing);
    void set_current_pos_x(double pos_x);
    void set_current_pos_y(double pos_y);
    double get_current_heading();
    double get_current_bearing();
    double get_current_pos_x();
    double get_current_pos_y();
    void update_sums();
    double get_avg_heading();
    double get_avg_bearing();
    double get_avg_x();
    double get_avg_y();
    void update_count();
    int set_error();


  };

}

#endif //TD_NAVIGATION_HPP__
