#ifndef TD_NAVIGATION_HPP__
#define TD_NAVIGATION_HPP__

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <hw_interface/base_interface.hpp>

#include <boost/scoped_ptr.hpp>
#include <math.h>

#include <hw_interface_plugin_timedomain/Range_Request.h>
#include <hw_interface_plugin_timedomain/RCM_Range_Info.h>


namespace td_navigation
{
  //class that let's us get variables from callbacks mostly
  class listener
  {
  public:
    listener();

    bool confirmed;
    int* count;
    int selector;

    double rad0_DistL;
    double rad0_DistR;

    double rad1_DistL;
    double rad1_DistR;

    void radCallBack(const hw_interface_plugin_timedomain::RCM_Range_Info::ConstPtr &msg);
  };
}

#endif //TD_NAVIGATION_HPP__
