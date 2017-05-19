#ifndef TD_NAVIGATION_HPP__
#define TD_NAVIGATION_HPP__

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <hw_interface/base_interface.hpp>
#include <vector>

#include <boost/scoped_ptr.hpp>
#include <math.h>
#include <algorithm>

#include <hw_interface_plugin_timedomain/Range_Request.h>
#include <hw_interface_plugin_timedomain/RCM_Range_Info.h>
#include <messages/NavFilterOut.h>
#include <td_navigation/Average_angle.h>
#include <td_navigation/Td_navigation_Status.h>
#include <td_navigation/Running_Half_Pose.h>
#include <td_navigation/Localize.h>

#include <td_navigation/radio_nav.hpp>

namespace td_navigation
{
  //class that let's us get variables from callbacks mostly
  class worker
  {
  public:
    worker(int average_length_val, double base_station_distance_val,
           int rad_L_val, int rad_R_val, int z_estimate_val,
           int robot_length_offset_val, int mob_rad_dist_value);

    bool position_init;
    bool confirmed;
    int count;
    int selector;

    int average_length;
    double base_station_distance;
    int rad_L;
    int rad_R;
    double z_estimate;
    double robot_length_offset;
    double mob_rad_dist;

    double x, y, bearing, heading;
    radio_nav rad_nav;

    ros::Subscriber mob_rad_l_sub;
    ros::Subscriber mob_rad_r_sub;
    ros::Subscriber nav_filter_sub;

    ros::Publisher mob_rad_l_pub;
    ros::Publisher mob_rad_r_pub;

    ros::Publisher rhp_pub;
    
    ros::Publisher status_pub;

    ros::Publisher aa_p;


    std::vector < std::vector<double> > dist0_l;
    std::vector < std::vector<double> > dist0_r;
    std::vector < std::vector<double> > dist1_l;
    std::vector < std::vector<double> > dist1_r;

    int add_distance(std::vector < std::vector<double> >& distances, double range, double error);
    bool send_and_recieve(int to, hw_interface_plugin_timedomain::Range_Request& rr, ros::Publisher& rad_pub);
    void mob_rad_0_CallBack(const hw_interface_plugin_timedomain::RCM_Range_Info::ConstPtr &msg);
    void mob_rad_1_CallBack(const hw_interface_plugin_timedomain::RCM_Range_Info::ConstPtr &msg);
    void nav_filter_callback(const messages::NavFilterOut::ConstPtr &msg);
    bool srvCallBack(td_navigation::Localize::Request &req, td_navigation::Localize::Response &res);
    int set_current_pos(double x_val, double y_val, double bearing_val, double heading_val);
    double get_avg_error(std::vector< std::vector< double> >& error, int& amount_to_avg);
    double get_avg_err0_l(int amount_to_avg);
    double get_avg_err0_r(int amount_to_avg);
    double get_avg_err1_l(int amount_to_avg);
    double get_avg_err1_r(int amount_to_avg);
    double get_med_error(std::vector< std::vector< double > >& error);
    double get_med_err0_l();
    double get_med_err0_r();
    double get_med_err1_l();
    double get_med_err1_r();
    double get_avg_dist(std::vector< std::vector< double> >& dist, int& amount_to_avg);
    double get_avg_dist0_l(int amount_to_avg);
    double get_avg_dist0_r(int amount_to_avg);
    double get_avg_dist1_l(int amount_to_avg);
    double get_avg_dist1_r(int amount_to_avg);
    double get_current_heading();
    double get_current_bearing();
    double get_current_pos_x();
    double get_current_pos_y();
    void update_count();
    double smart_atan(double adj, double opp);
    int run();
    int run_half_pose();


  };

}

#endif //TD_NAVIGATION_HPP__
