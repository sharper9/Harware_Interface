#include <td_navigation/td_navigation.h>

#define PI 3.14159265
//#define ERRNUM -200000000.1

td_navigation::worker::worker(int average_length_val, double base_station_distance_val,
                              int rad_L_val, int rad_R_val, int z_estimate_val,
                              int robot_length_offset_val, int mob_rad_dist_value)
{

  confirmed = 0;
  count = 0;
  selector = 0 ;

  heading = 0;
  bearing = 0;

  average_length = average_length_val;
  base_station_distance = base_station_distance_val;
  rad_L = rad_L_val;
  rad_R = rad_R_val;
  z_estimate = z_estimate_val;
  robot_length_offset = robot_length_offset_val;
  mob_rad_dist = mob_rad_dist_value;

  dist0_l.reserve(average_length);
  dist0_r.reserve(average_length);
  dist1_l.reserve(average_length);
  dist1_r.reserve(average_length);

  rad_nav.create_base_radio(0, -1.0 * base_station_distance/2.0, 0);
  rad_nav.create_base_radio(0, base_station_distance/2.0, 0);
  rad_nav.create_mobile_radio();
  rad_nav.create_mobile_radio();

  ros::NodeHandle nh;
  ROS_INFO(" - node handle created");

  ros::ServiceServer service = nh.advertiseService("localize", &td_navigation::worker::srvCallBack, this);
  ROS_INFO("td_navigation node ready to Localize");

  //subscriber for recieving messages
  mob_rad_l_sub = nh.subscribe("/radio104/data", 5, &td_navigation::worker::mob_rad_0_CallBack, this);
  mob_rad_r_sub = nh.subscribe("/radio105/data", 5, &td_navigation::worker::mob_rad_1_CallBack, this);

  nav_filter_sub = nh.subscribe("/navigation/navigationfilterout/navigationfilterout", 1, &td_navigation::worker::nav_filter_callback, this);


  //publisher for sending a message to timedomain serial to get a range request
  mob_rad_l_pub = nh.advertise<hw_interface_plugin_timedomain::Range_Request>("/radio104/cmd", 5);
  mob_rad_r_pub = nh.advertise<hw_interface_plugin_timedomain::Range_Request>("/radio105/cmd", 5);

  rhp_pub = nh.advertise<td_navigation::Running_Half_Pose>("/Half_Pose", 1);

  //publisher for getting average angle measurements
  aa_p = nh.advertise<td_navigation::Average_angle>("/average_angles", 1);

  while(nh.ok())
  {
    run_half_pose();
  }
}


bool td_navigation::worker::send_and_recieve(int to, hw_interface_plugin_timedomain::Range_Request& rr, ros::Publisher& rad_pub){
  int wait = 0;
  int timeout = 0;

  //TODO: change back to about 5000
  ros::Rate loop_rate(5000);


  while(!confirmed){
    rr.radio_id_to_target = to;
    rad_pub.publish(rr);
    while(!confirmed && wait < 100){
      ros::spinOnce();
      loop_rate.sleep();
      wait ++;
    }
    wait = 0;
    if (timeout >= 10){
      return false;
    }
    timeout++;
  }
  confirmed = false;
  return true;
}

int td_navigation::worker::add_distance(std::vector < std::vector<double> >& distances,
                                        double range, double error){
  std::vector< std::vector<double> >::iterator it = distances.begin();
  std::vector<double> range_and_error;
  range_and_error.push_back(range);
  range_and_error.push_back(error);

  if(distances.size() == average_length){
    distances.pop_back();
  }

  distances.insert(it, range_and_error);

  return 0;
}

void td_navigation::worker::mob_rad_0_CallBack(const hw_interface_plugin_timedomain::RCM_Range_Info::ConstPtr &msg){
    ROS_INFO("TDRR Navigation Callback");
    //check if the radio was busy
    if (msg->busy == true){
        ROS_WARN("TDRR was busy");
        return;
    }
    //check if the radio range has failed
    if (msg->failed == true){
        ROS_WARN("TDRR request has failed!");
        return;

    }

    //Selector for assigning the distance to the correct value
    if(msg->msgID >=0 && msg->msgID < 32000 ){

      //reading 0 to l
        if(msg->msgID == count + selector && selector == 0){
            ROS_INFO("Reading from Mob_0 to Base_0");
            ROS_DEBUG("Precise Range Measure: %d", msg->PRM);
            add_distance(dist0_l, (double)msg->PRM, (double)msg->PRMError);
            confirmed = true;
            return;

        //reading 1 to l
      }else if(msg-> msgID == count + selector && selector == 1){
          ROS_INFO("Reading from Mob_0 to Base_1");
          ROS_DEBUG("Precise Range Measure: %d", msg->PRM);
          add_distance(dist0_r, msg->PRM, msg->PRMError);
          confirmed = true;
          return;

        //no match
        }else{
            ROS_WARN("TDRR MsgID mismatched!");
            return;
        }

    }else{
        ROS_WARN("TDRR: Recieved MsgID never assigned!");
    }


}

void td_navigation::worker::mob_rad_1_CallBack(const hw_interface_plugin_timedomain::RCM_Range_Info::ConstPtr &msg){
    ROS_INFO("TDRR Navigation Callback");
    //check if the radio was busy
    if (msg->busy == true){
        ROS_WARN("TDRR was busy");
        return;
    }
    //check if the radio range has failed
    if (msg->failed == true){
        ROS_WARN("TDRR request has failed!");
        return;

    }



    //Selector for assigning the distance to the correct value
    if(msg->msgID >=0 && msg->msgID < 32000 ){

      //reading 0 to r
      if(msg-> msgID == count + selector && selector == 2){
            ROS_INFO("Reading from Mob_0 to Base_1");
            ROS_DEBUG("Precise Range Measure: %d", msg->PRM);
            add_distance(dist1_l, msg->PRM, msg->PRMError);
            confirmed = true;
            return;

      //reading 1 to r
    }else if(msg-> msgID == count + selector && selector == 3){
          ROS_INFO("Reading from Mob_1 to Base_1");
          ROS_DEBUG("Precise Range Measure: %d", msg->PRM);
          add_distance(dist1_r, msg->PRM, msg->PRMError);
          confirmed = true;
          return;

        //no match
        }else{
            ROS_WARN("TDRR MsgID mismatched!");
            return;
        }

    }else{
        ROS_WARN("TDRR: Recieved MsgID never assigned!");
    }


}

void td_navigation::worker::nav_filter_callback(const messages::NavFilterOut::ConstPtr &msg){
  heading = msg->heading*PI/180.0;
  bearing = msg->bearing*PI/180.0;
}




bool td_navigation::worker::srvCallBack(td_navigation::Localize::Request &req,
                                        td_navigation::Localize::Response &res){
dist0_l.clear();
dist0_r.clear();
dist1_l.clear();
dist1_r.clear();

int max = average_length;

if(req.average_length <= average_length){
  max = req.average_length;
}

for(int i = 0; i < max; i++){
  if (run() != 0){
    res.fail = true;
    return false;
  }
}

//service response
res.x = x/1000.0;
res.y = y/1000.0;
res.heading = heading;
res.bearing = bearing;
res.avg_error = get_avg_error(req.average_length);
res.max_error = get_max_error(req.average_length);
res.min_error = get_min_error(req.average_length);
res.left_interference = false;
res.right_interference = false;
if(get_avg_error_left(req.average_length) > 13){
  res.left_interference = true;
}

if(get_avg_error_right(req.average_length) > 13){
  res.right_interference = true;
}

res.fail = false;

return true;
}

double td_navigation::worker::get_avg_dist(std::vector< std::vector< double> >& dist, int& amount_to_avg){
  double top = 0;
  double bottom = 0;
  int max = 0;

  if(amount_to_avg <= dist.size()){
    max = amount_to_avg;
  }else{
    max = dist.size();
  }
  for(int i = 0; i < max; i++){

    top += dist[i][0] * (65535 - dist[i][1]);
    bottom += (65535 - dist[i][1]);
  }

  if(bottom == 0){
    return 0;
  }

  return top/bottom;

}

double td_navigation::worker::get_avg_dist0_l(int amount_to_avg){
  return get_avg_dist(dist0_l, amount_to_avg);
}

double td_navigation::worker::get_avg_dist0_r(int amount_to_avg){
  return get_avg_dist(dist0_r, amount_to_avg);

}

double td_navigation::worker::get_avg_dist1_l(int amount_to_avg){
  return get_avg_dist(dist1_l, amount_to_avg);
}

double td_navigation::worker::get_avg_dist1_r(int amount_to_avg){
  return get_avg_dist(dist1_r, amount_to_avg);
}

int td_navigation::worker::set_current_pos(double x_val, double y_val,
                                           double bearing_val, double heading_val){
  x = x_val;
  y = y_val;
  bearing = bearing_val;
  heading = heading_val;
  return 0;
}

double td_navigation::worker::get_avg_error(int amount_to_avg){
  double sum = 0;
  int max = dist1_r.size();
  if(amount_to_avg < dist1_r.size()){
    max = amount_to_avg;
  }
  for(int i = 0; i < max; i++){
    sum += dist0_l[i][1] + dist0_r[i][1] + dist1_l[i][1] + dist1_r[i][1];
  }

  return sum/(max * 4);
}

double td_navigation::worker::get_avg_error_left(int amount_to_avg){
  double sum = 0;
  int max = dist1_r.size();
  if(amount_to_avg < dist1_r.size()){
    max = amount_to_avg;
  }
  for(int i = 0; i < max; i++){
    sum += dist0_l[i][1] + dist0_r[i][1];
  }

  return sum/(max * 4);
}

double td_navigation::worker::get_avg_error_right(int amount_to_avg){
  double sum = 0;
  int max = dist1_r.size();
  if(amount_to_avg < dist1_r.size()){
    max = amount_to_avg;
  }
  for(int i = 0; i < max; i++){
    sum += dist1_l[i][1] + dist1_r[i][1];
  }

  return sum/(max * 4);
}

double td_navigation::worker::get_max_error(int amount_to_avg){
  double max_err = 0;
  int max = dist1_r.size();
  if(amount_to_avg < dist1_r.size()){
    max = amount_to_avg;
  }
  for(int i = 0; i < max; i++){
    max_err = std::max(dist0_l[i][1], max_err);
    max_err = std::max(dist0_r[i][1], max_err);
    max_err = std::max(dist1_l[i][1], max_err);
    max_err = std::max(dist1_r[i][1], max_err);
  }

  return max_err;
}

double td_navigation::worker::get_min_error(int amount_to_avg){
  double min_err = 0;
  int max = dist1_r.size();
  if(amount_to_avg < dist1_r.size()){
    max = amount_to_avg;
  }
  for(int i = 0; i < max; i++){
    min_err = std::min(dist0_l[i][1], min_err);
    min_err = std::min(dist0_r[i][1], min_err);
    min_err = std::min(dist1_l[i][1], min_err);
    min_err = std::min(dist1_r[i][1], min_err);
  }

  return min_err;
}


double td_navigation::worker::get_current_heading(){
  return heading;
}

double td_navigation::worker::get_current_bearing(){
  return bearing;
}

double td_navigation::worker::get_current_pos_x(){
  return x;
}

double td_navigation::worker::get_current_pos_y(){
  return y;
}

void td_navigation::worker::update_count(){
  count += 10;
}

double td_navigation::worker::smart_atan(double adj, double opp){
      if(adj == 0){
        if(opp > 0){
          return -PI/2.0;
        }else if (opp <= 0){
          return PI/2.0;
        }
      }else if(opp == 0){
        if(adj >= 0){
          return 0;
        }else if(adj < 0){
          return PI;
        }
      }else if(opp > 0){
        if(adj > 0){
          return -1.0 * atan(opp/adj);
        }else if(adj < 0){
          return -1.0 * PI - atan(opp/adj);
        }
      }else if(opp < 0){
        if(adj > 0){
          return -1.0 * atan(opp/adj);
        }else if(adj < 0){
          return PI - atan(opp/adj);
        }
      }
      return -500;
}


int td_navigation::worker::run(){

  ROS_INFO("Count: %d", count);

  hw_interface_plugin_timedomain::Range_Request rr;
  rr.send_range_request = true;


  confirmed = false;
  selector = 0;
  rr.msgID = count + selector;
  //range request and response from 104 to 101
  if (send_and_recieve(rad_L, rr, mob_rad_l_pub) == false){
    //call a function to tell about the malfunction
    ROS_DEBUG("We didn't get a response in time!");
  }

  selector = 1;
  rr.msgID = count + selector;
  //range request and response from 104 to 106
  if (send_and_recieve(rad_R, rr, mob_rad_l_pub) == false){
    //call a function to tell about the malfunction
    ROS_DEBUG("We didn't get a response in time!");
  }

  selector = 2;
  rr.msgID = count + selector;
  //range request and response from 105 to 101
  if (send_and_recieve(rad_L, rr, mob_rad_r_pub) == false){
    //call a function to tell about the malfunction
    ROS_DEBUG("We didn't get a response in time!");
  }

  selector = 3;
  rr.msgID = count + selector;
  //range request and response from 105 to 106
  if (send_and_recieve(rad_R, rr, mob_rad_r_pub) == false){
    //call a function to tell about the malfunction
    ROS_DEBUG("We didn't get a response in time!");
  }

  std::vector<double> distance_to_base_rads;


  distance_to_base_rads.push_back(get_avg_dist0_l(average_length));
  distance_to_base_rads.push_back(get_avg_dist0_r(average_length));
  rad_nav.update_mobile_radio(0 ,distance_to_base_rads);


  distance_to_base_rads[0] = get_avg_dist1_l(average_length);
  distance_to_base_rads[1] = get_avg_dist1_r(average_length);
  rad_nav.update_mobile_radio(1 ,distance_to_base_rads);




  if (rad_nav.triangulate_2Base(z_estimate) == 0){


    heading = smart_atan( (rad_nav.get_mobile_radio_coordinate(1,1) - rad_nav.get_mobile_radio_coordinate(0,1)),
                          (rad_nav.get_mobile_radio_coordinate(1,0) - rad_nav.get_mobile_radio_coordinate(0,0)) );

    x = ( (rad_nav.get_mobile_radio_coordinate(0,0) + rad_nav.get_mobile_radio_coordinate(1,0)) / 2.0 ) +
         (cos(heading) * robot_length_offset);

    y = ( (rad_nav.get_mobile_radio_coordinate(0,1) + rad_nav.get_mobile_radio_coordinate(1,1)) / 2.0 ) +
         (sin(heading) * robot_length_offset);

    bearing = -smart_atan(x,y);


    ROS_DEBUG("Head: %lf, Bear: %lf, x: %lf, y: %lf", heading * 180.0 / PI, bearing * 180.0 / PI, x, y );


    td_navigation::Average_angle aa;

    aa.heading = heading * 180.0 / PI;
    aa.bearing = bearing * 180.0 / PI;
    aa.x = x/1000.0;
    aa.y = y/1000.0;
    aa.rad0toL = get_avg_dist0_l(average_length);
    aa.rad0toR = get_avg_dist0_r(average_length);
    aa.rad1toL = get_avg_dist1_l(average_length);
    aa.rad1toR = get_avg_dist1_r(average_length);

    aa_p.publish(aa);


  }else {
    ROS_DEBUG("Problem encountered with triangulation!");
    return -1;

  }

  //update count
  update_count();
  if(count >= 32000){
    count = 0;
}

return 0;

}

int td_navigation::worker::run_half_pose(){

  ROS_INFO("Count: %d", count);

  hw_interface_plugin_timedomain::Range_Request rr;
  rr.send_range_request = true;


  confirmed = false;
  selector = 0;
  rr.msgID = count + selector;
  //range request and response from 104 to 101
  if (send_and_recieve(rad_L, rr, mob_rad_l_pub) == false){
    //call a function to tell about the malfunction
    ROS_DEBUG("We didn't get a response in time!");
  }

  selector = 1;
  rr.msgID = count + selector;
  //range request and response from 104 to 106
  if (send_and_recieve(rad_R, rr, mob_rad_l_pub) == false){
    //call a function to tell about the malfunction
    ROS_DEBUG("We didn't get a response in time!");
  }

  std::vector<double> distance_to_base_rads;


  distance_to_base_rads.push_back(get_avg_dist0_l(1));
  distance_to_base_rads.push_back(get_avg_dist0_r(1));
  rad_nav.update_mobile_radio(0 ,distance_to_base_rads);

  if (rad_nav.triangulate_2Base(z_estimate) == 0){

    double angle_2 = heading + PI/2.0;
    double back_x, back_y;

    back_x = rad_nav.get_mobile_radio_coordinate(0,0) + cos(angle_2) * mob_rad_dist / 2.0;

    back_y = rad_nav.get_mobile_radio_coordinate(0,1) + sin(angle_2) * mob_rad_dist / 2.0;

    x = back_x + (cos(heading) * robot_length_offset);

    y = back_y + (sin(heading) * robot_length_offset);


    ROS_DEBUG("Head: %lf, Bear: %lf, x: %lf, y: %lf", heading * 180.0 / PI, bearing * 180.0 / PI, x, y );


    td_navigation::Running_Half_Pose rhp;

    rhp.x = x/1000.0;
    rhp.y = y/1000.0;

    rhp_pub.publish(rhp);


  }else {
    ROS_DEBUG("Problem encountered with triangulation!");
    return -1;
  }

  //update count
  update_count();
  if(count >= 32000){
    count = 0;
}

return 0;

}

/*
td_navigation::worker::worker(int average_length_val, double base_station_distance_val,
                              int rad_L_val, int rad_R_val, int z_estimate_val,
                              int robot_length_offset_val, int mob_rad_dist_value)
*/

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

  //TODO: these values should be launch params
  td_navigation::worker worker(10, 725.4875, 101, 106, 0, 673, 546);

  ROS_DEBUG("td_navigation closing");
  return 0;
}
