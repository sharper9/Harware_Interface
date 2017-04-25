#include <td_navigation/td_navigation.h>

#define PI 3.14159265
//#define ERRNUM -200000000.1

td_navigation::worker::worker(int average_length_val, double base_station_distance_val,
                              int rad_L_val, int rad_R_val, int z_estimate_val,
                              int robot_length_offset_val)
{

  confirmed = 0;
  count = 0;
  selector = 0 ;

  average_length = average_length_val;
  base_station_distance = base_station_distance_val;
  rad_L = rad_L_val;
  rad_R = rad_R_val;
  z_estimate = z_estimate_val;
  robot_length_offset = robot_length_offset_val;

  // rad104_DistL = 0;
  // rad104_DistR = 0;
  //
  // rad105_DistL = 0;
  // rad105_DistR = 0;
  //
  //
  //
  // headings.reserve(average_length);
  // bearings.reserve(average_length);
  // x.reserve(average_length);
  // y.reserve(average_length);

  dist0_l.reserve(average_length);
  dist0_r.reserve(average_length);
  dist1_l.reserve(average_length);
  dist1_r.reserve(average_length);

}


bool td_navigation::worker::send_and_recieve(int to, hw_interface_plugin_timedomain::Range_Request& rr, ros::Publisher& rad_pub){
  int wait = 0;
  int timeout = 0;
  //TODO: change back to about 200
  ros::Rate loop_rate(2);

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
  std::vector<double> range_and_error(range, error);

  if(distances.size() == average_length){
    distances.pop_back();
  }

  distances.insert(it, range_and_error);

  return 0;
}

void td_navigation::worker::radCallBack(const hw_interface_plugin_timedomain::RCM_Range_Info::ConstPtr &msg){
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
        if(msg->msgID == count && selector == 0){
            ROS_INFO("Reading from rad0 to leftRad");
            ROS_DEBUG("Precise Range Measure: %d", msg->PRM);
            add_distance(dist0_l, msg->PRM, msg->PRMError);
            confirmed = true;
            return;

        //reading 0 to r
        }else if(msg-> msgID == count && selector == 1){
            ROS_INFO("Reading from rad0 to RightRad");
            ROS_DEBUG("Precise Range Measure: %d", msg->PRM);
            add_distance(dist0_r, msg->PRM, msg->PRMError);
            confirmed = true;
            return;

        //reading 1 to l
        }else if(msg-> msgID == count && selector == 2){
            ROS_INFO("Reading from rad1 to leftRad");
            ROS_DEBUG("Precise Range Measure: %d", msg->PRM);
            add_distance(dist1_l, msg->PRM, msg->PRMError);
            confirmed = true;
            return;

        //reading 1 ot r
        }else if(msg-> msgID == count && selector == 3){
            ROS_INFO("Reading from rad1 to RightRad");
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



bool td_navigation::worker::srvCallBack(td_navigation::Localize::Request &req,
                                        td_navigation::Localize::Response &res){

//service response
res.x = this->x;
res.y = this->y;
res.heading = this->heading;
res.bearing = this->bearing;
res.fail = false;

return true;
}

double td_navigation::worker::get_avg_dist0_l(int amount_to_avg){
  double top = 0;
  double bottom = 0;
  int max = 0;

  if(amount_to_avg <= dist0_l.size()){
    max = amount_to_avg;
  }else{
    max = dist0_l.size();
  }

  for(int i = 0; i < max; i++){
    if (dist0_l[i][1] == 0){
      top += dist0_l[i][0];
      bottom += 1;
    }else{
      top += dist0_l[i][0] * (1/dist0_l[i][1]);
      bottom += 1/dist0_l[i][1];
    }
  }

  if(bottom == 0){
    return 0;
  }

  return top/bottom;

}

double td_navigation::worker::get_avg_dist0_r(int amount_to_avg){
  double top = 0;
  double bottom = 0;
  int max = 0;

  if(amount_to_avg <= dist0_r.size()){
    max = amount_to_avg;
  }else{
    max = dist0_r.size();
  }

  for(int i = 0; i < max; i++){
    if (dist0_r[i][1] == 0){
      top += dist0_r[i][0];
      bottom += 1;
    }else{
      top += dist0_r[i][0] * (1/dist0_r[i][1]);
      bottom += 1/dist0_r[i][1];
    }
  }

  if(bottom == 0){
    return 0;
  }

  return top/bottom;

}

double td_navigation::worker::get_avg_dist1_l(int amount_to_avg){
  double top = 0;
  double bottom = 0;
  int max = 0;

  if(amount_to_avg <= dist1_l.size()){
    max = amount_to_avg;
  }else{
    max = dist1_l.size();
  }

  for(int i = 0; i < max; i++){
    if (dist1_l[i][1] == 0){
      top += dist0_l[i][0];
      bottom += 1;
    }else{
      top += dist1_l[i][0] * (1/dist1_l[i][1]);
      bottom += 1/dist1_l[i][1];
    }
  }

  if(bottom == 0){
    return 0;
  }

  return top/bottom;

}

double td_navigation::worker::get_avg_dist1_r(int amount_to_avg){
  double top = 0;
  double bottom = 0;
  int max = 0;

  if(amount_to_avg <= dist1_r.size()){
    max = amount_to_avg;
  }else{
    max = dist1_r.size();
  }

  for(int i = 0; i < max; i++){
    if (dist1_r[i][1] == 0){
      top += dist1_r[i][0];
      bottom += 1;
    }else{
      top += dist1_r[i][0] * (1/dist1_r[i][1]);
      bottom += 1/dist1_r[i][1];
    }
  }

  if(bottom == 0){
    return 0;
  }

  return top/bottom;

}

int td_navigation::worker::set_current_pos(double x_val, double y_val,
                                           double bearing_val, double heading_val){
  x = x_val;
  y = y_val;
  bearing = bearing_val;
  heading = heading_val;
  return 0;
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
  count++;
}

double smart_atan(double x, double y);

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

  td_navigation::worker worker(20, 1930.4, 101, 106, 0, 750);
  radio_nav rad_nav;
  rad_nav.create_base_radio(0, -1.0 * worker.base_station_distance/2.0, 0);
  rad_nav.create_base_radio(0, worker.base_station_distance/2.0, 0);
  rad_nav.create_mobile_radio();
  rad_nav.create_mobile_radio();
  rad_nav.create_mobile_radio();
  double x, y, heading, bearing;

  ros::NodeHandle nh;
  ROS_INFO(" - node handle created");

  ros::ServiceServer service = nh.advertiseService("localize", &td_navigation::worker::srvCallBack, &worker);
  ROS_INFO("td_navigation node ready to Localize");

  //subscriber for recieving messages
  ros::Subscriber rad104_s = nh.subscribe("/radio104/data", 5, &td_navigation::worker::radCallBack, &worker);
  ros::Subscriber rad105_s = nh.subscribe("/radio105/data", 5, &td_navigation::worker::radCallBack, &worker);


  //publisher for sending a message to timedomain serial to get a range request
  ros::Publisher rad104_p = nh.advertise<hw_interface_plugin_timedomain::Range_Request>("/radio104/cmd", 5);
  ros::Publisher rad105_p = nh.advertise<hw_interface_plugin_timedomain::Range_Request>("/radio105/cmd", 5);

  //publisher for getting average angle measurements
  ros::Publisher aa_p = nh.advertise<td_navigation::Average_angle>("/average_angles", 1);

  bool begin_avg = false;

  while(nh.ok())
  {

      ROS_INFO("Count: %d", worker.count);

      hw_interface_plugin_timedomain::Range_Request rr;
      rr.send_range_request = true;

      rr.msgID = worker.count;
      worker.confirmed = false;
      worker.selector = 0;

      //range request and response from 104 to 101
      if (worker.send_and_recieve(worker.rad_L, rr, rad104_p) == false){
        //call a function to tell about the malfunction
        ROS_DEBUG("We didn't get a response in time!");
      }

      worker.selector = 1;
      rr.msgID = worker.count;
      //range request and response from 104 to 106
      if (worker.send_and_recieve(worker.rad_R, rr, rad104_p) == false){
        //call a function to tell about the malfunction
        ROS_DEBUG("We didn't get a response in time!");
      }

      worker.selector = 2;
      rr.msgID = worker.count;
      //range request and response from 105 to 101
      if (worker.send_and_recieve(worker.rad_L, rr, rad105_p) == false){
        //call a function to tell about the malfunction
        ROS_DEBUG("We didn't get a response in time!");
      }

      worker.selector = 3;
      rr.msgID = worker.count;
      //range request and response from 105 to 106
      if (worker.send_and_recieve(worker.rad_R, rr, rad105_p) == false){
        //call a function to tell about the malfunction
        ROS_DEBUG("We didn't get a response in time!");
      }

      std::vector<double> distance_to_base_rads;

      ROS_INFO("TEST: 1");
      distance_to_base_rads.push_back(worker.get_avg_dist0_l(20));
      distance_to_base_rads.push_back(worker.get_avg_dist0_r(20));
      rad_nav.update_mobile_radio(0,distance_to_base_rads);
      ROS_INFO("TEST: 2");

      distance_to_base_rads[0] = worker.get_avg_dist1_l(20);
      distance_to_base_rads[1] = worker.get_avg_dist1_r(20);
      rad_nav.update_mobile_radio(1,distance_to_base_rads);
      ROS_INFO("TEST: 3");



      if (rad_nav.triangulate_2Base(worker.z_estimate) == 0){
        ROS_INFO("TEST: 4");

        heading = -smart_atan( (rad_nav.get_mobile_radio_coordinate(1,1) - rad_nav.get_mobile_radio_coordinate(0,1)),
                              (rad_nav.get_mobile_radio_coordinate(1,0) - rad_nav.get_mobile_radio_coordinate(0,0)) );
        ROS_INFO("TEST: 5");
        x = ( (rad_nav.get_mobile_radio_coordinate(0,0) + rad_nav.get_mobile_radio_coordinate(1,0)) / 2.0 ) +
             cos(heading) * worker.robot_length_offset;
        ROS_INFO("TEST: 6");
        y = ( (rad_nav.get_mobile_radio_coordinate(0,1) + rad_nav.get_mobile_radio_coordinate(1,1)) / 2.0 ) +
             sin(heading) * worker.robot_length_offset;
        ROS_INFO("TEST: 7");
        bearing = smart_atan(x,y);
        ROS_INFO("TEST: 8");
        worker.set_current_pos(x, y, heading * 180.0 / PI, bearing * 180.0 / PI);
        ROS_INFO("TEST: 9");
        ROS_DEBUG("Head: %lf, Bear: %lf, x: %lf, y: %lf", heading, bearing, x, y );
        ROS_INFO("TEST: 10");

        td_navigation::Average_angle aa;

        aa.heading = heading;
        aa.bearing = bearing;
        aa.x = x;
        aa.y = y;

        aa_p.publish(aa);


      }else {
        ROS_DEBUG("Problem encountered with triangulation!");
      }

  //update count
  worker.update_count();
  if(worker.count >= 32000){
      worker.count = 0;
  }

}

  ROS_DEBUG("td_navigation closing");
  return 0;
}

double smart_atan(double x, double y){
      if (x == 0 && y >= 0){
        return PI / 2.0;
      }else if (x == 0 && y < 0){
        return -1 * PI / 2.0;
      }else if (x < 0){
        return atan(y/x) + PI;
      }else {
        return atan(y/x);
      }
      return -500;
}
