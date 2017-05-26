#include <td_navigation/td_navigation.h>

#define PI 3.14159265358

#define DEG_TO_RAD 0.0174532925
#define WT_VALUE 500
#define REQUEST_TIMEOUT 0.5

#define DIG_MAP_X_LEN 7380
#define DIG_MAP_Y_LEN 3780
#define ROBOT_CENTER_TO_SCOOP 1000 //in millimeters
#define WALL_BUFFER_LEN 500 //in millimeters


td_navigation::worker::worker(int average_length_val, double base_station_distance_val,
                              int rad_L_val, int rad_R_val, int z_estimate_val,
                              int robot_length_offset_val, int mob_rad_dist_value)
{
  position_init = false;
  request_confirmed = 0;
  request_failed = false;
  count = 0;
  selector = 0 ;
  successful_range = false;
  bad_ranges = 0;

  half_dist_0 = 0;
  half_dist_1 = 0;

  base_rad_0_malfunction = false;
  base_rad_1_malfunction = false;
  mob_rad_0_malfunction = false;
  mob_rad_1_malfunction = false;

  half_angle_left = true;


  heading = 0;
  bearing = 0;

  average_length = average_length_val;
  base_station_distance = base_station_distance_val;
  rad_L = rad_L_val;
  rad_R = rad_R_val;
  z_estimate = z_estimate_val;
  robot_length_offset = robot_length_offset_val;
  mob_rad_dist = mob_rad_dist_value;

  dist0_l.resize(2);
  dist0_r.resize(2);
  dist1_l.resize(2);
  dist1_r.resize(2);


  rad_nav.create_base_radio(0, -1.0 * base_station_distance/2.0, 0);
  rad_nav.create_base_radio(0, base_station_distance/2.0, 0);
  rad_nav.create_mobile_radio();
  rad_nav.create_mobile_radio();

  ros::NodeHandle nh;
  ROS_INFO(" - node handle created");

  ros::ServiceServer service = nh.advertiseService("localize", &td_navigation::worker::srvCallBack, this);
  ROS_INFO("td_navigation node ready to Localize");

  //subscriber for recieving messages
  mob_rad_l_sub = nh.subscribe("/radio_left/data", 5, &td_navigation::worker::mob_rad_0_CallBack, this);
  mob_rad_r_sub = nh.subscribe("/radio_right/data", 5, &td_navigation::worker::mob_rad_1_CallBack, this);

  nav_filter_sub = nh.subscribe("/navigation/navigationfilterout/navigationfilterout", 1, &td_navigation::worker::nav_filter_callback, this);


  //publisher for sending a message to timedomain serial to get a range request
  mob_rad_l_pub = nh.advertise<hw_interface_plugin_timedomain::Range_Request>("/radio_left/cmd", 5);
  mob_rad_r_pub = nh.advertise<hw_interface_plugin_timedomain::Range_Request>("/radio_right/cmd", 5);

  rhp_pub = nh.advertise<td_navigation::Running_Half_Pose>("/Half_Pose", 1);

  status_pub = nh.advertise<td_navigation::Td_navigation_Status>("/Td_Status", 1);

  //publisher for getting average angle measurements
  aa_p = nh.advertise<td_navigation::Average_angle>("/average_angles", 1);

  while(nh.ok())
  {
    if(position_init == true){
      if(half_angle_left){
        if(run_half_pose_left() != 0){
          half_angle_left = false;
        }
      }else{
        if(run_half_pose_right() != 0){
          half_angle_left = true;
        }
      }
    }
    ros::spinOnce();
  }
}


bool td_navigation::worker::send_and_recieve(int to, hw_interface_plugin_timedomain::Range_Request& rr, ros::Publisher& rad_pub){


  //TODO: change back to about 5000
  //ros::Rate loop_rate(5000);
    rr.radio_id_to_target = to;
    rad_pub.publish(rr);
    double time_sent = ros::Time::now().toSec();


  while(!request_confirmed){

    ros::spinOnce();
    if(ros::Time::now().toSec() - time_sent > REQUEST_TIMEOUT){
      return false;
    }

  }
  request_confirmed = false;
  return true;
}

int td_navigation::worker::add_distance(std::vector < std::vector<double> >& distances,
                                        double range, double error){


  if(distances[0].size() < average_length){

    distances[0].push_back(range);
    distances[1].push_back(error);


    if(error >= mob_rad_dist/2){
      bad_ranges++;
    }
  }else{
    if(bad_ranges > 0 && error < mob_rad_dist/2){

      for(int i = 0; i < distances[0].size(); i++){

        if(distances[1][i] >= mob_rad_dist/2){

          distances[0][i] = range;
          distances[1][i] = error;
          bad_ranges -= 1;
        }
      }
    }
  }

  return 0;
}

void td_navigation::worker::mob_rad_0_CallBack(const hw_interface_plugin_timedomain::RCM_Range_Info::ConstPtr &msg){
    ROS_INFO("TDRR Navigation Callback");
    //check if the radio was busy
    if (msg->busy == true){
        ROS_WARN("TDRR was busy");
        request_failed = true;
        return;
    }
    //check if the radio range has failed
    if (msg->failed == true){
        ROS_WARN("TDRR request has failed!");
        request_failed = true;
        return;

    }

    //Selector for assigning the distance to the correct value
    if(msg->msgID >=0 && msg->msgID < 32000 ){

      //reading 0 to l
        if(msg->msgID == count + selector && selector == 0){
            ROS_INFO("Reading from Mob_0 to Base_0");
            ROS_DEBUG("Precise Range Measure: %d", msg->PRM);
            add_distance(dist0_l, (double)msg->PRM, (double)msg->PRMError);
            request_confirmed = true;
            return;

        //reading 1 to l
      }else if(msg-> msgID == count + selector && selector == 1){
          ROS_INFO("Reading from Mob_0 to Base_1");
          ROS_DEBUG("Precise Range Measure: %d", msg->PRM);
          add_distance(dist0_r, msg->PRM, msg->PRMError);
          request_confirmed = true;
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
            ROS_INFO("Reading from Mob_1 to Base_0");
            ROS_DEBUG("Precise Range Measure: %d", msg->PRM);
            add_distance(dist1_l, msg->PRM, msg->PRMError);
            request_confirmed = true;
            return;

      //reading 1 to r
    }else if(msg-> msgID == count + selector && selector == 3){
          ROS_INFO("Reading from Mob_1 to Base_1");
          ROS_DEBUG("Precise Range Measure: %d", msg->PRM);
          add_distance(dist1_r, msg->PRM, msg->PRMError);
          request_confirmed = true;
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
  position_init = msg->initial_pose_found;
}




bool td_navigation::worker::srvCallBack(td_navigation::Localize::Request &req,
                                        td_navigation::Localize::Response &res){


  double mob_rad_0_l_dev = 0;
  double mob_rad_0_r_dev = 0;
  double mob_rad_1_l_dev = 0;
  double mob_rad_1_r_dev = 0;


  double mob_rad_0_error = 0;
  double mob_rad_1_error = 0;

  double max_angle_error = 0;

  td_navigation::Td_navigation_Status stat;

  dist0_l[0].clear();
  dist0_l[1].clear();
  dist0_r[0].clear();
  dist0_r[1].clear();
  dist1_l[0].clear();
  dist1_l[1].clear();
  dist1_r[0].clear();
  dist1_r[1].clear();

  average_length = req.average_length;

  int error_type = run_full_pose();

  res.base_L_failure = base_rad_0_malfunction;
  res.base_R_failure = base_rad_1_malfunction;
  res.mob_L_failure = mob_rad_0_malfunction;
  res.mob_R_failure = mob_rad_1_malfunction;

  ROS_INFO("Base_L_fail:%d, Base_R_fail:%d, Mob_L_fail:%d, Mob_R_fail:%d", base_rad_0_malfunction, base_rad_1_malfunction, mob_rad_0_malfunction, mob_rad_1_malfunction);


  if(error_type == -1){
    res.triangulation_failure = true;
    res.fail = true;
    //check if any obstructions are clear, gives a hint to our position
    bool b_0_m_0_obstruction = get_avg_err0_l(average_length) > mob_rad_dist;
    bool b_0_m_1_obstruction = get_avg_err0_r(average_length) > mob_rad_dist;
    bool b_1_m_0_obstruction = get_avg_err1_l(average_length) > mob_rad_dist;
    bool b_1_m_1_obstruction = get_avg_err1_r(average_length) > mob_rad_dist;

    //for future programmers: this behavioral analysis doesn't really belong here,
    //it should be done in mission control/planning area. This was a bit of a last minute addition

    if(b_0_m_0_obstruction && b_1_m_1_obstruction){
      stat.initialization_maneuver = 16; //not yet set // left 130
    }else if(b_0_m_0_obstruction && b_0_m_1_obstruction){
      stat.initialization_maneuver = 17; //not yet set // right 90
    }else if(b_0_m_1_obstruction){
      stat.initialization_maneuver = 18; //not yet set // right 40
    }else if(b_1_m_0_obstruction){
      stat.initialization_maneuver = 19; //not yet set // left 40
    }else if(b_1_m_0_obstruction && b_1_m_1_obstruction){
      stat.initialization_maneuver = 20; //not yet set //left 90
    }else {
      stat.initialization_maneuver = 5; //turn left 20
    }
    stat.success = false;
    status_pub.publish(stat);
    ros::spinOnce();
    return true;
  }
  if(error_type == -2){
    stat.success = false;
    //check if any obstructions are clear, gives a hint to our position
    bool b_0_m_0_obstruction = get_avg_err0_l(average_length) > mob_rad_dist;
    bool b_0_m_1_obstruction = get_avg_err0_r(average_length) > mob_rad_dist;
    bool b_1_m_0_obstruction = get_avg_err1_l(average_length) > mob_rad_dist;
    bool b_1_m_1_obstruction = get_avg_err1_r(average_length) > mob_rad_dist;

    //for future programmers: this behavioral analysis doesn't really belong here,
    //it should be done in mission control/planning area. This was a bit of a last minute addition

    if(b_0_m_0_obstruction && b_1_m_1_obstruction){
      stat.initialization_maneuver = 16; //not yet set // left 130
    }else if(b_0_m_0_obstruction && b_0_m_1_obstruction){
      stat.initialization_maneuver = 17; //not yet set // right 90
    }else if(b_0_m_1_obstruction){
      stat.initialization_maneuver = 18; //not yet set // right 40
    }else if(b_1_m_0_obstruction){
      stat.initialization_maneuver = 19; //not yet set // left 40
    }else if(b_1_m_0_obstruction && b_1_m_1_obstruction){
      stat.initialization_maneuver = 20; //not yet set //left 90
    }else {
      stat.initialization_maneuver = 5; //turn left 20
    }

    status_pub.publish(stat);
    res.fail = true;
    ros::spinOnce();
    return true;
  }

  res.x = x/1000.0;
  res.y = y/1000.0;
  res.heading = heading;
  res.bearing = bearing;

  mob_rad_0_l_dev = get_std_dev(dist0_l);
  mob_rad_0_r_dev = get_std_dev(dist0_r);
  mob_rad_1_l_dev = get_std_dev(dist1_l);
  mob_rad_1_r_dev = get_std_dev(dist1_r);

  res.mob_rad_0_l_dev = mob_rad_0_l_dev;
  res.mob_rad_0_r_dev = mob_rad_0_r_dev;
  res.mob_rad_1_l_dev = mob_rad_1_l_dev;
  res.mob_rad_1_r_dev = mob_rad_1_r_dev;


  mob_rad_0_error = sqrt( pow(mob_rad_0_l_dev, 2.0) + pow(mob_rad_0_r_dev, 2.0));
  mob_rad_1_error = sqrt( pow(mob_rad_1_l_dev, 2.0) + pow(mob_rad_1_r_dev, 2.0));
  ROS_DEBUG("mob_rad_0_error: %lf", mob_rad_0_error);
  ROS_DEBUG("mob_rad_1_error: %lf", mob_rad_1_error);

  if(mob_rad_0_error >= mob_rad_dist || mob_rad_1_error >= mob_rad_dist){
    max_angle_error = PI;
  }else{
    max_angle_error = atan(mob_rad_0_error/ (mob_rad_dist/
                  ( (mob_rad_0_error/mob_rad_1_error)+1) ) );
  }
  ROS_DEBUG("max_angle_error: %lf", max_angle_error);
  res.max_angle_error = max_angle_error;
    ROS_INFO("x:%lf, y:%lf\nhead:%lf, bear:%lf\nmob_rad_0_l_dev:%lf, mob_rad_0_r_dev:%lf\nmob_rad_1_l_dev:%lf, mob_rad_1_r_dev%lf\nmax_angle_error:%lf", x/1000,y/1000,heading,bearing,mob_rad_0_l_dev, mob_rad_0_r_dev, mob_rad_1_l_dev, mob_rad_1_r_dev, max_angle_error);
  res.fail = false;//this has been changed to always true


  if (max_angle_error > 15*DEG_TO_RAD){
    stat.success = false;
  }else{
    stat.success = true;
  }
  if( (y + ROBOT_CENTER_TO_SCOOP < DIG_MAP_Y_LEN - WALL_BUFFER_LEN) && (y - ROBOT_CENTER_TO_SCOOP > -DIG_MAP_Y_LEN + WALL_BUFFER_LEN)
     && (x -ROBOT_CENTER_TO_SCOOP > WALL_BUFFER_LEN) && position_init){        // ready to move
     stat.success = true;
     stat.initialization_maneuver = 15;
  }else if (heading < 30 * DEG_TO_RAD && heading > -30 * DEG_TO_RAD){
    stat.initialization_maneuver = 1;
  }else if(heading >= 30 * DEG_TO_RAD && heading <= 100 * DEG_TO_RAD){
    stat.initialization_maneuver = 9;
  }else if(heading >= 100 * DEG_TO_RAD && heading <=135 * DEG_TO_RAD){
    stat.initialization_maneuver = 19;
  }else if(heading <= -30 * DEG_TO_RAD && heading >= -100 * DEG_TO_RAD){
    stat.initialization_maneuver = 7;
  }else if(heading <= -100 * DEG_TO_RAD && heading >= -135 * DEG_TO_RAD){
    stat.initialization_maneuver = 18;
  }else if ( (heading >= -180 * DEG_TO_RAD && heading < -150 * DEG_TO_RAD) || (heading <= 180 * DEG_TO_RAD && heading > 150 * DEG_TO_RAD) ){
    stat.initialization_maneuver = 12;
  }else{
    stat.initialization_maneuver = 5;
  }

  status_pub.publish(stat);
  ros::spinOnce();

  return true;
}

double td_navigation::worker::get_avg_dist(std::vector< std::vector< double> >& dist, int& amount_to_avg){
  double top = 0;
  double bottom = 0;
  double weight = 0;
  int max = 0;

  if(amount_to_avg <= dist[0].size()){
    max = amount_to_avg;
  }else{
    max = dist[0].size();
  }
  for(int i = 0; i < max; i++){
    weight = (-1.0 * WT_VALUE / (PI/2.0) * atan(0.05 * (dist[1][i] - (base_station_distance / 5))) + WT_VALUE + 1);
    ROS_INFO("dist:%lf ,err:%lf, wt:%lf", dist[0][i], dist[1][i], weight);
    top += dist[0][i] * weight;
    bottom += weight;
  }

  if(bottom == 0){
    return 0;
  }

  return top/bottom;

}

double td_navigation::worker::get_std_dev(std::vector< std::vector< double> >& distances){
  double avg = get_avg_dist(distances, average_length);
  double sum = 0;
  for(int i = 0; i < distances[0].size(); i++){
    sum += ( pow(distances[0][i] - avg, 2.0) );
  }

  if(distances[0].size() == 0){
    return 0;
  }
  return sqrt(sum/distances[0].size());
}

double td_navigation::worker::get_avg_error(std::vector< std::vector< double > >& error, int& amount_to_avg){
  double sum = 0;
  double bottom = 0;
  double weight = 0;

  int max = 0;

  if(amount_to_avg <= error[0].size()){
    max = amount_to_avg;
  }else{
    max = error[0].size();
  }
  for(int i = 0; i < max; i++){
    sum += error[1][i];
    bottom += 1;
  }

  if(max == 0){
    return 0;
  }
  return sum/max;
}

double td_navigation::worker::get_med_error(std::vector< std::vector< double > >& error){
    std::vector<double> err;
    err.resize(error[0].size());
    for(int i = 0; i < error.size(); i++){
        err[i] = error[1][i];
    }
    sort(err.begin(), err.end());
    return err[(int)(err.size()/2)];

}

double td_navigation::worker::get_med_err0_l(){
    return get_med_error(dist0_l);
}

double td_navigation::worker::get_med_err0_r(){
    return get_med_error(dist0_r);
}

double td_navigation::worker::get_med_err1_l(){
    return get_med_error(dist1_l);
}

double td_navigation::worker::get_med_err1_r(){
    return get_med_error(dist1_r);
}

double td_navigation::worker::get_avg_err0_l(int amount_to_avg){
  return get_avg_error(dist0_l, amount_to_avg);
}

double td_navigation::worker::get_avg_err0_r(int amount_to_avg){
  return get_avg_error(dist0_r, amount_to_avg);
}

double td_navigation::worker::get_avg_err1_l(int amount_to_avg){
  return get_avg_error(dist1_l, amount_to_avg);
}

double td_navigation::worker::get_avg_err1_r(int amount_to_avg){
  return get_avg_error(dist1_r, amount_to_avg);
}



double td_navigation::worker::get_avg_dist0_l(int amount_to_avg){
  ROS_INFO("Radio mob_l to base_l");
  return get_avg_dist(dist0_l, amount_to_avg);
}

double td_navigation::worker::get_avg_dist0_r(int amount_to_avg){
    ROS_INFO("Radio mob_l to base_r");
  return get_avg_dist(dist0_r, amount_to_avg);

}

double td_navigation::worker::get_avg_dist1_l(int amount_to_avg){
    ROS_INFO("Radio mob_r to base_l");
  return get_avg_dist(dist1_l, amount_to_avg);
}

double td_navigation::worker::get_avg_dist1_r(int amount_to_avg){
    ROS_INFO("Radio mob_r to base_r");
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


int td_navigation::worker::run_full_pose(){

  ROS_INFO("Count: %d", count);

  hw_interface_plugin_timedomain::Range_Request rr;
  rr.send_range_request = true;

  bool rad0_l_mal = false;
  bool rad0_r_mal = false;
  bool rad1_l_mal = false;
  bool rad1_r_mal = false;

  int doom_count = 0;
  bool failed = false;
  request_confirmed = false;
  selector = 0;
  int num = 0;
  count = 0;
  while(num <= average_length + 20 && !(dist0_l[0].size() == average_length && bad_ranges == 0) && doom_count < 10){

    rr.msgID = count + selector;
    //range request and response from 104 to 101
    if (send_and_recieve(rad_L, rr, mob_rad_l_pub) == false){
      //call a function to tell about the malfunction
      ROS_DEBUG("Mob_L Base_L didn't respond in time!");
      rad0_l_mal = true;
      doom_count++;
      failed = true;
    }
    if(!failed){
    rad0_l_mal = false;
    doom_count = 0;
    num++;
    update_count();
    }
    failed = false;
  }


  selector = 1;
  bad_ranges = 0;
  doom_count = 0;
  count = 0;
  num = 0;
  while(num <= average_length + 20 && !(dist0_r[0].size() == average_length && bad_ranges == 0) && doom_count < 10){

    rr.msgID = count + selector;
    //range request and response from 104 to 106
    if (send_and_recieve(rad_R, rr, mob_rad_l_pub) == false){
      //call a function to tell about the malfunction
      ROS_DEBUG("Mob_L Base_R didn't responsd in time!");
      rad0_r_mal = true;
      doom_count++;
      failed = true;
    }
    if(!failed){
      rad0_r_mal = false;
      doom_count = 0;
      num++;
      update_count();
    }
    failed = false;
  }

  selector = 2;
  bad_ranges = 0;
  doom_count = 0;
  count = 0;
  num = 0;
  while(num <= average_length + 20 && !(dist1_l[0].size() == average_length && bad_ranges == 0) && doom_count < 10){

    rr.msgID = count + selector;
    //range request and response from 105 to 101
    if (send_and_recieve(rad_L, rr, mob_rad_r_pub) == false){
      //call a function to tell about the malfunction
      ROS_DEBUG("Mob_R Base_L didn't responsd in time!");
      rad1_l_mal = true;
      doom_count++;
      failed = true;
    }
    if(!failed){
      rad1_l_mal = false;
      doom_count = 0;
      num++;
      update_count();
    }
    failed = false;

  }

  selector = 3;
  bad_ranges = 0;
  doom_count = 0;
  count = 0;
  num = 0;
  while(num <= average_length + 20 && !(dist1_r[0].size() == average_length && bad_ranges == 0) && doom_count < 10){
    rr.msgID = count + selector;
    //range request and response from 105 to 106
    if (send_and_recieve(rad_R, rr, mob_rad_r_pub) == false){
      //call a function to tell about the malfunction
      ROS_DEBUG("Mob_R Base_R didn't responsd in time!");
      rad1_r_mal = true;
      doom_count++;
      failed = true;
    }
    if(!failed){
     rad1_l_mal = false;
      doom_count = 0;
      num++;
      update_count();
    }
    failed = false;

  }
  doom_count = 0;


  base_rad_0_malfunction = rad0_l_mal && rad1_l_mal;
  base_rad_1_malfunction = rad0_r_mal && rad1_r_mal;
  mob_rad_0_malfunction = rad0_l_mal && rad0_r_mal;
  mob_rad_1_malfunction = rad1_l_mal && rad1_r_mal;

  if (rad0_l_mal || rad0_r_mal || rad1_l_mal || rad1_r_mal){
    return -2;
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

return 0;

}

int td_navigation::worker::run_half_pose_left(){

  dist0_l[0].clear();
  dist0_l[1].clear();
  dist0_r[0].clear();
  dist0_r[1].clear();
  dist1_l[0].clear();
  dist1_l[1].clear();
  dist1_r[0].clear();
  dist1_r[1].clear();

  hw_interface_plugin_timedomain::Range_Request rr;
  rr.send_range_request = true;

  bool to_left_mal = false;
  bool to_right_mal = false;

  request_confirmed = false;
  selector = 0;
  rr.msgID = count + selector;
  //range request and response from 104 to 101
  if (send_and_recieve(rad_L, rr, mob_rad_l_pub) == false){
    //call a function to tell about the malfunction
    ROS_DEBUG("Mob_L Base_L didn't respond in time!");
    to_left_mal = true;
  }

  selector = 1;
  rr.msgID = count + selector;
  //range request and response from 104 to 106
  if (send_and_recieve(rad_R, rr, mob_rad_l_pub) == false){
    //call a function to tell about the malfunction
    ROS_DEBUG("Mob_L Base_R didn't respond in time!");
    to_right_mal = true;
  }

  if(to_left_mal || to_right_mal){
    return -2;

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
    ROS_ERROR("Problem encountered with triangulation!");
    return -1;
  }

return 0;
}

int td_navigation::worker::run_half_pose_right(){

  dist0_l[0].clear();
  dist0_l[1].clear();
  dist0_r[0].clear();
  dist0_r[1].clear();
  dist1_l[0].clear();
  dist1_l[1].clear();
  dist1_r[0].clear();
  dist1_r[1].clear();

  hw_interface_plugin_timedomain::Range_Request rr;
  rr.send_range_request = true;

  bool to_left_mal = false;
  bool to_right_mal = false;

  request_confirmed = false;
  selector = 2;
  rr.msgID = count + selector;
  //range request and response from 104 to 101
  if (send_and_recieve(rad_L, rr, mob_rad_r_pub) == false){
    //call a function to tell about the malfunction
    ROS_DEBUG("Mob_R Base_L didn't respond in time!");
    to_left_mal = true;
  }

  selector = 3;
  rr.msgID = count + selector;
  //range request and response from 104 to 106
  if (send_and_recieve(rad_R, rr, mob_rad_r_pub) == false){
    //call a function to tell about the malfunction
    ROS_DEBUG("Mob_R Base_R didn't respond in time!");
    to_right_mal = true;
  }

  if(to_left_mal || to_right_mal){
    return -2;
  }

  std::vector<double> distance_to_base_rads;


  distance_to_base_rads.push_back(get_avg_dist1_l(1));
  distance_to_base_rads.push_back(get_avg_dist1_r(1));
  rad_nav.update_mobile_radio(1 ,distance_to_base_rads);

  if (rad_nav.triangulate_2Base(z_estimate) == 0){

    double angle_2 = heading + PI/2.0;
    double back_x, back_y;

    back_x = rad_nav.get_mobile_radio_coordinate(1,0) - cos(angle_2) * mob_rad_dist / 2.0;

    back_y = rad_nav.get_mobile_radio_coordinate(1,1) - sin(angle_2) * mob_rad_dist / 2.0;

    x = back_x + (cos(heading) * robot_length_offset);

    y = back_y + (sin(heading) * robot_length_offset);


    ROS_DEBUG("Head: %lf, Bear: %lf, x: %lf, y: %lf", heading * 180.0 / PI, bearing * 180.0 / PI, x, y );


    td_navigation::Running_Half_Pose rhp;

    rhp.x = x/1000.0;
    rhp.y = y/1000.0;

    rhp_pub.publish(rhp);


  }else {
    ROS_ERROR("Problem encountered with triangulation!");
    return -1;
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
  td_navigation::worker worker(50, 1776, 104, 106, 0, 595, 557); // base = 1780, robot width = 557, robot length = 635

  ROS_DEBUG("td_navigation closing");
  return 0;
}
