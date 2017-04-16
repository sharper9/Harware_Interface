#include <td_navigation/td_navigation.h>

#define PI 3.14159265
#define ERRNUM -200000000.1

td_navigation::worker::worker()
{
  average_length = 20;
  dStation =  1930.4;
  rad_L =  101;
  rad_R = 106;


  confirmed = 0;
  count = 0;
  selector = 0 ;

  rad104_DistL = 0;
  rad104_DistR = 0;

  rad105_DistL = 0;
  rad105_DistR = 0;

  headings.reserve(average_length);
  bearings.reserve(average_length);
  x.reserve(average_length);
  y.reserve(average_length);
}


bool td_navigation::worker::send_and_recieve(int& to, hw_interface_plugin_timedomain::Range_Request& rr, ros::Publisher& rad_pub){
  int wait = 0;
  int timeout = 0;
  //TODO: change back to about 200
  ros::Rate loop_rate(500);

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

    //Selector for assigneing the distance to the correct value
    if(msg->msgID >=0 && msg->msgID < 32000 ){
        if(msg->msgID == count && selector == 0){
            ROS_INFO("Reading from rad0 to leftRad");
            ROS_DEBUG("Precise Range Measure: %d", msg->PRM);
            rad104_DistL = msg->PRM;
            confirmed = true;
        }else if(msg-> msgID == count && selector == 1){
            ROS_INFO("Reading from rad0 to RightRad");
            ROS_DEBUG("Precise Range Measure: %d", msg->PRM);
            rad104_DistR = msg->PRM;
            confirmed = true;
            return;
        }else if(msg-> msgID == count && selector == 2){
            ROS_INFO("Reading from rad1 to leftRad");
            ROS_DEBUG("Precise Range Measure: %d", msg->PRM);
            rad105_DistL = msg->PRM;
            confirmed = true;
            return;
        }else if(msg-> msgID == count && selector == 3){
            ROS_INFO("Reading from rad1 to RightRad");
            ROS_DEBUG("Precise Range Measure: %d", msg->PRM);
            rad105_DistR = msg->PRM;
            confirmed = true;
            return;

        }else{
            ROS_WARN("TDRR MsgID mismatched!");
            return;
        }

    }else{
        ROS_WARN("TDRR: Recieved MsgID never assigned!");
    }


}

bool td_navigation::worker::srvCallBack(td_navigation::Localize::Request &req, td_navigation::Localize::Response &res){
  int non_err = 0;

  double h_dev_sum = 0;
  double b_dev_sum = 0;
//get the standard deviation
  for(int i = 0; i < average_length; i++){
    if (headings[i] != ERRNUM){
      h_dev_sum += pow(headings[i] - this->get_avg_heading(), 2.0);
      b_dev_sum += pow(bearings[i] - this->get_avg_bearing(), 2.0);
      non_err++;
    }
  }

if (non_err == 0){
  res.fail = true;
  return false;
}

//service response
res.x = this->get_avg_x();
res.y = this->get_avg_y();
res.heading = this->get_avg_heading();
res.head_dev = sqrtl(h_dev_sum/non_err);
res.bearing = this->get_avg_bearing();
res.bear_dev = sqrtl(b_dev_sum/non_err);
res.fail = false;

return true;
}

void td_navigation::worker::set_current_heading(double heading){
  headings[count%average_length] = heading;
}

void td_navigation::worker::set_current_bearing(double bearing){
  bearings[count%average_length] = bearing;
}

void td_navigation::worker::set_current_pos_x(double pos_x){
  x[count%average_length] = pos_x;
}

void td_navigation::worker::set_current_pos_y(double pos_y){
  y[count%average_length] = pos_y;
}

double td_navigation::worker::get_current_heading(){
  return headings[count%average_length];
}

double td_navigation::worker::get_current_bearing(){
  return bearings[count%average_length];
}

double td_navigation::worker::get_current_pos_x(){
  return x[count%average_length];
}

double td_navigation::worker::get_current_pos_y(){
  return y[count%average_length];
}

double td_navigation::worker::get_avg_heading(){
  int non_err = 0;
  double head_sum = 0;

  for(int i = 0; i < average_length; i++){
    if (headings[i] != ERRNUM){
      head_sum += headings[i];
      non_err++;
    }
  }

  if (non_err == 0){
    return ERRNUM;
  }

  return head_sum/non_err;
}

double td_navigation::worker::get_avg_bearing(){
  int non_err = 0;
  double bear_sum = 0;

  for(int i = 0; i < average_length; i++){
    if (bearings[i] != ERRNUM){
      bear_sum += bearings[i];
      non_err++;
    }
  }

  if (non_err == 0){
    return ERRNUM;
  }

  return bear_sum/non_err;
}

double td_navigation::worker::get_avg_x(){
  int non_err = 0;
  double x_sum = 0;

  for(int i = 0; i < average_length; i++){
    if (x[i] != ERRNUM){
      x_sum += x[i];
      non_err++;
    }
  }

  if (non_err == 0){
    return ERRNUM;
  }

  return x_sum/non_err;
}

double td_navigation::worker::get_avg_y(){
  int non_err = 0;
  double y_sum = 0;

  for(int i = 0; i < average_length; i++){
    if (y[i] != ERRNUM){
      y_sum += y[i];
      non_err++;
    }
  }

  if (non_err == 0){
    return ERRNUM;
  }

  return y_sum/non_err;
}

void td_navigation::worker::update_count(){
  count++;
}

int td_navigation::worker::set_error(){
  headings[count%average_length] = ERRNUM;
  bearings[count%average_length] = ERRNUM;
  x[count%average_length] = ERRNUM;
  y[count%average_length] = ERRNUM;

  int num_of_errors;
  for (int i = 0; i < average_length; i++){
    if (headings[i] == ERRNUM){
      num_of_errors++;
    }
  }
  return num_of_errors;
}

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

  td_navigation::worker worker;

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


      radio_nav rad_nav;
      rad_nav.set_all(worker.dStation, worker.rad104_DistL, worker.rad104_DistR, worker.rad105_DistL, worker.rad105_DistR);
      rad_nav.triangulate();

      if (rad_nav.success()){

        if (worker.count == worker.average_length){
          begin_avg = true;
        }

        ROS_DEBUG("RadNavHead: %lf, RadNavBear: %lf", rad_nav.get_heading() * 180.0 / PI, rad_nav.get_bearing() * 180.0 / PI);
        //put the most recent measurement into the array
        //angle the bot is looking
        worker.set_current_heading(rad_nav.get_heading() * 180.0 / PI);
        worker.set_current_bearing(rad_nav.get_bearing() * 180.0 / PI);

        worker.set_current_pos_x( ( rad_nav.get_rad0_x() + rad_nav.get_rad1_x() ) / (2.0) );
        worker.set_current_pos_y( ( rad_nav.get_rad0_y() + rad_nav.get_rad1_y() ) / (2.0) );

        ROS_DEBUG("Head: %lf, Bear: %lf, x: %lf, y: %lf", worker.get_current_heading(), worker.get_current_bearing(),
                  worker.get_current_pos_x(), worker.get_current_pos_y() );

        if(begin_avg){
          td_navigation::Average_angle aa;

          aa.heading = worker.get_avg_heading();
          aa.bearing = worker.get_avg_bearing();
          aa.x = worker.get_avg_x();
          aa.y = worker.get_avg_y();

          aa_p.publish(aa);
        }

      }else {
        ROS_DEBUG("NumberOfErrorsInLast%d: %d", worker.average_length, worker.set_error());
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
