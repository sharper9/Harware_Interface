#include <td_navigation/td_navigation.h>

#define PI 3.14159265

td_navigation::worker::worker()
{
  // ros::param::get("average_length", average_length);
  // ros::param::get("dStation", dStation);
  // ros::param::get("rad_L", rad_L);
  // ros::param::get("rad_R", rad_R);

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
  
  headings.resize(average_length);
  bearings.resize(average_length);
  x.resize(average_length);
  y.resize(average_length);

  x_sum = 0;
  y_sum = 0;
  head_sum = 0;
  bear_sum = 0;
  h_dev_sum = 0;
  b_dev_sum = 0;
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

//get the standard deviation
  for(int i = 0; i < average_length; i++){
    h_dev_sum += pow(headings[i] - head_sum/average_length, 2.0);
    b_dev_sum += pow(bearings[i] - bear_sum/average_length, 2.0);
  }

//service response
res.x = x_sum/average_length;
res.y = y_sum/average_length;
res.heading = head_sum/average_length;
res.head_dev = sqrtl(h_dev_sum/average_length);
res.bearing = bear_sum/average_length;
res.bear_dev = sqrtl(b_dev_sum/average_length);
res.fail = false;

return true;
}

void td_navigation::worker::subtract_oldest(){
  head_sum -= headings[count%average_length];
  bear_sum -= bearings[count%average_length];
  x_sum -= x[count%average_length];
  y_sum -= y[count%average_length];
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

void td_navigation::worker::update_sums(){
  head_sum += headings[count%average_length];
  bear_sum += bearings[count%average_length];

  x_sum += x[count%average_length];
  y_sum += y[count%average_length];
}

double td_navigation::worker::get_avg_heading(){
  return head_sum/average_length;
}

double td_navigation::worker::get_avg_bearing(){
  return bear_sum/average_length;
}

double td_navigation::worker::get_avg_x(){
  return x_sum/average_length;
}

double td_navigation::worker::get_avg_y(){
  return y_sum/average_length;
}

void td_navigation::worker::update_count(){
  count++;
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



      if (worker.count == worker.average_length){
        begin_avg = true;
      }

      //take away the oldest value from the sum (only happens once we've started averaging)
      if(begin_avg == true){
        worker.subtract_oldest();
      }

      ROS_DEBUG("RadNavHead: %lf, RadNavBear: %lf", rad_nav.get_heading() * 180.0 / PI, rad_nav.get_bearing() * 180.0 / PI);
      //put the most recent measurement into the vector
      //angle the bot is looking
      worker.set_current_heading(rad_nav.get_heading() * 180.0 / PI);
      worker.set_current_bearing(rad_nav.get_bearing() * 180.0 / PI);

      worker.set_current_pos_x( ( rad_nav.get_rad0_x() + rad_nav.get_rad1_x() ) / (2.0) );
      worker.set_current_pos_y( ( rad_nav.get_rad0_y() + rad_nav.get_rad1_y() ) / (2.0) );

      worker.update_sums();

      ROS_DEBUG("Head: %lf, Bear: %lf, x: %lf, y: %lf", worker.get_current_heading(), worker.get_current_bearing(),
                worker.get_current_pos_x(), worker.get_current_pos_y() );
                
      ROS_DEBUG("HeadSum: %lf, BearSum: %lf", worker.get_avg_heading() * worker.average_length, worker.get_avg_bearing() * worker.average_length);
      

      if (begin_avg == true){
        td_navigation::Average_angle aa;

        aa.heading = worker.get_avg_heading();
        aa.bearing = worker.get_avg_bearing();
        aa.x = worker.get_avg_x();
        aa.y = worker.get_avg_y();
        aa.rad0toL = rad_nav.get_rad0_distl();
        aa.rad0toR = rad_nav.get_rad0_distr();
        aa.rad1toL = rad_nav.get_rad1_distl();
        aa.rad1toR = rad_nav.get_rad1_distr();

        aa_p.publish(aa);
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
