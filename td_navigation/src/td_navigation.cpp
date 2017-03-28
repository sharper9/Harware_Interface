#include <td_navigation/td_navigation.h>

#define PI 3.14159265

td_navigation::listener::listener()
{
  confirmed = 0;
  rad104_DistL = 0;
  rad104_DistR = 0;
  rad105_DistL = 0;
  rad105_DistR = 0;
  selector = 0;
}

//check the message id, busy, failed
//update appropriate distance
void td_navigation::listener::radCallBack(const hw_interface_plugin_timedomain::RCM_Range_Info::ConstPtr &msg){
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
        if(msg->msgID == *count && selector == 0){
            ROS_INFO("Reading from rad0 to leftRad");
            ROS_DEBUG("Precise Range Measure: %d", msg->PRM);
            rad104_DistL = msg->PRM;
            confirmed = true;
        }else if(msg-> msgID == *count && selector == 1){
            ROS_INFO("Reading from rad0 to RightRad");
            ROS_DEBUG("Precise Range Measure: %d", msg->PRM);
            rad104_DistR = msg->PRM;
            confirmed = true;
            return;
        }else if(msg-> msgID == *count && selector == 2){
            ROS_INFO("Reading from rad1 to leftRad");
            ROS_DEBUG("Precise Range Measure: %d", msg->PRM);
            rad105_DistL = msg->PRM;
            confirmed = true;
            return;
        }else if(msg-> msgID == *count && selector == 3){
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

bool send_and_recieve(td_navigation::listener& listener, int& to, hw_interface_plugin_timedomain::Range_Request& rr, ros::Publisher& rad_pub);

bool get_info(td_navigation::Localize::Request &req, td_navigation::Localize::Response &res){

  td_navigation::listener listener;

  ros::NodeHandle nh;

  //subscriber for recieving messages
  ros::Subscriber rad104_s = nh.subscribe("/radio104/data", 5, &td_navigation::listener::radCallBack, &listener);
  ros::Subscriber rad105_s = nh.subscribe("/radio105/data", 5, &td_navigation::listener::radCallBack, &listener);


  //publisher for sending a message to timedomain serial to get a range request
  ros::Publisher rad104_p = nh.advertise<hw_interface_plugin_timedomain::Range_Request>("/radio104/cmd", 5);
  ros::Publisher rad105_p = nh.advertise<hw_interface_plugin_timedomain::Range_Request>("/radio105/cmd", 5);

  //publisher for getting average angle measurements
  ros::Publisher aa_p = nh.advertise<td_navigation::Average_angle>("/average_angles", 1);

  int count = 0;
  //bool begin_avg = false;
  int average_length = req.average_length;
  listener.count = &count;
  //Hard coded numbers, should be launch params
  double dStation = 1930.4;
  int rad_L = 101;
  int rad_R = 106;
  double headings[average_length];
  double bearings[average_length];

  // double bot_x, bot_y, heading, bearing;
  int x_sum = 0;
  int y_sum = 0;
  int head_sum = 0;
  int bear_sum = 0;
  int h_dev_sum = 0;
  int b_dev_sum = 0;

  while(count <= average_length)
  {

      ROS_INFO("Count: %d, L_Count: %d", count, *(listener.count));

      hw_interface_plugin_timedomain::Range_Request rr;
      rr.send_range_request = true;

      rr.msgID = count;
      listener.confirmed = false;
      listener.selector = 0;

      //range request and response from 104 to 101
      if (send_and_recieve(listener, rad_L, rr, rad104_p) == false){
        //call a function to tell about the malfunction
        ROS_DEBUG("We didn't get a response in time!");
      }

      listener.selector = 1;
      rr.msgID = count;
      //range request and response from 104 to 106
      if (send_and_recieve(listener, rad_R, rr, rad104_p) == false){
        //call a function to tell about the malfunction
        ROS_DEBUG("We didn't get a response in time!");
      }

      listener.selector = 2;
      rr.msgID = count;
      //range request and response from 105 to 101
      if (send_and_recieve(listener, rad_L, rr, rad105_p) == false){
        //call a function to tell about the malfunction
        ROS_DEBUG("We didn't get a response in time!");
      }

      listener.selector = 3;
      rr.msgID = count;
      //range request and response from 105 to 106
      if (send_and_recieve(listener, rad_R, rr, rad105_p) == false){
        //call a function to tell about the malfunction
        ROS_DEBUG("We didn't get a response in time!");
      }


      radio_nav rad_nav;
      rad_nav.set_all(dStation, listener.rad104_DistL, listener.rad104_DistR, listener.rad105_DistL, listener.rad105_DistR);
      rad_nav.triangulate();

      x_sum += (rad_nav.get_rad0_x() + rad_nav.get_rad1_x()) / 2.0;
      y_sum += (rad_nav.get_rad0_y() + rad_nav.get_rad1_y()) / 2.0;
      head_sum += rad_nav.get_heading() * 180.0 / PI;
      bear_sum += rad_nav.get_bearing() * 180.0 / PI;

      //put the most recent measurement into the array
      headings[count] = rad_nav.get_heading() * 180.0 / PI;
      bearings[count] = rad_nav.get_bearing() * 180.0 / PI;

      // //angle that the bot is looking
      // heading = rad_nav.get_heading() * 180.0 / PI;
      //
      // //if angle is 0 we are looking straight forward (perpindicular to the stationary time domain ranging radios)
      // bearing = rad_nav.get_bearing() * 180.0 / PI;
      //
      // ROS_DEBUG("X: %e, Y: %e, Ang: %e, Bear: %e", bot_x, bot_y, heading, bearing);



      // if (count == average_length){
      //   begin_avg = true;
      // }

      // //take away the oldest value from the sum (only happens once we've started averaging)
      // if(begin_avg){
      //   head_sum -= headings[count%average_length];
      //   bear_sum -=  bearings[count%average_length];
      // }

  //     //put the most recent measurement into the array
  //     headings[count%average_length] = heading;
  //     bearings[count%average_length] = bearing;
  //
  //     //take the average of the 20 values, publish message with info
  //     if(begin_avg){
  //
  //       head_sum += headings[count%average_length];
  //       bear_sum +=  bearings[count%average_length];
  //
  //       td_navigation::Average_angle aa;
  //       aa.heading = head_sum/(double)average_length;
  //       aa.bearing = bear_sum/(double)average_length;
  //       aa.x = bot_x;
  //       aa.y = bot_y;
  //       aa_p.publish(aa);
  //
  // }

  //update count
  ++count;
  if(count >= 32000){
      count = 0;
  }

}//end while

//get the standard deviation
  for(int i = 0; i < average_length; i++){
    h_dev_sum += pow(headings[i], 2.0);
    b_dev_sum += pow(bearings[i], 2.0);
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

  ros::ServiceServer service = nh.advertiseService("localize", get_info);
  ROS_INFO("td_navigation node ready to Localize");
  ros::spin();

  ROS_DEBUG("td_navigation closing");
  return 0;
}

//send a range request and recieve the data back from the request
bool send_and_recieve(td_navigation::listener& listener, int& to, hw_interface_plugin_timedomain::Range_Request& rr, ros::Publisher& rad_pub){
  int wait = 0;
  int timeout = 0;
  ros::Rate loop_rate(200);

  while(!listener.confirmed){
    rr.radio_id_to_target = to;
    rad_pub.publish(rr);
    while(!listener.confirmed && wait < 100){
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
  listener.confirmed = false;
  return true;
}
