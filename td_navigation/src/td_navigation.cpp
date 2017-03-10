#include <td_navigation/td_navigation.h>

//Range_Request_()
//  : send_range_request(false)
//  , radio_id_to_target(0)
//  , msgID(0)  {
//  }


//RCM_Range_Info_()
//  : RespondingNode(0)
//  , AntennaMode(0)
//  , rangeStatus(0)
//  , PRM(0)
//  , CRE(0)
//  , FRE(0)
//  , PRMError(0)
//  , CREError(0)
//  , FREError(0)
//  , rangeType(0)
//  , busy(false)
//  , failed(false)
//  , msgID(0)  {
//  }

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

void td_navigation::listener::radCallBack(const hw_interface_plugin_timedomain::RCM_Range_Info::ConstPtr &msg){
    ROS_INFO("TDRR Navigation Callback");
    //check the message id, busy, failed
    //update appropriate distance
    if (msg->busy == true){
        ROS_WARN("TDRR was busy");
        return;
    }
    if (msg->failed == true){
        ROS_WARN("TDRR request has failed!");
        return;

    }

    //from rad0 to leftRad
    if(msg->msgID >=0 && msg->msgID < 400 ){
        if(msg->msgID == *count && selector == 0){
            ROS_INFO("Reading from rad0 to leftRad");
            ROS_INFO("Precise Range Measure: %d", msg->PRM);
            rad104_DistL = msg->PRM;
            confirmed = true;
        }else if(msg-> msgID == 100 + *count && selector == 1){
            ROS_INFO("Reading from rad0 to RightRad");
            ROS_INFO("Precise Range Measure: %d", msg->PRM);
            rad104_DistR = msg->PRM;
            confirmed = true;
            return;
        }else if(msg-> msgID == 200 + *count && selector == 2){
            ROS_INFO("Reading from rad1 to leftRad");
            ROS_INFO("Precise Range Measure: %d", msg->PRM);
            rad105_DistL = msg->PRM;
            confirmed = true;
            return;
        }else if(msg-> msgID == 300 + *count && selector == 3){
            ROS_INFO("Reading from rad1 to RightRad");
            ROS_INFO("Precise Range Measure: %d", msg->PRM);
            rad105_DistR = msg->PRM;
            confirmed = true;
            return;

        }else{
            ROS_WARN("TDRR MsgID mismatched!");
            return;
        }

    }else{
        ROS_WARN("TDRR MsgID Very Wrong!");
    }
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

    td_navigation::listener listener;


    //subscriber for recieving messages

    ros::Subscriber rad104_s = nh.subscribe("/radio104/data", 5, &td_navigation::listener::radCallBack, &listener);
    ros::Subscriber rad105_s = nh.subscribe("/radio105/data", 5, &td_navigation::listener::radCallBack, &listener);


    //publisher for sending a message to timedomain serial to get a range request
    ros::Publisher rad104_p = nh.advertise<hw_interface_plugin_timedomain::Range_Request>("/radio104/cmd", 5);
    ros::Publisher rad105_p = nh.advertise<hw_interface_plugin_timedomain::Range_Request>("/radio105/cmd", 5);


    ros::Rate loop_rate(5);


    int count = 0;
    listener.count = &count;
    //temporary value
    const double dStation = 850.0;
    double d0l, d0r, d1l, d1r;
    double angle_104, rad104_x, rad104_y;
    double angle_105, rad105_x, rad105_y;
    double bot_x, bot_y, angle_bot;
    double temp;

    while(nh.ok())
    {
        //test
        ROS_INFO("Count: %d, L_Count: %d", count, *(listener.count));

        hw_interface_plugin_timedomain::Range_Request rr;
        rr.send_range_request = true;

        rr.msgID = count;
        listener.confirmed = false;
        listener.selector = 0;

        while(!listener.confirmed){
            //publish a range request from rad104 to leftRad
            rr.radio_id_to_target = 101;
            rad104_p.publish(rr);
            ros::spinOnce();
            //for testing
            loop_rate.sleep();
            //test
            //ROS_INFO("Woot0");

        }

        listener.confirmed = false;
        listener.selector = 1;
        rr.msgID = 100 + count;

        while(!listener.confirmed){
            //publish a range request from rad104 to RightRad
            rr.radio_id_to_target = 106;
            rad104_p.publish(rr);
            ros::spinOnce();
            //for Testing
            loop_rate.sleep();
            //test
            //ROS_INFO("Woot1");

        }

        listener.confirmed = false;
        listener.selector = 2;
        rr.msgID = 200 + count;

        while(listener.confirmed != true){
            //publish a range request from rad104 to rightRad
            //publish a range request
            rr.radio_id_to_target = 101;

            rad105_p.publish(rr);
            ros::spinOnce();
            //for Testing
            loop_rate.sleep();
            //ROS_INFO("Woot2");
        }

        listener.confirmed = false;
        listener.selector = 3;
        rr.msgID = 300 + count;

        while(listener.confirmed != true){
            //publish a range request from rad1041 to leftRad
            //publish a range request
            rr.radio_id_to_target = 106;

            rad105_p.publish(rr);
            ros::spinOnce();
            //for Testing
            loop_rate.sleep();
            //ROS_INFO("Woot3");
        }


        listener.confirmed = false;

        d0l = listener.rad104_DistL;
        d0r = listener.rad104_DistR;
        d1l = listener.rad105_DistL;
        d1r = listener.rad105_DistR;


        ROS_DEBUG("d0l %e, d0r %e, d1l %e, d1r %e", d0l, d0r, d1l, d1r);

        //triangulation of rad_0
        temp = pow(d0r,2.0) + pow(dStation,2.0) - pow(d0l,2.0);
        angle_104 = acos(temp/(2.0 * dStation * d0l));
        rad104_x = (d0r * cos(angle_104)) - (dStation/(2.0));
        rad104_y = d0r * sin(angle_104);

        ROS_DEBUG("X_0: %e, Y_0: %e, Ang_0: %e", rad104_x, rad104_y, (angle_104* 180.0 / PI) );

        //triangulation of rad_1
        angle_105 = acos((pow(d1r,2.0) + pow(dStation,2.0) - pow(d1l,2.0))/(2.0 * dStation * d1l));
        rad105_x = (d1r * cos(angle_105)) - (dStation/(2.0));
        rad105_y = d1r * sin(angle_105);

        ROS_DEBUG("X_1: %e, Y_1: %e, Ang_1: %e", rad105_x, rad105_y, (angle_105* 180.0 / PI));

        //bot_x and bot_y describe the point in between the two tdrr on the bot, not sure how accurate this'll be once the hardware is there
        bot_x = (rad104_x + rad105_x)/2.0;
        bot_y = (rad104_y + rad105_y)/2.0;



        //if angle is 0 we are looking straight forward (perpindicular to the stationary time domain ranging radios)
        //as long as the tdrr are in the places I thought they were
        angle_bot = atan((rad105_y - rad104_y)/(rad105_x - rad104_x));

        ROS_DEBUG("X: %e, Y: %e, Ang: %e", bot_x, bot_y, (angle_bot * 180.0 / PI));


        ++count;
        if(count >= 100){
            count = 0;
        }
        loop_rate.sleep();
    }

    ROS_DEBUG("td_navigation Closing");
    return 0;
}
