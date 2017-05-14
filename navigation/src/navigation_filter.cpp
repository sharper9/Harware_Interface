#include <navigation/navigation_filter.hpp>

NavigationFilter::NavigationFilter()
{
    sub_mission_planning = nh.subscribe("/control/missionplanning/info", 1, &NavigationFilter::getMissionPlanningInfoCallback, this);
    sub_exec = nh.subscribe("/control/exec/info", 1, &NavigationFilter::getExecInfoCallback, this);
    pause_switch = false;
    stopFlag = true;
    turnFlag = false;

    ranging_radio_client = nh.serviceClient<td_navigation::Localize>("localize");
    sub_rr_half_pose = nh.subscribe("/Half_Pose", 1, &NavigationFilter::getRRHalfPose, this);

    //void Filter::initialize_states(double phi_init, double theta_init, double psi_init, double x_init, double y_init, double P_phi_init, double P_theta_init, double P_psi_init, double P_x_init, double P_y_init)
    filter.initialize_states(0,0,initHeading,initX,initY,filter.P_phi,filter.P_theta,filter.P_psi,filter.P_x,filter.P_y);

#ifdef USE_COMP_WHEELS
    encoders.set_wheel_radius(COMP_WHEEL_RADIUS);
    encoders.set_counts_per_revolution_front_right(COMP_WHEEL_COUNTS_PER_REV_FRONT_RIGHT);
    encoders.set_counts_per_revolution_front_left(COMP_WHEEL_COUNTS_PER_REV_FRONT_LEFT);
    encoders.set_counts_per_revolution_back_right(COMP_WHEEL_COUNTS_PER_REV_BACK_RIGHT);
    encoders.set_counts_per_revolution_back_left(COMP_WHEEL_COUNTS_PER_REV_BACK_LEFT);
#else
    encoders.set_wheel_radius(REG_WHEEL_RADIUS);
    encoders.set_counts_per_revolution_front_right(REG_WHEEL_COUNTS_PER_REV_FRONT_RIGHT);
    encoders.set_counts_per_revolution_front_left(REG_WHEEL_COUNTS_PER_REV_FRONT_LEFT);
    encoders.set_counts_per_revolution_back_right(REG_WHEEL_COUNTS_PER_REV_BACK_RIGHT);
    encoders.set_counts_per_revolution_back_left(REG_WHEEL_COUNTS_PER_REV_BACK_LEFT);
#endif // USE_COMP_WHEELS

    current_time = ros::Time::now().toSec();

    //added for new User Interface -Matt G.
    std::string tempServiceName = "";
    if(!(ros::param::get("NavControlServiceName", tempServiceName)))
    {
        //if the parameter does not exist, use this default one
        tempServiceName = "/navigation/navigationfilter/control";
    }
    //    nav_control_server = nh.advertiseService(tempServiceName,
    //                                                &NavigationFilter::navFilterControlServiceCallback,
    //                                                    this);

}

void NavigationFilter::update_time()
{
    dt = ros::Time::now().toSec() - current_time;
    current_time = ros::Time::now().toSec();
}

void NavigationFilter::run()
{
    if (turnFlag) //if turing in place
    {
        rr_found_full_pose=false;
        rr_full_pose_failed_counter = 0;
        prev_stopped = false;
        collecting_accelerometer_data = false;
        collected_gyro_data = false;
        //ROS_INFO("imu.new_nb1 = %i", imu.new_nb1);
        if (imu.new_nb1!=0) //if data from netburner 1 is available (this is based on netburner counters)
        {
            filter.turning(imu.p,imu.q,imu.r,imu.dt); //predict state and covariance
        }
        else
        {
            filter.blind_turning(imu.p,imu.q,imu.r,imu.dt); //predict covariance only
        }

        filter.clear_accelerometer_values();
        imu.clear_gyro_values();
    }
    else if (stopFlag) //if stopped
    {
        //re-initialize position if ranging radios are available

        if (!prev_stopped && !stop_request)
        {
            if(imu.nb1_current)
            {
                imu.nb1_good_prev = true;
            }
        }

        if (!prev_stopped && !collecting_accelerometer_data) //if not previously stopped and not collecting accel data then start collecting accel data
        {
            collecting_accelerometer_data = true;
        }

        //if no motion detected
        if (/*(fabs(sqrt(imu.ax*imu.ax+imu.ay*imu.ay+imu.az*imu.az)-1)< 0.175) &&*/
                (sqrt((imu.p)*(imu.p)+(imu.q)*(imu.q)+(imu.r)*(imu.r))<0.0175) && encoders.delta_distance == 0) //if no motion detected
        {
            if(perform_rr_heading_update && !rr_found_full_pose && rr_full_pose_failed_counter<rr_full_pose_failed_max_count)
            {
                td_navigation::Localize rr_srv;
                rr_srv.request.average_length = 10; // value to be changed

                if(ranging_radio_client.call(rr_srv))
                {
                    double rr_heading; //radians
                    double rr_x;
                    double rr_y;
                    if(!rr_initial_pose_found || fabs(rr_srv.response.heading - filter.psi) < rr_heading_update_tolerance) // Trust RR updated heading
                    {
                        if(!rr_initial_pose_found || ((sqrt(pow(filter.x+rr_srv.response.x,2)+pow(filter.y+rr_srv.response.y,2))) < rr_stopped_position_update_tolerence))
                        {
                            rr_initial_pose_found=true;
                            rr_x = rr_srv.response.x;
                            rr_y = rr_srv.response.y;
                            rr_heading = rr_srv.response.heading;
                            //void Filter::initialize_states(double phi_init, double theta_init, double psi_init, double x_init, double y_init, double P_phi_init, double P_theta_init, double P_psi_init, double P_x_init, double P_y_init)
                            filter.initialize_states(filter.phi, filter.theta, rr_heading, rr_x, rr_y, filter.P_phi, 0.05, filter.P_psi, 1.0, 1.0);
                            rr_found_full_pose = true;
                            rr_full_pose_failed_counter = 0;
                        }
                    }
                    else // RR heading update too far off from current heading. Do not trust
                    {
                        rr_found_full_pose = false;
                        rr_full_pose_failed_counter++;
                        ROS_WARN("RR heading too far from current heading. Not updating pose.");
                    }
                }
                else // RR service request failed
                {
                    rr_found_full_pose = false;
                    rr_full_pose_failed_counter++;
                    ROS_WARN("RR service request failed. Not updating pose.");
                }
            }

            if (collecting_accelerometer_data) //if accel data is set to be started
            {
                if (filter.ax_values.size() > 50) //check number of data points collected is enough
                {
                    collecting_accelerometer_data = false;
                    filter.roll_pitch_G_update(); //update attitude
                    filter.clear_accelerometer_values();
                }
                else
                {
                    if(imu.new_nb1!=0) //check if data is available to save
                    {
                        filter.collect_accelerometer_data(imu.ax, imu.ay, imu.az);
                    }
                }
            }

            if (1) //always run bias removal when stopping if no motion is detected
            {
                //ROS_INFO("running bias removal");
                if (imu.p1_values.size() > NUMBER_OF_DATA_POINTS_BIAS_REMOVAL && collected_gyro_data!=true) //if enough data points are collected and we did not already remove the bias
                {
                    //ROS_INFO("calculating bias offset");
                    imu.calculate_gyro1_offset(); //calculate offset
                    if(imu.good_bias1) //if offset does not exceed reasonable threshold
                    {
                        //ROS_INFO("setting bias offset");
                        collected_gyro_data = true;
                        imu.set_gyro1_offset();
                        filter.Q_phi = 2.2847e-008;
                        filter.Q_theta = 2.2847e-008;
                        filter.Q_psi = 2.2847e-008;
                    }
                    else
                    {
                        imu.clear_gyro1_values();
                    }
                }
                else if (collected_gyro_data==true)
                {
                    //ROS_INFO("set collecting gyro data");
                    collected_gyro_data = true;
                }
                else
                {
                    //ROS_INFO("collecting gyro data");
                    imu.collect_gyro1_data();
                    collected_gyro_data = false;
                }
            }
        }
        else //if motion is detected then predict states
        {
            ROS_WARN("Stop Flag set but motion detected! %2.12f  %2.12f %2.3f", 
                                    fabs(sqrt(imu.ax*imu.ax+imu.ay*imu.ay+imu.az*imu.az)-1), 
                                    sqrt((imu.p)*(imu.p)+(imu.q)*(imu.q)+(imu.r)*(imu.r)),
                                    encoders.delta_distance);
            rr_found_full_pose=false;
            rr_full_pose_failed_counter = 0;
            if (imu.new_nb1!=0)
            {
                filter.dead_reckoning(imu.nb1_p,imu.nb1_q,imu.nb1_r,encoders.delta_distance,imu.dt);
            }
            else
            {
                filter.blind_dead_reckoning(imu.nb1_p,imu.nb1_q,imu.nb1_r,encoders.delta_distance,imu.dt);
            }
        }
        prev_stopped = true;
    }
    else //if normal drive
    {
        prev_stopped = false;
        collecting_accelerometer_data = false;
        collected_gyro_data = false;
        rr_found_full_pose=false;
        rr_full_pose_failed_counter = 0;
        filter.clear_accelerometer_values();
        imu.clear_gyro_values();

        if((fabs((sqrt(pow(filter.x+rr_new_pose.x,2)+pow(filter.y+rr_new_pose.y,2))-sqrt(pow(filter.x,2)+pow(filter.y,2)))) < rr_position_update_moving_tolerence)) //half pose update
        {
            //void Filter::initialize_states(double phi_init, double theta_init, double psi_init, double x_init, double y_init, double P_phi_init, double P_theta_init, double P_psi_init, double P_x_init, double P_y_init)
            filter.initialize_states(filter.phi, filter.theta, filter.psi, rr_new_pose.x, rr_new_pose.y, filter.P_phi, 0.05, filter.P_psi, 1.0, 1.0);
            rr_new_half_pose=false;
        }
        if (imu.new_nb1!=0)
        {
            filter.dead_reckoning(imu.nb1_p,imu.nb1_q,imu.nb1_r,encoders.delta_distance,imu.dt);
        }
        else
        {
            filter.blind_dead_reckoning(imu.nb1_p,imu.nb1_q,imu.nb1_r,encoders.delta_distance,imu.dt);
        }
    }

    //send stop request if imu data is missing
    if (imu.nb1_missed_counter>50 && imu.nb1_good && stop_request == false)
    {
        stop_request = true;
        stop_time = ros::Time::now().toSec();
    }
    else if (stop_request == true && imu.nb1_current)
    {
        stop_request = false;

        if(imu.nb1_current)
        {
            imu.nb1_good_prev = true;
        }
    }
    else if (ros::Time::now().toSec()-stop_time>20.0 && stop_request == true)
    {
        stop_request = false;
        if (imu.nb1_missed_counter>50)
        {
            imu.nb1_good = false;
            imu.nb1_good_prev = false;
        }
    }
    else if (stop_request == false)
    {
        stop_time = ros::Time::now().toSec();
    }

    // TODO: latest_nav_control_request compile error
    if (collected_gyro_data) //output status when bias removal is complete (if not performing bias removal output 0 always)
    {
        nav_status_output = 1;
    }
    else
    {
        nav_status_output = 0;
    }

}

void NavigationFilter::getExecInfoCallback(const messages::ExecInfo::ConstPtr &msg)
{
    this->pause_switch = msg->pause;
    this->turnFlag = msg->turnFlag;
    this->stopFlag = msg->stopFlag;
}

void NavigationFilter::getMissionPlanningInfoCallback(const messages::MissionPlanningInfo::ConstPtr &msg)
{
    perform_rr_heading_update = msg->performFullPose;
}

void NavigationFilter::getRRHalfPose(const td_navigation::Running_Half_Pose::ConstPtr &msg)
{
    rr_new_half_pose=rr_initial_pose_found;
    rr_new_pose=*msg;
}

////added for new User Interface -Matt G.
//bool NavigationFilter::navFilterControlServiceCallback(messages::NavFilterControl::Request &request, messages::NavFilterControl::Response &response)
//{
//    this->latest_nav_control_request = request;
//    response.pOffset = imu.p_offset;
//    response.qOffset = imu.q_offset;
//    response.rOffset = imu.r_offset;

//    if(request.setGlobalPose)
//    {
//        ROS_INFO("Teleporting to new Location");
//        filter.initialize_states(filter.phi,filter.theta,request.newHeading*DEG_2_RAD,request.newX,request.newY,filter.P_phi,filter.P_theta,filter.P_psi,filter.P_x,filter.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
//        homing_updated = true;
//    }

//    return true;
//}
