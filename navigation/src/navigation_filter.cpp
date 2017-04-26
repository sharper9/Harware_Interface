#include <navigation/navigation_filter.hpp>

NavigationFilter::NavigationFilter()
{
	sub_exec = nh.subscribe("/control/exec/info", 1, &NavigationFilter::getExecInfoCallback, this);
	pause_switch = false;
	stopFlag = false;
	turnFlag = false;

	filter.initialize_states(0,0,PI,1,0,filter.P_phi,filter.P_theta,filter.P_psi,filter.P_x,filter.P_y);

  encoders.set_wheel_radius(REG_WHEEL_RADIUS);
  encoders.set_counts_per_revolution_front(REG_WHEEL_COUNTS_PER_REV_FRONT);
  encoders.set_counts_per_revolution_back(REG_WHEEL_COUNTS_PER_REV_BACK);
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
		prev_stopped = false;
		collecting_accelerometer_data = false;
		collected_gyro_data = false;

		if (imu.new_nb1!=0) //if imu data is available
		{
			filter.turning(imu.nb1_p,imu.nb1_q,imu.nb1_r,imu.dt1); //predict state and covariance
		}
		else
		{
			filter.blind_turning(imu.nb1_p,imu.nb1_q,imu.nb1_r,imu.dt1); //predict covariance only
		}

		filter.clear_accelerometer_values();
		imu.clear_gyro_values();
	}
	else if (stopFlag) //if stopped
	{
		if (!prev_stopped && !stop_request) //this needs to be verified no obvious purpose for this statement
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

		if ((fabs(sqrt(imu.ax*imu.ax+imu.ay*imu.ay+imu.az*imu.az)-1)< 0.05 && sqrt((imu.p)*(imu.p)+(imu.q)*(imu.q)+(imu.r)*(imu.r))<0.005) && encoders.delta_distance == 0) //if no motion detected
		{
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

			if (0/*latest_nav_control_request.runBiasRemova*/) //run bias removal if command is sent (and no motion is detected from previous if)
			{
				if (imu.p1_values.size() > 500 && collected_gyro_data!=true) //if enough data points are collected and we did not already remove the bias
				{
					imu.calculate_gyro1_offset(); //calculate offset
					if(imu.good_bias1) //if offset does not exceed reasonable threshold
					{
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
					collected_gyro_data = true;
				}
				else
				{
					imu.collect_gyro1_data();
					collected_gyro_data = false;
				}
			}
		}
		else //if motion is detected then predict states
		{
			if (imu.new_nb1!=0)
			{
				filter.dead_reckoning(imu.nb1_p,imu.nb1_q,imu.nb1_r,encoders.delta_distance,imu.dt1);
			}
			else
			{
				filter.blind_dead_reckoning(imu.nb1_p,imu.nb1_q,imu.nb1_r,encoders.delta_distance,imu.dt1);
			}
		}
		prev_stopped = true;
	}
	else //if normal drive
	{
		prev_stopped = false;
		collecting_accelerometer_data = false;
		collected_gyro_data = false;
		filter.clear_accelerometer_values();
		imu.clear_gyro_values();
		if (imu.new_nb1!=0)
		{
			filter.dead_reckoning(imu.nb1_p,imu.nb1_q,imu.nb1_r,encoders.delta_distance,imu.dt1);
		}
		else
		{
			filter.blind_dead_reckoning(imu.nb1_p,imu.nb1_q,imu.nb1_r,encoders.delta_distance,imu.dt1);
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
	if (0/*latest_nav_control_request.runBiasRemoval*/ && collected_gyro_data) //output when bias removal is complete (if not performing bias removal output 0)
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
