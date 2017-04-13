#include <navigation/navigation_filter.hpp>

NavigationFilter::NavigationFilter()
{ 
	sub_exec = nh.subscribe("/control/exec/info", 1, &NavigationFilter::getExecInfoCallback, this);
	pause_switch = false;
	stopFlag = false;
	turnFlag = false;

	sub_mission = nh.subscribe("/control/missionplanning/info", 1, &NavigationFilter::getMissionPlanningInfoCallback, this);

	sub_lidar = nh.subscribe("lidar/lidarfilteringout/lidarfilteringout", 1, &NavigationFilter::getLidarFilterOutCallback, this);
    homing_x=0.0;
	homing_y=0.0;
	homing_heading=0.0;
    homing_found=false;
    dull_x=0.0;
	dull_y=0.0;
	shiny_x=0.0;
	shiny_y=0.0;
	cylinder_std = 100.0;
	registration_counter = 0;
	registration_counter_prev = 0;

	filter.initialize_states(0,0,PI,1,0,filter.P_phi,filter.P_theta,filter.P_psi,filter.P_x,filter.P_y);
	init_filter.initialize_states(0,0,PI,1,0,filter.P_phi,filter.P_theta,filter.P_psi,filter.P_x,filter.P_y);
	filter1.initialize_states(0,0,PI,1,0,filter.P_phi,filter.P_theta,filter.P_psi,filter.P_x,filter.P_y);
	filter2.initialize_states(0,0,PI,1,0,filter.P_phi,filter.P_theta,filter.P_psi,filter.P_x,filter.P_y);
	filterS.initialize_states(0,0,PI,1,0,filter.P_phi,filter.P_theta,filter.P_psi,filter.P_x,filter.P_y);

	encoders.set_wheel_radius(0.2286/2);
	encoders.set_counts_per_revolution(4476.16*1.062);
	current_time = ros::Time::now().toSec();

    //added for new User Interface -Matt G.
    std::string tempServiceName = "";
    if(!(ros::param::get("NavControlServiceName", tempServiceName)))
    {
        //if the parameter does not exist, use this default one
        tempServiceName = "/navigation/navigationfilter/control";
    }
    nav_control_server = nh.advertiseService(tempServiceName,
                                                &NavigationFilter::navFilterControlServiceCallback,
                                                    this);

}

void NavigationFilter::update_time()
{
	dt = ros::Time::now().toSec() - current_time;
	current_time = ros::Time::now().toSec();
}

void NavigationFilter::run()
{
	// Predict Methods

	if (turnFlag)
	{
		prev_stopped = false;
		collecting_accelerometer_data = false;
		collected_gyro_data = false;

		if (imu.new_nb1!=0)
		{
			filter1.turning(imu.nb1_p,imu.nb1_q,imu.nb1_r,imu.dt1);
		}
		else
		{
			filter1.blind_turning(imu.nb1_p,imu.nb1_q,imu.nb1_r,imu.dt1);
		}

		filter.clear_accelerometer_values();
		imu.clear_gyro_values();
	}
	else if (stopFlag)
	{
		if (!prev_stopped && !stop_request)
		{
			if(imu.nb1_current)
			{
				imu.nb1_good_prev = true;
			}
		}


		if (!prev_stopped&&!collecting_accelerometer_data)
		{
			collecting_accelerometer_data = true;
		}
	
		if ((fabs(sqrt(imu.ax*imu.ax+imu.ay*imu.ay+imu.az*imu.az)-1)< 0.05 && sqrt((imu.p)*(imu.p)+(imu.q)*(imu.q)+(imu.r)*(imu.r))<0.005) && encoders.delta_distance == 0)
		{
			if (collecting_accelerometer_data)
			{
				if (filter.ax_values.size() > 50)
				{
					collecting_accelerometer_data = false;
					filter.roll_pitch_G_update();
					filter.clear_accelerometer_values();
				}
				else
				{
                                        if(imu.new_nb1=0)
					{
						filter.collect_accelerometer_data(imu.ax, imu.ay, imu.az);
					}
				}
			}
			if (latest_nav_control_request.runBiasRemoval)
			{
				if (imu.p1_values.size() > 500 && collected_gyro1_data!=true)
				{
					imu.calculate_gyro1_offset();
					if(imu.good_bias1)
					{
						collected_gyro1_data = true;
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
				else if (collected_gyro1_data==true)
				{
					collected_gyro1_data = true;
				}
				else
				{
					imu.collect_gyro1_data();
					collected_gyro1_data = false;
				}
			}
		}
		else
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
	else
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
	else if (stop_request == false)
	{
            stop_time = ros::Time::now().toSec();
	}

	if (latest_nav_control_request.runBiasRemoval && collected_gyro_data)
	{
            nav_status_output = 1;
	}
	else
	{
            nav_status_output = 0;
	}
    }

}

void NavigationFilter::getExecInfoCallback(const messages::ExecInfo::ConstPtr &msg)
{
	this->pause_switch = msg->pause;
	this->turnFlag = msg->turnFlag;
	this->stopFlag = msg->stopFlag;
}

//added for new User Interface -Matt G.
bool NavigationFilter::navFilterControlServiceCallback(messages::NavFilterControl::Request &request, messages::NavFilterControl::Response &response)
{
    this->latest_nav_control_request = request;
    response.pOffset = imu.p_offset;
    response.qOffset = imu.q_offset;
    response.rOffset = imu.r_offset;
    
    if(request.setGlobalPose)
    {
        ROS_INFO("Teleporting to new Location");
        filter.initialize_states(filter.phi,filter.theta,request.newHeading*DEG_2_RAD,request.newX,request.newY,filter.P_phi,filter.P_theta,filter.P_psi,filter.P_x,filter.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y    	
        homing_updated = true;
    }
    
    return true;
}
