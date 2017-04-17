#include <navigation/imu_class.hpp>


IMU::IMU()
{
	start_time = ros::Time::now().toSec();

	//combined imus
	bias_thresh = 0.01;
	p=0;
	q=0;
	r=0;
	ax=0;
	ay=0;
	az=0;

	//netburner1
	nb1_p=0;
	nb1_q=0;
	nb1_r=0;
	nb1_ax=0;
	nb1_ay=0;
	nb1_az=0;
    nb1_num_imus=0;
	nb1_imu_nums=0;
	nb1_counter=0;
	nb1_counter_prev = 0;
	nb1_diff_prev = 1;
	nb1_drive_counter = 0;
    nb1_current = true;
	nb1_good = true;
	nb1_good_prev = true;
	call_counter1=0;
	prev_time1 = 0;
	dt1 = 0;
	new_nb1 = 0;
	subscriber_imu1 = node.subscribe("hw_interface/nb1in/nb1in", 1, &IMU::getIMU1Callback,this);

	//imu1
	p1=0;
	q1=0;
	r1=0;
	ax1=0;
	ay1=0;
	az1=0;
	p1_offset = 0.00155578029808;
	q1_offset = 0.000418158248067;
	r1_offset = 0.000437778187916;
	E_p1_offset = 0.00155578029808;
	E_q1_offset = 0.000418158248067;
	E_r1_offset = 0.000437778187916;
	mean_p1 = 0;
	mean_q1 = 0;
	mean_r1 = 0;
	good_bias1 = true;
	p1_values.clear();
	q1_values.clear();
	r1_values.clear();
	new_imu1 = 0;
	imu_1_good = false;
}

void IMU::collect_gyro1_data()
{
	if (new_imu1!=0)
	{
		p1_values = arma::join_vert(p1_values,arma::mat(1,1,arma::fill::ones)*(p1+p1_offset));
		q1_values = arma::join_vert(q1_values,arma::mat(1,1,arma::fill::ones)*(q1+q1_offset));
		r1_values = arma::join_vert(r1_values,arma::mat(1,1,arma::fill::ones)*(r1+r1_offset));
	}
}

void IMU::calculate_gyro1_offset()
{
	arma::mat p_holder1 = p1_values;
	arma::mat q_holder1 = q1_values;
	arma::mat r_holder1 = r1_values;
	arma::mat p_holder2;
	arma::mat q_holder2;
	arma::mat r_holder2;
	arma::mat p_meds;
	arma::mat q_meds;
	arma::mat r_meds;
	arma::mat m_temp;
	good_bias1 = true;
	
	// p_est
	double m_p = 0;
	int s_p = 0;
	double min_dist_p = 0;
	int min_index_p = 0;
	p_holder2.clear();
	p_meds.clear();
	for (int jj = 0; jj<25; jj++)
	{
		m_temp = arma::median(arma::median(p_holder1));
		m_p = m_temp(0,0);
		s_p = p_holder1.size();
		min_dist_p = fabs(p_holder1(0,0)-m_p);
		min_index_p = 0;
		for (int ii = 1; ii<s_p; ii++)
		{
			if (fabs(p_holder1(ii,0)-m_p) < min_dist_p)
			{
				min_dist_p = fabs(p_holder1(ii,0)-m_p);
				min_index_p = ii;
			}
		}
		
		for (int ii = 0; ii<s_p; ii++)
		{
			if (ii != min_index_p)
			{
				p_holder2 = arma::join_vert(p_holder2,arma::mat(1,1,arma::fill::ones)*p_holder1(ii,0));
			}
			else
			{
				p_meds = arma::join_vert(p_meds,arma::mat(1,1,arma::fill::ones)*p_holder1(ii,0));
			}
		}
		p_holder1.clear();
		p_holder1 = p_holder2;
		p_holder2.clear();
	}
	mean_p1 = arma::mean(arma::mean(p_meds));
	if (fabs(mean_p1-E_p1_offset)>bias_thresh)
	{
		good_bias1 = false;
	}

	// q_est
	double m_q = 0;
	int s_q = 0;
	double min_dist_q = 0;
	int min_index_q = 0;
	q_holder2.clear();
	q_meds.clear();
	for (int jj = 0; jj<25; jj++)
	{
		m_temp = arma::median(arma::median(q_holder1));
		m_q = m_temp(0,0);
		s_q = q_holder1.size();
		min_dist_q = fabs(q_holder1(0,0)-m_q);
		min_index_q = 0;
		for (int ii = 1; ii<s_q; ii++)
		{
			if (fabs(q_holder1(ii,0)-m_q) < min_dist_q)
			{
				min_dist_q = fabs(q_holder1(ii,0)-m_q);
				min_index_q = ii;
			}
		}
		
		for (int ii = 0; ii<s_q; ii++)
		{
			if (ii != min_index_q)
			{
				q_holder2 = arma::join_vert(q_holder2,arma::mat(1,1,arma::fill::ones)*q_holder1(ii,0));
			}
			else
			{
				q_meds = arma::join_vert(q_meds,arma::mat(1,1,arma::fill::ones)*q_holder1(ii,0));
			}
		}
		q_holder1.clear();
		q_holder1 = q_holder2;
		q_holder2.clear();
	}
	mean_q1 = arma::mean(arma::mean(q_meds));
	if (fabs(mean_q1-E_q1_offset)>bias_thresh)
	{
		good_bias1 = false;
	}

	// r_est
	double m_r = 0;
	int s_r = 0;
	double min_dist_r = 0;
	int min_index_r = 0;
	r_holder2.clear();
	r_meds.clear();
	for (int jj = 0; jj<25; jj++)
	{
		m_temp = arma::median(arma::median(r_holder1));
		m_r = m_temp(0,0);
		s_r = r_holder1.size();
		min_dist_r = fabs(r_holder1(0,0)-m_r);
		min_index_r = 0;
		for (int ii = 1; ii<s_r; ii++)
		{
			if (fabs(r_holder1(ii,0)-m_r) < min_dist_r)
			{
				min_dist_r = fabs(r_holder1(ii,0)-m_r);
				min_index_r = ii;
			}
		}
		
		for (int ii = 0; ii<s_r; ii++)
		{
			if (ii != min_index_r)
			{
				r_holder2 = arma::join_vert(r_holder2,arma::mat(1,1,arma::fill::ones)*r_holder1(ii,0));
			}
			else
			{
				r_meds = arma::join_vert(r_meds,arma::mat(1,1,arma::fill::ones)*r_holder1(ii,0));
			}
		}
		r_holder1.clear();
		r_holder1 = r_holder2;
		r_holder2.clear();
	}
	mean_r1 = arma::mean(arma::mean(r_meds));
	if (fabs(mean_r1-E_r1_offset)>bias_thresh)
	{
		good_bias1 = false;
	}
	
}

void IMU::clear_gyro_values()
{
	p1_values.clear();
	q1_values.clear();
	r1_values.clear();
}

void IMU::clear_gyro1_values()
{
	p1_values.clear();
	q1_values.clear();
	r1_values.clear();
}

void IMU::set_gyro1_offset()
{
	if(good_bias1)
	{
		p1_offset = mean_p1;
		q1_offset = mean_q1;
		r1_offset = mean_r1;
	}
}

void IMU::determine_new_data()
{
	if (nb1_counter!=nb1_counter_prev)
	{
		new_nb1 = 1;
	}
	else
	{
		new_nb1 = 0;
	}
}

void IMU::filter_imu_values()
{
		
	if (new_nb1 == 1)
	{
		if (prev_time1!=0)
		{
			dt1 = time1 - prev_time1;
		}
		else 
		{
			dt1 = 0.0;
		}
        if (fabs(dt1)>0.1)
		{
            dt1 = 0.1;
		}

		p1 = p1-p1_offset;
		q1 = q1-q1_offset;
		r1 = r1-r1_offset;

        if (imu_1_good)
		{
            nb1_p = p1;
            nb1_q = q1;
            nb1_r = r1;
            nb1_ax = ax1;
            nb1_ay = ay1;
            nb1_az = az1;
		}
		else
		{
			nb1_p = 0;
			nb1_q = 0;
			nb1_r = 0;
			nb1_ax = 0;
			nb1_ay = 0;
			nb1_az = 0;
		}

	}
	else
	{
        dt1 = 0;
        nb1_p = 0;
		nb1_q = 0;
		nb1_r = 0;
		nb1_ax = 0;
		nb1_ay = 0;
		nb1_az = 0;
		new_imu1 = 0;
		nb1_num_imus = 0;
	}

    if (nb1_num_imus!=0 || !nb1_good)
    {
        p = nb1_p;
        q = nb1_q;
        r = nb1_r;
        ax = nb1_ax;
        ay = nb1_ay;
        az = nb1_az;
    }
    else
    {
        p = 0;
        q = 0;
        r = 0;
        ax = 0;
        ay = 0;
        az = 0;
    }

}

void IMU::set_prev_counters()
{
	if (nb1_counter_prev == nb1_counter)
	{
		nb1_missed_counter = nb1_missed_counter+1;
        if (nb1_missed_counter>1)
		{
			nb1_current = false;
			nb1_drive_counter = nb1_drive_counter+1;
		}
	}
	else if (nb1_counter_prev == nb1_counter-1)
	{
		nb1_missed_counter = 0;
		nb1_good = true;
		if(nb1_diff_prev==1)
		{
			nb1_current = true;
			nb1_drive_counter = 0;
		}
	}
	else
	{
		nb1_missed_counter = 0;
		nb1_current = false;
		nb1_good = true;
		nb1_drive_counter = nb1_drive_counter-1;
	}

	nb1_diff_prev = nb1_counter - nb1_counter_prev;
	nb1_counter_prev = nb1_counter;
	prev_time1 = time1;
}
