#include <ros/ros.h>

#include <hw_interface_plugin_netburner/hw_interface_plugin_netburner_UDP.hpp>


hw_interface_plugin_netburner::netburner_UDP::netburner_UDP()
{
    ROS_DEBUG_EXTRA_SINGLE("A Wild NB Plugin Appeared!");
    //localAddress = boost::asio::ip::address::from_string("192.168.2.122");

    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
           ros::console::notifyLoggerLevelsChanged();
        }
    enableMetrics();
}

bool hw_interface_plugin_netburner::netburner_UDP::subPluginInit(ros::NodeHandlePtr nhPtr)
{
    ROS_DEBUG_EXTRA("%s Plugin Init", pluginName.c_str());

    std::string tempString = "";
    if(!ros::param::get(pluginName+"/remoteAddress", tempString))
    {
        ROS_WARN_EXTRA_SINGLE("failed to find remote address param");
    }
    else
    {
        remoteAddress = boost::asio::ip::address::from_string(tempString.c_str());
    }

    if(ros::param::get(pluginName+"/publishToTopic", tempString))
    {
        nbDataPub = nhPtr->advertise<messages::nb1_to_i7_msg>(tempString, 1);
    }
    else
    {
        ROS_ERROR("Could not find publish topic name!!");
    }

    localPort = 0;
    remotePort = 0;
    ros::param::get(pluginName+"/remotePort", remotePort);
    ros::param::get(pluginName+"/localPort", localPort);

    ROS_INFO("%s :: Remote IP: %s :: Remote Port: %d :: Local Port: %d",
                pluginName.c_str(), remoteAddress.to_string().c_str(), remotePort, localPort );

    return true;
}


bool hw_interface_plugin_netburner::netburner_UDP::interfaceReadHandler(const size_t &bufferSize, int arrayStartPos, const boost::system::error_code &ec)
{
    //ROS_DEBUG_EXTRA_SINGLE("NB Plugin Data Handler");
    
    nbDataMsg_t *nbMsg = (nbDataMsg_t*)(&(receivedData[0]));
    
    nbROSMsg.counter         	= nbMsg->counter;
    nbROSMsg.nb_clock			    = ((double)nbMsg->clock_reg_count + ((double)nbMsg->clock_reg_reset_count * ((double) 0xFFFFFFFF + 1.0))) / (double)125000000.0;
    nbROSMsg.acc_x 			      = nbMsg->accX1*.00025/65536.0;
    nbROSMsg.acc_y 			      = nbMsg->accY1*0.00025/65536.0;
    nbROSMsg.acc_z 			      = -1.0*nbMsg->accZ1*0.00025/65536.0;
    nbROSMsg.rate_p          	= nbMsg->rateQ1*0.02/65536.0; //quick and dirty transform
    nbROSMsg.rate_q          	= nbMsg->rateP1*0.02/65536.0;
    nbROSMsg.rate_r          	= nbMsg->rateR1*0.02/65536.0;
    nbROSMsg.num_imus         = nbMsg->imuStatus;
    nbROSMsg.counter         	= nbMsg->counter;
    nbROSMsg.counter         	= nbMsg->counter;
    nbROSMsg.mainLoopCounter  = nbMsg->mainLoopCounter;
    nbROSMsg.i7_clock         = ros::Time::now().toSec();
    
    nbDataPub.publish(nbROSMsg);
    
    
    return true;
}

bool hw_interface_plugin_netburner::netburner_UDP::verifyChecksum()
{
    return true;
}
