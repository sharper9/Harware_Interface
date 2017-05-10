#include <ros/ros.h>

#include <hw_interface_plugin_agent/hw_interface_plugin_agent_UDP.hpp>


hw_interface_plugin_agent::agent_UDP::agent_UDP()
{
    ROS_DEBUG_EXTRA_SINGLE("A Wild agent Plugin Appeared!");
    //localAddress = boost::asio::ip::address::from_string("192.168.2.122");

    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
           ros::console::notifyLoggerLevelsChanged();
        }
    enableMetrics();
    lastLOStime=ros::Time::now();
    lastMsgTimeStamp = 0.0;
    agentTimeOnFirstReceipt = 0.0;
    agentOpTimeOffset = 0.0;
    opTimeOnFirstReceipt = 0.0;
    agentTimeOnLastMsg=0.0;
    instantLatency=0.0;
    expectedHeartBeatRate_ms = 50;
    operatorConnected = false;
}

void hw_interface_plugin_agent::agent_UDP::msgCallback(const topic_tools::ShapeShifter::ConstPtr &msg, uint8_t type)
{
    if(operatorConnected)
    {
        ROS_DEBUG("Sending Scan");
        if((type!=MSG_TYPE_CAMERA_IMAGE)&&(type!=MSG_TYPE_LASER_SCAN))
        {
            postInterfaceWriteRequest(const_shared_buf_agent(*msg,type));
        }
        else
        {
            boost::lock_guard<boost::mutex> guard(webCamCmdMutex);
            if(webCamCmdFromCommand.start && (type==MSG_TYPE_CAMERA_IMAGE))
            {
                ROS_INFO("Camera IMAGE");
                postInterfaceWriteRequest(const_shared_buf_agent(*msg,type));
            }
            else if(!(webCamCmdFromCommand.start) && (type==MSG_TYPE_LASER_SCAN))
            {
                ROS_INFO("scan");
                postInterfaceWriteRequest(const_shared_buf_agent(*msg,type));
            }
        }
    }
}

void hw_interface_plugin_agent::agent_UDP::webCamCmdCallback(const msgs_and_srvs::WebcamCommands::ConstPtr &msg)
{
    boost::lock_guard<boost::mutex> guard(webCamCmdMutex);
    webCamCmdFromCommand=*msg;
}


bool hw_interface_plugin_agent::agent_UDP::pluginStart()
{
    ROS_INFO("Agent Starting");

    LOSUpdateTimer.reset(new boost::asio::deadline_timer(interfaceSocket->get_io_service(),
                                                                boost::posix_time::milliseconds(1000)));
    LOSUpdateTimer->expires_from_now(boost::posix_time::milliseconds(1000));
    LOSUpdateTimer->async_wait(boost::bind(&hw_interface_plugin_agent::agent_UDP::LOSTimeoutHandler, this, boost::asio::placeholders::error()));
    return true;
}

bool hw_interface_plugin_agent::agent_UDP::subPluginInit(ros::NodeHandlePtr nhPtr)
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

    if(ros::param::get(pluginName+"/topicSubscription", tempString))
    {
        //rosDataSub = ros::NodeHandlePtr
    }
    if(!(ros::param::get(pluginName+"/heartbeatRate_ms", expectedHeartBeatRate_ms)))
    {
        ROS_WARN_EXTRA_NAME("Using deafult response timeout%s"," ");
        expectedHeartBeatRate_ms=50;
    }

    LOSPub = nhPtr->advertise<hw_interface_plugin_agent::LOS>("/agent/LOS",1,true);
    webCamCmdPub = nhPtr->advertise<msgs_and_srvs::WebcamCommands>("/agent/operatorIP",1,true);

    scanSub = nhPtr->subscribe<topic_tools::ShapeShifter>("/scan",1,boost::bind(&hw_interface_plugin_agent::agent_UDP::msgCallback,this,_1,MSG_TYPE_LASER_SCAN));
    bumperParamsSub = nhPtr->subscribe<topic_tools::ShapeShifter>("/command/interface/vvirtualbumperparams",1,boost::bind(&hw_interface_plugin_agent::agent_UDP::msgCallback,this,_1,MSG_TYPE_LASER_SCAN));
    armPositionSub = nhPtr->subscribe<topic_tools::ShapeShifter>("/arm/position",1,boost::bind(&hw_interface_plugin_agent::agent_UDP::msgCallback,this,_1,MSG_TYPE_ARM_POSITION));
    bumperStatusSub = nhPtr->subscribe<topic_tools::ShapeShifter>("/lidar/lidar/virtualbumperstatus",1,boost::bind(&hw_interface_plugin_agent::agent_UDP::msgCallback,this,_1,MSG_TYPE_BUMPER_STATUS));
    driveCommandsSub = nhPtr->subscribe<topic_tools::ShapeShifter>("/command/interface/drivecommands",1,boost::bind(&hw_interface_plugin_agent::agent_UDP::msgCallback,this,_1,MSG_TYPE_DRIVE_CMDS));
    armModeSub = nhPtr->subscribe<topic_tools::ShapeShifter>("/command/interface/armmanualmode",1,boost::bind(&hw_interface_plugin_agent::agent_UDP::msgCallback,this,_1,MSG_TYPE_ARM_MODE));
    cameraImageSub = nhPtr->subscribe<topic_tools::ShapeShifter>("/webcam/publishimages/image",1,boost::bind(&hw_interface_plugin_agent::agent_UDP::msgCallback,this,_1,MSG_TYPE_CAMERA_IMAGE));
    webCamCmdSub = nhPtr->subscribe<msgs_and_srvs::WebcamCommands>("/command/interface/webcam",1,&hw_interface_plugin_agent::agent_UDP::webCamCmdCallback,this);

//    nhPtr->setParam("~image_transport","compressed");
//    image_transport::ImageTransport it(*nhPtr);
//    imageTransportSub = it.subscribe("/webcam/publishimages/image",1,&hw_interface_plugin_agent::agent_UDP::compressedImageTransportCallback,this);

    localPort = 0;
    remotePort = 0;
    ros::param::get(pluginName+"/remotePort", remotePort);
    ros::param::get(pluginName+"/localPort", localPort);

    joyPublisher = nhPtr->advertise<sensor_msgs::Joy>("/joy", 1, true);

    ROS_INFO("%s :: Remote IP: %s :: Remote Port: %d :: Local Port: %d",
                pluginName.c_str(), remoteAddress.to_string().c_str(), remotePort, localPort );

    return true;
}

void hw_interface_plugin_agent::agent_UDP::LOSTimeoutHandler(const boost::system::error_code& error)
{
    if(error!=boost::asio::error::operation_aborted)
    {
        boost::lock_guard<boost::mutex> guard(LOSTimeMutex);
        ROS_DEBUG("Checking LOS");
        if((ros::Time::now().toSec()-lastLOStime.toSec())>=1)
        {
            //we are LOS
            ROS_WARN("We have LOS");
            hw_interface_plugin_agent::LOS losMsg;
            losMsg.LOS=1;
            LOSPub.publish(losMsg);
        }
        else
        {
            hw_interface_plugin_agent::LOS losMsg;
            losMsg.LOS=0;
            LOSPub.publish(losMsg);
            msgs_and_srvs::WebcamCommands webCamCmd;
            webCamCmd.serverAddress=remoteEndpoint->address().to_string();
            webCamCmdPub.publish(webCamCmd);
        }
    }
    LOSUpdateTimer->expires_from_now(boost::posix_time::milliseconds(1000));
    LOSUpdateTimer->async_wait(boost::bind(&hw_interface_plugin_agent::agent_UDP::LOSTimeoutHandler, this, boost::asio::placeholders::error()));
}


bool hw_interface_plugin_agent::agent_UDP::interfaceReadHandler(const size_t &bufferSize, int arrayStartPos)
{
    double currentAgentTime = ros::Time::now().toSec();
    //ROS_INFO("agent Plugin Data Handler %lu  %d", bufferSize, arrayStartPos);
    //for(uint32_t i = 0; i < bufferSize; i++)
   // {
   //     std::printf("%x %c || ", receivedData[i],receivedData[i]);
   // }
    //std::printf("\r\n");

    ROS_INFO_THROTTLE(2,"Remote Address?: %s", senderEndpoint.address().to_string().c_str());

    ROS_DEBUG_THROTTLE(1,"Incoming Pkt timestamp %2.9f", *((double*) &(receivedData[MSG_TIMESTAMP_OFFSET])));
    ROS_DEBUG_THROTTLE(1,"Time at Receipt:       %2.9f", currentAgentTime);
    if(lastMsgTimeStamp < *((double*) &(receivedData[MSG_TIMESTAMP_OFFSET])))
    {
        //ROS_DEBUG("New Packet");
        if(agentTimeOnLastMsg)
        {
            instantLatency = ((currentAgentTime-agentTimeOnLastMsg) - ((double)expectedHeartBeatRate_ms)/1000.0);
            ROS_INFO_THROTTLE(1,"Instant Latency %2.9f", instantLatency);
        }
        if(instantLatency)
        {
            agentOpTimeOffset = currentAgentTime-(*((double*) &(receivedData[MSG_TIMESTAMP_OFFSET])))-instantLatency/2;
            ROS_INFO_THROTTLE(1,"Agent to Operator Time Offset %2.9f", agentOpTimeOffset);
        }
        agentTimeOnLastMsg = currentAgentTime;
        lastMsgTimeStamp = *((double*) &(receivedData[MSG_TIMESTAMP_OFFSET]));
        if(receivedData[MSG_TYPE_OFFSET]==MSG_TYPE_JOY)
        {
            try {
                //joy msg
                //ROS_DEBUG("Joy Msg");
                //ROS_DEBUG("Creating Stream");
                ros::serialization::IStream stream((uint8_t*)(&(receivedData[MSG_OFFSET])), ros::serialization::serializationLength(newJoyVal));
                //ROS_DEBUG("reading into msg");
                ros::serialization::Serializer<sensor_msgs::Joy>::read(stream, newJoyVal);
                //ROS_DEBUG("publishing");
                joyPublisher.publish(newJoyVal);
            }
            catch(std::exception e)
            {
                ROS_ERROR("STD Exception Caught! %s", e.what());
            }
        }
        else if(receivedData[MSG_TYPE_OFFSET]==MSG_TYPE_HEARTBEAT)
        {
            ROS_INFO_THROTTLE(1,"Got Heartbeat");
            boost::lock_guard<boost::mutex> guard(LOSTimeMutex);
            lastLOStime=ros::Time::now();
            //reset LOS timer
            if(!operatorConnected)
            {
                opTimeOnFirstReceipt=*((double*) &(receivedData[MSG_TIMESTAMP_OFFSET]));
                agentTimeOnFirstReceipt=currentAgentTime;

                remoteAddress=senderEndpoint.address();
                *remoteEndpoint=senderEndpoint;
                operatorConnected=true;
            }
        }


    }

    return true;
}

bool hw_interface_plugin_agent::agent_UDP::verifyChecksum()
{
    return true;
}
