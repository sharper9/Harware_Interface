#include <ros/ros.h>

#include <hw_interface_plugin_operator/hw_interface_plugin_operator_UDP.hpp>


hw_interface_plugin_operator::operator_UDP::operator_UDP()
{
    ROS_DEBUG_EXTRA_SINGLE("A Wild Operator Plugin Appeared!");
    //localAddress = boost::asio::ip::address::from_string("192.168.2.122");

    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info) ) {
           ros::console::notifyLoggerLevelsChanged();
        }
    enableMetrics();
    lastMsgTimeStamp = 0.0;
    agentTimeOnFirstReceipt = 0.0;
    agentOpTimeOffset = 0.0;
    opTimeOnFirstReceipt = 0.0;
    agentTimeOnLastMsg=0.0;
    instantLatency=0.0;
    joyUpdateRate_ms=50;
    heartbeatRate_ms=50;
    gotOneJoy=false;
    agentConnected=false;
}



void hw_interface_plugin_operator::operator_UDP::rosMsgCallback(const topic_tools::ShapeShifter::ConstPtr &msg, std::string &topicName)
{
//    ROS_INFO("Topic name %s", topicName.c_str());
//    //ROS_INFO("Data Type %s", msg->getDataType().c_str());
//    //ROS_INFO("Def %s", msg->getMessageDefinition().c_str());
//    std::array<uint8_t,500> arr;
//    arr.fill(0);

//    ros::serialization::IStream is(arr.data(), 500);
//    ROS_INFO("Reading into buf %u", msg->size());
//    msg->write(is);


//    for(uint32_t i = 0; i < msg->size(); i++)
//    {
//        std::printf("%x || ", arr[i]);
//    }
//    std::printf("\r\n");

//    ros::serialization::Serializer<topic_tools::ShapeShifter>::write(is, *msg);
//    ros::NodeHandle nh;
//    rosPub=msg->advertise(nh,"/topic/test",1,false);
//    rosPub.publish(*msg);


//    postInterfaceWriteRequest(const_shared_buf_operator(*msg,1,1));

    boost::lock_guard<boost::mutex> guardMsg(msgMutex);
    gotOneJoy=true;
    lastJoyMsg=*msg;

}

void hw_interface_plugin_operator::operator_UDP::joyTimeoutHandler(const boost::system::error_code& error)
{
    joyUpdateTimer->cancel();
    if(error!=boost::asio::error::operation_aborted)
    {
        //stuff
        ROS_DEBUG_THROTTLE(2,"sending joy pkt");
        boost::lock_guard<boost::mutex> guardMsg(msgMutex);
        if(gotOneJoy)
        {
            postInterfaceWriteRequest(const_shared_buf_operator(lastJoyMsg,1));
        }
    }
    joyUpdateTimer->expires_from_now(boost::posix_time::milliseconds(joyUpdateRate_ms));
    joyUpdateTimer->async_wait(boost::bind(&hw_interface_plugin_operator::operator_UDP::joyTimeoutHandler, this, boost::asio::placeholders::error()));
}

void hw_interface_plugin_operator::operator_UDP::heartbeatTimeoutHandler(const boost::system::error_code &error)
{
    heartbeatUpdateTimer->cancel();
    if(error!=boost::asio::error::operation_aborted)
    {
        ROS_DEBUG("Operator:: Sending Heartbeat");
        postInterfaceWriteRequest(const_shared_buf_operator(0xff));
    }
    heartbeatUpdateTimer->expires_from_now(boost::posix_time::milliseconds(heartbeatRate_ms));
    heartbeatUpdateTimer->async_wait(boost::bind(&hw_interface_plugin_operator::operator_UDP::heartbeatTimeoutHandler, this, boost::asio::placeholders::error()));
}

bool hw_interface_plugin_operator::operator_UDP::subPluginInit(ros::NodeHandlePtr nhPtr)
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
    subscriberVector.push_back(nhPtr->subscribe<topic_tools::ShapeShifter>("/joy", 1,
                                                                  boost::bind(&hw_interface_plugin_operator::operator_UDP::rosMsgCallback, this, _1, std::string("/test/shifter"))));


    if(!(ros::param::get(pluginName+"/updateRate_ms", joyUpdateRate_ms)))
    {
        ROS_WARN_EXTRA_NAME("Using deafult response timeout%s"," ");
        joyUpdateRate_ms=50;
    }
    if(!(ros::param::get(pluginName+"/heartbeatRate_ms", heartbeatRate_ms)))
    {
        ROS_WARN_EXTRA_NAME("Using deafult response timeout%s"," ");
        heartbeatRate_ms=50;
    }

    scanPublisher = nhPtr->advertise<messages::NavFilterOut>("/navigation/navigationfilterout/navigationfilterout",1,false);

    localPort = 0;
    remotePort = 0;
    ros::param::get(pluginName+"/remotePort", remotePort);
    ros::param::get(pluginName+"/localPort", localPort);

    ROS_INFO("%s :: Remote IP: %s :: Remote Port: %d :: Local Port: %d",
                pluginName.c_str(), remoteAddress.to_string().c_str(), remotePort, localPort );

    return true;
}

bool hw_interface_plugin_operator::operator_UDP::pluginStart()
{
    ROS_INFO("Operator Starting");
    joyUpdateTimer.reset(new boost::asio::deadline_timer(interfaceSocket->get_io_service(),
                                                                boost::posix_time::milliseconds(joyUpdateRate_ms)));
    joyUpdateTimer->expires_from_now(boost::posix_time::milliseconds(joyUpdateRate_ms));
    joyUpdateTimer->async_wait(boost::bind(&hw_interface_plugin_operator::operator_UDP::joyTimeoutHandler, this, boost::asio::placeholders::error()));

    heartbeatUpdateTimer.reset(new boost::asio::deadline_timer(interfaceSocket->get_io_service(),
                                                                boost::posix_time::milliseconds(heartbeatRate_ms)));
    heartbeatUpdateTimer->expires_from_now(boost::posix_time::milliseconds(heartbeatRate_ms));
    heartbeatUpdateTimer->async_wait(boost::bind(&hw_interface_plugin_operator::operator_UDP::heartbeatTimeoutHandler, this, boost::asio::placeholders::error()));
    return true;
}


bool hw_interface_plugin_operator::operator_UDP::interfaceReadHandler(const size_t &bufferSize, int arrayStartPos, const boost::system::error_code &ec)
{
    double currentAgentTime = ros::Time::now().toSec();
    ROS_DEBUG("Operator Plugin Data Handler %lu  %d", bufferSize, arrayStartPos);
    //for(uint32_t i = 0; i < bufferSize; i++)
    //{
    //   std::printf("%x %c || ", receivedData[i],receivedData[i]);
    //}
    //std::printf("\r\n");

    ROS_DEBUG("Remote Address?: %s", senderEndpoint.address().to_string().c_str());

    ROS_DEBUG("Incoming Pkt timestamp %2.9f", *((double*) &(receivedData[MSG_TIMESTAMP_OFFSET])));
    ROS_DEBUG("Time at Receipt:       %2.9f", currentAgentTime);
    if(lastMsgTimeStamp < *((double*) &(receivedData[MSG_TIMESTAMP_OFFSET])))
    {
        ROS_DEBUG("New Packet");
//        if(agentTimeOnLastMsg)
//        {
//            instantLatency = ((currentAgentTime-agentTimeOnLastMsg) - ((double)expectedHeartBeatRate_ms)/1000.0);
//            ROS_DEBUG("Instant Latency %2.9f", instantLatency);
//        }
//        if(instantLatency)
//        {
//            agentOpTimeOffset = currentAgentTime-(*((double*) &(receivedData[MSG_TIMESTAMP_OFFSET])))-instantLatency/2;
//            ROS_DEBUG("Agent to Operator Time Offset %2.9f", agentOpTimeOffset);
//        }
        agentTimeOnLastMsg = currentAgentTime;
        lastMsgTimeStamp = *((double*) &(receivedData[MSG_TIMESTAMP_OFFSET]));
        try {
            if(receivedData[MSG_TYPE_OFFSET]==MSG_TYPE_LASER_SCAN)
            {
                    //scan msg
                    ROS_DEBUG("scan Msg");
                    ROS_DEBUG("Creating Stream");
                    ros::serialization::IStream stream((uint8_t*)(&(receivedData[MSG_OFFSET])), ros::serialization::serializationLength(lastScan));
                    ROS_DEBUG("reading into msg");
                    ros::serialization::Serializer<messages::NavFilterOut>::read(stream, lastScan);
                    ROS_DEBUG("publishing");
                    scanPublisher.publish(lastScan);

            }/*else if(receivedData[MSG_TYPE_OFFSET]==MSG_TYPE_BUMPER_STATUS)
            {
                ROS_DEBUG("bumper stat msg");

                //ros::serialization::IStream stream((uint8_t*)(&(receivedData[MSG_OFFSET])), ros::serialization::serializationLength())
            }*/
        }
        catch(std::exception e)
        {
            ROS_ERROR("STD Exception Caught! %s", e.what());
        }
//        else if(receivedData[MSG_TYPE_OFFSET]==MSG_TYPE_HEARTBEAT)
//        {
//            ROS_INFO_THROTTLE(1,"Got Heartbeat");
//            boost::lock_guard<boost::mutex> guard(LOSTimeMutex);
//            lastLOStime=ros::Time::now();
//            //reset LOS timer
//            if(!operatorConnected)
//            {
//                opTimeOnFirstReceipt=*((double*) &(receivedData[MSG_TIMESTAMP_OFFSET]));
//                agentTimeOnFirstReceipt=currentAgentTime;

//                remoteAddress=senderEndpoint.address();
//                *remoteEndpoint=senderEndpoint;
//                operatorConnected=true;
//            }
//        }


    }

    return true;
}

bool hw_interface_plugin_operator::operator_UDP::verifyChecksum()
{
    return true;
}
