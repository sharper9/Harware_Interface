#include <hw_interface_plugin_timedomain/hw_interface_plugin_timedomain_serial.hpp>

//class constructor, do required instatiation only here
    //do not setup other things as they my not have been setup by the parent classes yet
hw_interface_plugin_timedomain::timedomain_serial::timedomain_serial()
{
    //A debug message
    ROS_INFO("A Wild Timedomain Plugin Appeared!");

    //force the ROS Console level to Debug Level
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info) ) {
           ros::console::notifyLoggerLevelsChanged();
        }

    //enable automatic class metric collection.
    enableMetrics();
}

//this is called each time the plugin is enabled, before anything else of the plugin is called
bool hw_interface_plugin_timedomain::timedomain_serial::subPluginInit(ros::NodeHandlePtr nhPtr)
{
    ROS_DEBUG_EXTRA("%s Plugin Init", pluginName.c_str());

    /*for Serial interfaces, 'deviceName' is an inherited member and must be defined.
        failing to define this variable will disable the plugin.
        opening of the device port is handled automatically */

    const unsigned char head[] = {0xa5,0xa5,0x00};
    headerString = std::string((const char *)head);


    deviceName = "";
    ros::param::get(pluginName+"/deviceName", deviceName);

    //retrieve string from the ROS Parameter Server
        //of the format '<plugin_name>/<parameter>
    std::string tempString = "";

    //place that wants to write data to the device
    if(ros::param::get(pluginName+"/subscribeToTopic", tempString))
    {
        rosDataSub = nhPtr->subscribe(tempString, 1, &timedomain_serial::rosMsgCallback, this);
    }
    else
    {
        ROS_ERROR("%s:: Could not find topic subscription name", pluginName.c_str());
    }

    //place to publish the data after reading from device
    if(ros::param::get(pluginName+"/publishToTopic", tempString))
    {
        rosDataPub = nhPtr->advertise<hw_interface_plugin_timedomain::RCM_Range_Info>(tempString, 1, false);
    }
    else
    {
        ROS_ERROR("%s:: Could not find topic advertisment name", pluginName.c_str());
    }

    setupStreamMatcherDelimAndLength(ranging_radio_types::SEND_RANGE_CONFIRM_SIZE,headerString.c_str(),"");

    return true;
}

/*this function is called to setup the port
    typical serial port setup uses 115200 baud rate, 8 bit character size, no flow control,
      no parity, 1 stop bit. This is typical, but no all encompassing. Change this if
      the port requires different. */
void hw_interface_plugin_timedomain::timedomain_serial::setInterfaceOptions()
{
	int tempBaudRate = 0;
    ros::param::get(pluginName+"/baudrate", tempBaudRate);
    setOption<boost::asio::serial_port_base::baud_rate>(
                new boost::asio::serial_port_base::baud_rate(tempBaudRate));
    //8 bits per character
    setOption<boost::asio::serial_port_base::character_size>(
    			new boost::asio::serial_port_base::character_size( 8 ));

    //flow control
    setOption<boost::asio::serial_port_base::flow_control>(
    			new boost::asio::serial_port_base::flow_control(
    										boost::asio::serial_port_base::flow_control::type::none));

    //parity
    setOption<boost::asio::serial_port_base::parity>(
    			new boost::asio::serial_port_base::parity(
    										boost::asio::serial_port_base::parity::type::none));

    //stop-bits
    setOption<boost::asio::serial_port_base::stop_bits>(
    			new boost::asio::serial_port_base::stop_bits(
    										boost::asio::serial_port_base::stop_bits::type::one));

    ROS_INFO("%s :: Device: %s :: Baudrate %d", pluginName.c_str(), deviceName.c_str(), tempBaudRate);
}

void hw_interface_plugin_timedomain::timedomain_serial::rosMsgCallback(const hw_interface_plugin_timedomain::Range_Request::ConstPtr &msg)
{
    ROS_DEBUG_THROTTLE(2,"Timedomain callback");
    if(!isRequestInProgress() && msg->send_range_request && (msg->radio_id_to_target >= 0))
    {
        ROS_DEBUG("Sending Range Request");
        requestInProgress = true;
        ranging_radio_types::Range_Request_t rangeRequest;
        rangeRequest.msg.sync = 0xA5A5;
        rangeRequest.msg.length = ranging_radio_types::SEND_RANGE_REQUEST_SIZE-4; //packet length is length -4
        rangeRequest.msg.msgType = SEND_RANGE_REQUEST_MSGTYPE;
        rangeRequest.msg.msgID = msg->msgID;
        rangeRequest.msg.rspndNode = msg->radio_id_to_target;
        rangeRequest.msg.antenMode = 0;
        rangeRequest.msg.reserved = 0;
        rangeRequest.msg.dataSize = 0;
        rangeRequest.msg.data = 0;
        readLength = ranging_radio_types::SEND_RANGE_CONFIRM_SIZE;
        setupStreamMatcherDelimAndLength(ranging_radio_types::SEND_RANGE_CONFIRM_SIZE,headerString.c_str(),"");
        postInterfaceWriteRequest(hw_interface_support_types::shared_const_buffer(rangeRequest.msgDataChar, ranging_radio_types::SEND_RANGE_REQUEST_SIZE, true));
    }
    else
    {
        ROS_WARN("Radio is busy");
        hw_interface_plugin_timedomain::RCM_Range_Info rangeFailed;
        rangeFailed.msgID = msg->msgID;
        rangeFailed.busy = isRequestInProgress();
        rangeFailed.failed = true;

        rosDataPub.publish(rangeFailed);
    }
}

//this is called automatically when data that passes the streamMatcher is okay
    //this function is called with a data length and a position in an inherited array member
        //named 'receivedData'
bool hw_interface_plugin_timedomain::timedomain_serial::interfaceReadHandler(const size_t &length, int arrayStartPos, const boost::system::error_code &ec)
{
    ROS_DEBUG_THROTTLE(2,"TimeDomain Plugin Data Handler, %ld, %d", length, arrayStartPos);

    ROS_DEBUG("Buf Pointer: 0x%p\r\n", &receivedData[arrayStartPos]);
    /*
    
    std::printf("Contents: ");
    for(int i = 0; i < length; i++)
    {
        std::printf("%x | ", receivedData[arrayStartPos + i]);
    }
    std::printf("\r\n");
    
    */

    ranging_radio_types::Pre_Response_t preRead;
    ranging_radio_types::bigToLittleEndian((const uint8_t*)receivedData.get(), preRead.msgData, ranging_radio_types::PRE_READ_SIZE);

    if(preRead.msg.msgType == SEND_RANGE_CONFIRM_MSGTYPE)
    {
        ROS_DEBUG_EXTRA("%s Received Range Confirm", pluginName.c_str());
        //because the last message received was a range confirm, the next message will be a range info
        readLength = ranging_radio_types::RCM_RANGE_INFO_SIZE;
        setupStreamMatcherDelimAndLength(ranging_radio_types::RCM_RANGE_INFO_SIZE,headerString.c_str(),"");
    }
    else if(preRead.msg.msgType == RCM_RANGE_INFO_MSGTYPE)
    {
        ROS_DEBUG_EXTRA("%s Received Range Info", pluginName.c_str());
        requestInProgress = false;
        readLength = ranging_radio_types::SEND_RANGE_CONFIRM_SIZE;
        setupStreamMatcherDelimAndLength(ranging_radio_types::SEND_RANGE_CONFIRM_SIZE,headerString.c_str(),"");

        ranging_radio_types::Ranging_Radio_Data_t rangeData;
        ranging_radio_types::bigToLittleEndian((const uint8_t*)receivedData.get(), rangeData.msgData, ranging_radio_types::RCM_RANGE_INFO_SIZE);
        hw_interface_plugin_timedomain::RCM_Range_Info rosRangeMsg;
        rosRangeMsg.RespondingNode = rangeData.msg.responderID;
        rosRangeMsg.AntennaMode = rangeData.msg.antenMode;
        rosRangeMsg.rangeStatus = rangeData.msg.rangeStatus;
        rosRangeMsg.PRM = rangeData.msg.PRM;
        rosRangeMsg.CRE = rangeData.msg.CRE;
        rosRangeMsg.FRE = rangeData.msg.FRE;
        rosRangeMsg.PRMError = rangeData.msg.PRMError;
        rosRangeMsg.CREError = rangeData.msg.CREError;
        rosRangeMsg.FREError = rangeData.msg.FREError;
        rosRangeMsg.rangeType = rangeData.msg.rangeMeasType;
        rosRangeMsg.busy = false;
        rosRangeMsg.failed = (rangeData.msg.rangeStatus != 0);
        rosRangeMsg.msgID = rangeData.msg.msgID;

        rosDataPub.publish(rosRangeMsg);
    }
    return true;
}

//automatically called to check the checksum of the packet.
    //If un-wanted/un-needed, return true.
bool hw_interface_plugin_timedomain::timedomain_serial::verifyChecksum()
{
    return true;
}
