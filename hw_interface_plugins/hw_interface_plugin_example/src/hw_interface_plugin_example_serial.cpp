#include <ros/ros.h>

#include <hw_interface_plugin_example/hw_interface_plugin_example_serial.hpp>

//class constructor, do required instatiation only here
    //do not setup other things as they my not have been setup by the parent classes yet
hw_interface_plugin_example::example_serial::example_serial()
{
    //A debug message
    ROS_INFO("A Wild Example Plugin Appeared!");

    //force the ROS Console level to Debug Level
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
           ros::console::notifyLoggerLevelsChanged();
        }
        
    //enable automatic class metric collection.
    enableMetrics();
}

//this is called each time the plugin is enabled, before anything else of the plugin is called
bool hw_interface_plugin_example::example_serial::subPluginInit(ros::NodeHandlePtr nhPtr)
{
    ROS_DEBUG_EXTRA("%s Plugin Init", pluginName.c_str());

    /*for Serial interfaces, 'deviceName' is an inherited member and must be defined.
        failing to define this variable will disable the plugin.
        Opening of the device port is handled automatically
        
        deviceName is the name and path of the port to be opened
            example: "/dev/ttyS0" */
    deviceName = "";
    ros::param::get(pluginName+"/deviceName", deviceName);
    
    headerString = "FFab";
    footerString = "dbac";
    readLength = 26;
    setupStreamMatcherDelimAndLength(readLength, headerString.c_str(),
                                        footerString.c_str());

    //retrieve string from the ROS Parameter Server
        //of the format '<plugin_name>/<parameter>
    std::string tempString = "";
    
    //place that wants to write data to the device
    if(ros::param::get(pluginName+"/subscribeToTopic", tempString))
    {
        //rosDataSub = nh->subscribe(tempString, 1, &roboteq_drive::rosMsgCallback, this);
    }
    else
    {
        ROS_ERROR("%s:: Could not find topic subscription name", pluginName.c_str());
    }

    //place to publish the data after reading from device
    if(ros::param::get(pluginName+"/publishToTopic", tempString))
    {
        //rosDataPub = nh->advertise<messages::encoder_data>(tempString, 1, false);
    }
    else
    {
        ROS_ERROR("%s:: Could not find topic advertisment name", pluginName.c_str());
    }

    return true;
}

/*this function is called to setup the port
    typical serial port setup uses 115200 baud rate, 8 bit character size, no flow control,
      no parity, 1 stop bit. This is typical, but no all encompassing. Change this if
      the port requires different. */
void hw_interface_plugin_example::example_serial::setInterfaceOptions()
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

//this is called automatically when data that passes the streamMatcher is okay
    //this function is called with a data length and a position in an inherited array member
        //named 'receivedData'
//data should be published to topics from here
bool hw_interface_plugin_example::example_serial::interfaceReadHandler(const long &length,
                                                                            int arrayStartPos)
{
    ROS_INFO_EXTRA_SINGLE("Example Plugin Data Handler");
    
    ROS_DEBUG("Buf Pointer: 0x%p\r\n", &receivedData[arrayStartPos]);
    std::printf("Contents: ");
    for(int i = 0; i < length; i++)
    {
        std::printf("%x | ", receivedData[arrayStartPos + i]);
    }
    std::printf("\r\n");
    
    return true;
}

//automatically called to check the checksum of the packet. 
    //If un-wanted/un-needed, return true.
bool hw_interface_plugin_example::example_serial::verifyChecksum()
{
    return true;
}

void hw_interface_plugin_example::example_serial::rosMsgCallback(const messages::ActuatorOut::ConstPtr &msgIn)
{
   //this function is called when a ros message comes in
}
