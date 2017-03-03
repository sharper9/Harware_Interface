#include <hw_interface_plugin_roboteq/hw_interface_plugin_roboteq.hpp>

hw_interface_plugin_roboteq::roboteq_serial::roboteq_serial()
{
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
    {
        ros::console::notifyLoggerLevelsChanged();
    }
    ROS_INFO_EXTRA_SINGLE("Roboteq Plugin Instantiated");
}

bool hw_interface_plugin_roboteq::roboteq_serial::subPluginInit(ros::NodeHandlePtr nhPtr)
{
    nh = ros::NodeHandlePtr(nhPtr);

    std::string tempString = "";
    if(ros::param::get(pluginName+"/controllerType", tempString))
    {
    
        if (!tempString.compare("Right_Drive")) { roboteqType = controller_t::Right_Drive_Roboteq; }
        else if(!tempString.compare("Left_Drive")) { roboteqType = controller_t::Left_Drive_Roboteq; }
        else if(!tempString.compare("Bucket_Roboteq")) { roboteqType = controller_t::Bucket_Roboteq; }
        else { roboteqType = controller_t::Other; }
    }
    else
    {
        ROS_ERROR("%s:: No Controller Type Specifiecompleted!", pluginName.c_str());
    }

    ROS_DEBUG("%s:: Implementation Init", pluginName.c_str());
    implInit();
    ROS_INFO_EXTRA_SINGLE("Roboteq Plugin Init");
    enableMetrics();

    enableRegexReadUntil = true;
    regexExpr = "(CB|A|AI|AIC|BS|DI|DR|FF|BAR|BA){1}=(-?\\d+):(-?\\d+):(-?\\d+):(-?\\d+)(\\r){2}";

    deviceName = "";
    ros::param::get(pluginName+"/deviceName", deviceName);

    return true;
}

void hw_interface_plugin_roboteq::roboteq_serial::rosMsgCallback(const messages::ActuatorOut::ConstPtr &msgIn)
{
    std::string motorSpeedCmds = "";

    if(roboteqType == controller_t::Right_Drive_Roboteq)
    {
        motorSpeedCmds += "!G 1 " + boost::lexical_cast<std::string>(msgIn->fr_speed_cmd) + "\r";
        motorSpeedCmds += "!G 2 " + boost::lexical_cast<std::string>(msgIn->mr_speed_cmd) + "\r";
        postInterfaceWriteRequest(hw_interface_support_types::shared_const_buffer(motorSpeedCmds));
    }
    else if(roboteqType == controller_t::Left_Drive_Roboteq)
    {
        motorSpeedCmds += "!G 1 " + boost::lexical_cast<std::string>(msgIn->fl_speed_cmd) + "\r";
        motorSpeedCmds += "!G 2 " + boost::lexical_cast<std::string>(msgIn->ml_speed_cmd) + "\r";
        postInterfaceWriteRequest(hw_interface_support_types::shared_const_buffer(motorSpeedCmds));
    }
    else if(roboteqType == controller_t::Bucket_Roboteq)
    {
        motorSpeedCmds += "!G 1 " + boost::lexical_cast<std::string>(-msgIn->bucket_pos_cmd) + "\r";
        motorSpeedCmds += "!G 2 " + boost::lexical_cast<std::string>(msgIn->bucket_pos_cmd) + "\r";
        postInterfaceWriteRequest(hw_interface_support_types::shared_const_buffer(motorSpeedCmds));      
    }
    else
    {
        ROS_WARN("%s:: No Data written because of Incorrect Roboteq Type", pluginName.c_str());
    }
    
    ROS_INFO("%s", motorSpeedCmds.c_str());

    //need to add monitoring facilities to monitor health
}

bool hw_interface_plugin_roboteq::roboteq_serial::implInit()
{
    std::string tempString;
    if(ros::param::get(pluginName+"/subscribeToTopic", tempString))
    {
        rosDataSub = nh->subscribe(tempString, 1, &roboteq_serial::rosMsgCallback, this);
    }
    else
    {
        ROS_ERROR("%s:: Could not find topic subscription name", pluginName.c_str());
    }

    if(ros::param::get(pluginName+"/publishToTopic", tempString))
    {
        rosDataPub = nh->advertise<messages::encoder_data>(tempString, 1, false);
    }
    else
    {
        ROS_ERROR("%s:: Could not find topic advertisment name", pluginName.c_str());
    }

    //need to start async timers here for grabber monitoring
    return true;
}

void hw_interface_plugin_roboteq::roboteq_serial::setInterfaceOptions()
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

bool hw_interface_plugin_roboteq::roboteq_serial::interfaceReadHandler(const long &length,
                                                                            int arrayStartPos)
{
    ROS_INFO_EXTRA_SINGLE("Roboteq Plugin Data Handler");

    ROS_DEBUG("\r\nContents: %s\r\n", receivedRegexData.c_str());

    typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
    boost::char_separator<char> sep("= :\r\n");
    tokenizer tokens(receivedRegexData, sep);

    try
    {
      tokenizer::iterator tok_iter = tokens.begin();

      if(tok_iter != tokens.end())
      {

        while ((tok_iter->c_str()[0]) == '+' && tok_iter != tokens.end())
        {
          ROS_INFO("%s", tok_iter->c_str());
          ++tok_iter;
        }
        
        ROS_INFO("%s",tok_iter->c_str()); 
        m_command = tok_iter->c_str();
        ++tok_iter;
      }

      if(tok_iter != tokens.end())
      {
          ROS_INFO("%s",tok_iter->c_str());
          m_commandVal1 = tok_iter->c_str();
          ++tok_iter;
      }

      if(tok_iter != tokens.end())
      {
          ROS_INFO("%s",tok_iter->c_str());
          m_commandVal2 = tok_iter->c_str();
          ++tok_iter;
      }
      
      if(!implDataHandler())
      {
        ROS_ERROR("%s :: Implementation Data Handler returned a BAD Return", pluginName.c_str());
      }
    }
    catch (const boost::bad_lexical_cast& e ){
      ROS_ERROR("%s:: Caught bad lexical cast with error %s", pluginName.c_str(), e.what());
    }
    catch(...){
      ROS_ERROR("%s:: Caught Unknown Error while parsing packet in data handler", pluginName.c_str());
    }

    
    return true;
}

bool hw_interface_plugin_roboteq::roboteq_serial::verifyChecksum()
{
    return true;
}

typedef boost::asio::buffers_iterator<boost::asio::streambuf::const_buffers_type> matcherIterator;

std::pair<matcherIterator, bool>
hw_interface_plugin_roboteq::roboteq_serial::matchFooter(matcherIterator begin, matcherIterator end, const char* sequence)
{
    int i = 0;
    for(matcherIterator footerIt = (end-std::strlen(sequence)); footerIt!=end; footerIt++)
    {
        if(*footerIt!=sequence[i])
        {
            return std::make_pair(begin, false);
        }
        i++;
    }
    return std::make_pair(begin, true);
}
