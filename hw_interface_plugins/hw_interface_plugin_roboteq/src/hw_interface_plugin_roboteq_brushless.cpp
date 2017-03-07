#include <hw_interface_plugin_roboteq/hw_interface_plugin_roboteq_brushless.hpp>

hw_interface_plugin_roboteq::brushless::brushless()
{
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
  {
    ros::console::notifyLoggerLevelsChanged();
  }
}

bool hw_interface_plugin_roboteq::brushless::implStart()
{
  ROS_INFO("%s:: Plugin start!", pluginName.c_str());

  /*
   * Roboteq usage
   * 1. send '# C' to clear history buffer
   * 2. send '?CB' to get absolute brushless encoder count
   * 3. send '# 20' to have runtime queries repeated at 20 ms delta (50 hz)
   */
  std::string initializationCmd = "";
  if(!(ros::param::get(pluginName+"/initializationCmd", initializationCmd)))
  {
      ROS_WARN("Roboteq Initialization Command Unspecified, defaulting");
      initializationCmd = "\r# C\r?CB\r# 20\r";
  }
  ROS_INFO("Roboteq Init Cmd %s", initializationCmd.c_str());
  postInterfaceWriteRequest(hw_interface_support_types::shared_const_buffer(initializationCmd));

  return true;
}

bool hw_interface_plugin_roboteq::brushless::implStop()
{
    ROS_INFO("%s:: Plugin stop!", pluginName.c_str());
    return true;
}

bool hw_interface_plugin_roboteq::brushless::implDataHandler()
{
    ROS_DEBUG("%s :: Roboteq Brushless Implementation Data Handler", pluginName.c_str());
    //should check size of buffer is equal to size of msg, just in case.

    messages::encoder_data encoderData;
    encoderData.motor_1_encoder_count = boost::lexical_cast<int32_t>(m_commandVal1);
    encoderData.motor_2_encoder_count = boost::lexical_cast<int32_t>(m_commandVal2);
    rosDataPub.publish(encoderData);

    m_commandVal1 = "";
    m_commandVal2 = "";
    return true;
}
