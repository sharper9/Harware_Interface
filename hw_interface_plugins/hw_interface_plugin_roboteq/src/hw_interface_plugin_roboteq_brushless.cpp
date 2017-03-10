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
  if(ros::param::get(pluginName+"/initializationCmd", initializationCmd))
  {
    if (!command_list[initializationCmd].length())
    {
      ROS_WARN("Roboteq initialization command was not found, defaulting...");
      initializationCmd = "\r^ECHOF 1\r# C\r?CB\r# 20\r";
    }

    initializationCmd = "\r^ECHOF 1\r# C\r?" + command_list[initializationCmd] + "\r# 20 \r";
  }
  else
  {
      ROS_WARN("Roboteq Initialization Command Unspecified, defaulting");
      initializationCmd = "\r^ECHOF 1\r# C\r?CB\r# 20\r";
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

    return true;
}
