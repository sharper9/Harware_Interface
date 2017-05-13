#include <hw_interface_plugin_roboteq/hw_interface_plugin_roboteq_brushed.hpp>

hw_interface_plugin_roboteq::brushed::brushed()
{
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info))
  {
    ros::console::notifyLoggerLevelsChanged();
  }
}

bool hw_interface_plugin_roboteq::brushed::implStart()
{
  ROS_INFO("%s:: Plugin start!", pluginName.c_str());

  /*
   * Roboteq usage
   * 1. Disable Command Echo
   * 2. send '# C' to clear history buffer
   * 3. send '?AIC' read analog sensor after conversion
   * 4. send '# 20' to have runtime queries repeated at 20 ms delta (50 hz)
   */
  std::string initializationCmd = "";
  int initCmdCycle = 0;
  if(ros::param::get(pluginName+"/initializationCmd", initializationCmd))
  {
    if (!ros::param::get(pluginName+"/initCmdCycle", initCmdCycle))
    {
      initCmdCycle = 20;
    }
    initializationCmd = hw_interface_plugin_roboteq::roboteq_serial::getInitCommands(initializationCmd, initCmdCycle);
  }
  else
  {
    ROS_WARN("Roboteq Initialization Command Unspecified, defaulting");
    initializationCmd = "\r^ECHOF 1\r# C\r?AIC\r# 20\r";
  }
  ROS_INFO("Roboteq Init Cmd %s", initializationCmd.c_str());
  postInterfaceWriteRequest(hw_interface_support_types::shared_const_buffer(initializationCmd));

  return true;
}

bool hw_interface_plugin_roboteq::brushed::implStop()
{
    ROS_INFO("%s:: Plugin stop!", pluginName.c_str());
    return true;
}

bool hw_interface_plugin_roboteq::brushed::implDataHandler()
{
    ROS_DEBUG("%s :: Roboteq Brushed Implementation Data Handler", pluginName.c_str());
    //should check size of buffer is equal to size of msg, just in case.

    return true;
}
