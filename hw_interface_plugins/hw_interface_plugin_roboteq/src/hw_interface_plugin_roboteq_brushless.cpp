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

  std::string filename;

  if (roboteqType == controller_t::Left_Drive_Roboteq)
    filename = "../rmc_ws/src/hw_interface_plugins/hw_interface_plugin_roboteq/left-drive-closed-loop-script.txt";
  else
    filename = "../rmc_ws/src/hw_interface_plugins/hw_interface_plugin_roboteq/right-drive-closed-loop-script.txt";

  std::ifstream roboteqInitFile (filename, std::ifstream::in);
  if (!roboteqInitFile)
  {
    ROS_ERROR("Error locating RoboteQ startup script file");
  }
  else
  {
    //TODO: if roboteq disconnects do this again
    roboteqInit.assign( (std::istreambuf_iterator<char>(roboteqInitFile)),
                        (std::istreambuf_iterator<char>()) );
    postInterfaceWriteRequest(hw_interface_support_types::shared_const_buffer(roboteqInit));
  }


  /*
   * Roboteq usage
   * 1. send '# C' to clear history buffer
   * 2. send '?CB' to get absolute brushless encoder count
   * 3. send '# 20' to have runtime queries repeated at 20 ms delta (50 hz)
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
