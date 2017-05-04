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
  
  std::string script;

  if (roboteqType == controller_t::Left_Drive_Roboteq)
  {
    //script = "# C\r^DINA 6 52\r^THLD 2\r^BLFB 1 1\r^BLSTD 1 3\r^BLL 1 -65535\r^BHL 1 65535\r^MXRPM 1 6500\r^MAC 1 30000\r^MDEC 1 50000\r^MMOD 1 1\r^MVEL 1 6500\r^MXTRN 1 1092250\r^KP 1 1\r^KI 1 3\r^KD 1 1\r^CLERD 1 0\r^BLFB 2 1\r^BLSTD 2 3\r^BLL 2 -65535\r^BHL 2 65535\r^MXRPM 2 6500\r^MAC 2 30000\r^MDEC 2 50000\r^MMOD 2 1\r^MVEL 2 6500\r^MXTRN 2 1092250\r^KP 2 1\r^KI 2 2\r^KD 2 2\r^CLERD 2 0\r%eesav \r# C \r# \r ";
    script = "# C\r \
^DINA 6 52\r \
^THLD 2\r \
^BLFB 1 1\r \
^BLSTD 1 3\r \
^BLL 1 -65535\r \
^BHL 1 65535\r \
^MXRPM 1 6500\r \
^MAC 1 30000\r \
^MDEC 1 50000\r \
^MMOD 1 1\r \
^MVEL 1 6500\r \
^MXTRN 1 1092250\r \
^KP 1 1\r \
^KI 1 3\r \
^KD 1 1\r \
^CLERD 1 0\r \
^BLFB 2 1\r \
^BLSTD 2 3\r \
^BLL 2 -65535\r \
^BHL 2 65535\r \
^MXRPM 2 6500\r \
^MAC 2 30000\r \
^MDEC 2 50000\r \
^MMOD 2 1\r \
^MVEL 2 6500\r \
^MXTRN 2 1092250\r \
^KP 2 1\r \
^KI 2 2\r \
^KD 2 2\r \
^CLERD 2 0\r \
%eesav \r \
# C \r \
# \r ";
  } 
  else 
  {
    //script = "# C\r^DINA 6 53\r^THLD 2\r^BLFB 1 1\r^BLSTD 1 3\r^BLL 1 -65535\r^BHL 1 65535\r^MXRPM 1 6500\r^MAC 1 30000\r^MDEC 1 50000\r^MMOD 1 1\r^MVEL 1 6500\r^MXTRN 1 1092250\r^KP 1 1\r^KI 1 3\r^KD 1 1\r^CLERD 1 0\r^BLFB 2 1\r^BLSTD 2 3\r^BLL 2 -65535\r^BHL 2 65535\r^MXRPM 2 6500\r^MAC 2 30000\r^MDEC 2 50000\r^MMOD 2 1\r^MVEL 2 6500\r^MXTRN 2 1092250\r^KP 2 1\r^KI 2 2\r^KD 2 2\r^CLERD 2 0\r%eesav \r# C \r# \r ";
    script = "# C\r \
^DINA 6 53\r \
^THLD 2\r \
^BLFB 1 1\r \
^BLSTD 1 3\r \
^BLL 1 -65535\r \
^BHL 1 65535\r \
^MXRPM 1 6500\r \
^MAC 1 30000\r \
^MDEC 1 50000\r \
^MMOD 1 1\r \
^MVEL 1 6500\r \
^MXTRN 1 1092250\r \
^KP 1 1\r \
^KI 1 3\r \
^KD 1 1\r \
^CLERD 1 0\r \
^BLFB 2 1\r \
^BLSTD 2 3\r \
^BLL 2 -65535\r \
^BHL 2 65535\r \
^MXRPM 2 6500\r \
^MAC 2 30000\r \
^MDEC 2 50000\r \
^MMOD 2 1\r \
^MVEL 2 6500\r \
^MXTRN 2 1092250\r \
^KP 2 1\r \
^KI 2 2\r \
^KD 2 2\r \
^CLERD 2 0\r \
%eesav \r \
# C \r \
# \r ";
  }
  ROS_INFO("script: %s",script.c_str());
  postInterfaceWriteRequest(hw_interface_support_types::shared_const_buffer(script));
  
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
