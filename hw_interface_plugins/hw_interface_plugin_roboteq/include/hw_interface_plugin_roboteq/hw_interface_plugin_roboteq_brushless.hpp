#ifndef HW_INTERFACE_PLUGIN_ROBOTEQ_BRUSHLESS_HPP__
#define HW_INTERFACE_PLUGIN_ROBOTEQ_BRUSHLESS_HPP__

#include <hw_interface_plugin_roboteq/hw_interface_plugin_roboteq.hpp>

namespace hw_interface_plugin_roboteq
{
  class brushless : public hw_interface_plugin_roboteq::roboteq_serial
  {
  public:
    brushless();

  protected:
    bool implStart();
    bool implStop();
    bool implDataHandler();

  private:
    std::map <std::string, std::string> script_list = {
      {"Reg_Left", "../rmc_ws/src/hw_interface_plugins/hw_interface_plugin_roboteq/REG_L_Drive_CL_Script.txt"},
      {"Reg_Right", "../rmc_ws/src/hw_interface_plugins/hw_interface_plugin_roboteq/REG_R_Drive_CL_Script.txt"},
      {"Comp_Left", "../rmc_ws/src/hw_interface_plugins/hw_interface_plugin_roboteq/COMP_L_Drive_CL_Script.txt"},
      {"Comp_Right", "../rmc_ws/src/hw_interface_plugins/hw_interface_plugin_roboteq/COMP_R_Drive_CL_Script.txt"}
    };

  };
}

PLUGINLIB_EXPORT_CLASS(hw_interface_plugin_roboteq::brushless, base_classes::base_interface)

#endif //HW_INTERFACE_PLUGIN_ROBOTEQ_BRUSHLESS_HPP__
