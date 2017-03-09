#ifndef HW_INTERFACE_PLUGIN_ROBOTEQ_BRUSHED_HPP__
#define HW_INTERFACE_PLUGIN_ROBOTEQ_BRUSHED_HPP__

#include <hw_interface_plugin_roboteq/hw_interface_plugin_roboteq.hpp>

namespace hw_interface_plugin_roboteq
{
  class brushed : public hw_interface_plugin_roboteq::roboteq_serial
  {
  public:
    brushed();

  protected:
    bool implStart();
    bool implStop();
    bool implDataHandler();

  };
}

PLUGINLIB_EXPORT_CLASS(hw_interface_plugin_roboteq::brushed, base_classes::base_interface)

#endif //HW_INTERFACE_PLUGIN_ROBOTEQ_BRUSHED_HPP__
