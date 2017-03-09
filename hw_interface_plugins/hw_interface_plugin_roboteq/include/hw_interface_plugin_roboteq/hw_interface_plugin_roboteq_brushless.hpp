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

  };
}

PLUGINLIB_EXPORT_CLASS(hw_interface_plugin_roboteq::brushless, base_classes::base_interface)

#endif //HW_INTERFACE_PLUGIN_ROBOTEQ_BRUSHLESS_HPP__
