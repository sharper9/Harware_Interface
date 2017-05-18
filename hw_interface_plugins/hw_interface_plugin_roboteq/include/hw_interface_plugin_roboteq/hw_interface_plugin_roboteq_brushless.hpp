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
      {"Reg_Left", 
          "# C\r"
          "^DINA 6 0\r"
          "^THLD 2\r"
          "^BLFB 1 1\r"
          "^BLSTD 1 0\r"
          "^BLL 1 -65535\r"
          "^BHL 1 65535\r"
          "^MXRPM 1 6500\r"
          "^MAC 1 60000\r"
          "^MDEC 1 100000\r"
          "^MMOD 1 1\r"
          "^MVEL 1 6500\r"
          "^MXTRN 1 1092250\r"
          "^KP 1 1\r"
          "^KI 1 3\r"
          "^KD 1 5\r"
          "^CLERD 1 0\r"
          "^BLFB 2 1\r"
          "^BLSTD 2 0\r"
          "^BLL 2 -65535\r"
          "^BHL 2 65535\r"
          "^MXRPM 2 6500\r"
          "^MAC 2 60000\r"
          "^MDEC 2 100000\r"
          "^MMOD 2 1\r"
          "^MVEL 2 6500\r"
          "^MXTRN 2 1092250\r"
          "^KP 2 1\r"
          "^KI 2 2\r"
          "^KD 2 4\r"
          "^CLERD 2 0\r"
          "%eesav\r"
          "# C\r"
          "#\r"},
      {"Reg_Right", 
          "# C\r"
          "^DINA 6 0\r"
          "^THLD 2\r"
          "^BLFB 1 1\r"
          "^BLSTD 1 0\r"
          "^BLL 1 -65535\r"
          "^BHL 1 65535\r"
          "^MXRPM 1 6500\r"
          "^MAC 1 60000\r"
          "^MDEC 1 100000\r"
          "^MMOD 1 1\r"
          "^MVEL 1 6500\r"
          "^MXTRN 1 1092250\r"
          "^KP 1 1\r"
          "^KI 1 3\r"
          "^KD 1 5\r"
          "^CLERD 1 0\r"
          "^BLFB 2 1\r"
          "^BLSTD 2 0\r"
          "^BLL 2 -65535\r"
          "^BHL 2 65535\r"
          "^MXRPM 2 6500\r"
          "^MAC 2 60000\r"
          "^MDEC 2 100000\r"
          "^MMOD 2 1\r"
          "^MVEL 2 6500\r"
          "^MXTRN 2 1092250\r"
          "^KP 2 1\r"
          "^KI 2 2\r"
          "^KD 2 4\r"
          "^CLERD 2 0\r"
          "%eesav\r"
          "# C\r"
          "#\r"},
      {"Comp_Left", 
          "# C\r"
          "^DINA 6 0\r"
          "^THLD 2\r"
          "^BLFB 1 1\r"
          "^BLSTD 1 3\r"
          "^BLL 1 -65535\r"
          "^BHL 1 65535\r"
          "^MXRPM 1 6500\r"
          "^MAC 1 60000\r"
          "^MDEC 1 100000\r"
          "^MMOD 1 1\r"
          "^MVEL 1 6500\r"
          "^MXTRN 1 1092250\r"
          "^KP 1 1\r"
          "^KI 1 3\r"
          "^KD 1 1\r"
          "^CLERD 1 0\r"
          "^BLFB 2 1\r"
          "^BLSTD 2 3\r"
          "^BLL 2 -65535\r"
          "^BHL 2 65535\r"
          "^MXRPM 2 6500\r"
          "^MAC 2 60000\r"
          "^MDEC 2 100000\r"
          "^MMOD 2 1\r"
          "^MVEL 2 6500\r"
          "^MXTRN 2 1092250\r"
          "^KP 2 1\r"
          "^KI 2 2\r"
          "^KD 2 2\r"
          "^CLERD 2 0\r"
          "%eesav\r"
          "# C\r"
          "#\r"},
      {"Comp_Right", 
          "# C\r"
          "^DINA 6 0\r"
          "^THLD 2\r"
          "^BLFB 1 1\r"
          "^BLSTD 1 3\r"
          "^BLL 1 -65535\r"
          "^BHL 1 65535\r"
          "^MXRPM 1 6500\r"
          "^MAC 1 60000\r"
          "^MDEC 1 100000\r"
          "^MMOD 1 1\r"
          "^MVEL 1 6500\r"
          "^MXTRN 1 1092250\r"
          "^KP 1 1\r"
          "^KI 1 3\r"
          "^KD 1 1\r"
          "^CLERD 1 0\r"
          "^BLFB 2 1\r"
          "^BLSTD 2 3\r"
          "^BLL 2 -65535\r"
          "^BHL 2 65535\r"
          "^MXRPM 2 6500\r"
          "^MAC 2 60000\r"
          "^MDEC 2 100000\r"
          "^MMOD 2 1\r"
          "^MVEL 2 6500\r"
          "^MXTRN 2 1092250\r"
          "^KP 2 1\r"
          "^KI 2 2\r"
          "^KD 2 2\r"
          "^CLERD 2 0\r"
          "%eesav\r"
          "# C\r"
          "#\r"}
    };

  };
}

PLUGINLIB_EXPORT_CLASS(hw_interface_plugin_roboteq::brushless, base_classes::base_interface)

#endif //HW_INTERFACE_PLUGIN_ROBOTEQ_BRUSHLESS_HPP__
