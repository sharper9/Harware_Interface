#ifndef HW_INTERFACE_PLUGIN_ROBOTEQ_DRIVE_HPP__
#define HW_INTERFACE_PLUGIN_ROBOTEQ_DRIVE_HPP__

#include <hw_interface_plugin_roboteq/hw_interface_plugin_roboteq.hpp>

/*
 * Roboteq usage
 * 1. send '# C' to clear history buffer
 * 2. send '?CB' to get absolute brushless encoder count
 * 3. send '# 20' to have runtime queries repeated at 20 ms delta (50 hz)
 */

namespace hw_interface_plugin_roboteq {

    class roboteq_drive : public hw_interface_plugin_roboteq::roboteq_serial
    {
        public:
            roboteq_drive(){
                if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
                       ros::console::notifyLoggerLevelsChanged();
                    }
            }

            // void rosMsgCallback(const messages::ActuatorOut::ConstPtr &msgIn);

        protected:
            messages::encoder_data lastMsgPub;

            // bool implInit();
            bool implStart();
            bool implStop();
            bool implDataHandler(const long &length, int arrayStartPos);

            bool implChecksum();
    };

}

PLUGINLIB_EXPORT_CLASS(hw_interface_plugin_roboteq::roboteq_drive, base_classes::base_interface)

#endif //HW_INTERFACE_PLUGIN_ROBOTEQ_DRIVE_HPP__
