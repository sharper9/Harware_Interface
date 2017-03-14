#ifndef HW_INTERFACE_PLUGIN_EXAMPLE_HPP__
#define HW_INTERFACE_PLUGIN_EXAMPLE_HPP__

//always inlclude these
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <hw_interface/base_interface.hpp>

//include the header of the base type you want, Serial or UDP
#include <hw_interface/base_serial_interface.hpp>

#include <messages/ActuatorOut.h>

namespace hw_interface_plugin_example {

    class example_serial : public base_classes::base_serial_interface
    {
    public:
        example_serial();
        virtual ~example_serial() {}

    protected:

        //these methods are abstract as defined by the base_serial_interface
            //they must be defined
        bool subPluginInit(ros::NodeHandlePtr nhPtr);
        void setInterfaceOptions();
        bool interfaceReadHandler(const size_t &length, int arrayStartPos);
        bool verifyChecksum();

        void rosMsgCallback(const messages::ActuatorOut::ConstPtr &msgIn);
    };

}

//put the fully qualified type including its namespace here
//for more info: http://wiki.ros.org/pluginlib#pluginlib.2BAC8-pluginlib_groovy.Registering.2BAC8-Exporting_a_Plugin
PLUGINLIB_EXPORT_CLASS(hw_interface_plugin_example::example_serial, base_classes::base_interface)



#endif //HW_INTERFACE_PLUGIN_EXAMPLE_HPP__
