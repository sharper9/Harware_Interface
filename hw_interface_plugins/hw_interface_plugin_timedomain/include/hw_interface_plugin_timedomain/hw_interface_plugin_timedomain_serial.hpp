#ifndef HW_INTERFACE_PLUGIN_TIMEDOMAIN_HPP__
#define HW_INTERFACE_PLUGIN_TIMEDOMAIN_HPP__

//always inlclude these
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <hw_interface/base_interface.hpp>

//include the header of the base type you want, Serial or UDP
#include <hw_interface/base_serial_interface.hpp>

#include <hw_interface_plugin_timedomain/ranging_radio_types.h>

#include <hw_interface_plugin_timedomain/Range_Request.h>
#include <hw_interface_plugin_timedomain/RCM_Range_Info.h>



namespace hw_interface_plugin_timedomain {

    class timedomain_serial : public base_classes::base_serial_interface
    {
    public:
        timedomain_serial();
        virtual ~timedomain_serial() {}

    protected:

        //these methods are abstract as defined by the base_Serial_interface
            //they must be defined
        bool subPluginInit(ros::NodeHandlePtr nhPtr);
        void setInterfaceOptions();
        bool interfaceReadHandler(const size_t &length, int arrayStartPos);
        bool verifyChecksum();

        void rosMsgCallback(const hw_interface_plugin_timedomain::Range_Request::ConstPtr &msg);

        bool isRequestInProgress(){ return requestInProgress; }

    private:

        bool requestInProgress;
    };

}

//put the fully qualified type including its namespace here
PLUGINLIB_EXPORT_CLASS(hw_interface_plugin_timedomain::timedomain_serial, base_classes::base_interface)



#endif //HW_INTERFACE_PLUGIN_TIMEDOMAIN_HPP__
