#ifndef HW_INTERFACE_PLUGIN_ROBOTEQ_HPP__
#define HW_INTERFACE_PLUGIN_ROBOTEQ_HPP__

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>

#include <hw_interface/shared_const_buffer.hpp>
#include <hw_interface/base_interface.hpp>
#include <hw_interface/base_serial_interface.hpp>

#include <boost/tokenizer.hpp>
#include <iterator>
#include <boost/regex.hpp>

#include <messages/ActuatorOut.h>
#include <messages/encoder_data.h>
#include <messages/GrabberFeedback.h>

#include <hw_interface_plugin_roboteq/Analog_Input_Conversion_Info.h>

namespace hw_interface_plugin_roboteq {

    enum controller_t {Other, Left_Drive_Roboteq, Right_Drive_Roboteq, Bucket_Roboteq};

   class roboteq_serial : public base_classes::base_serial_interface
   {
    public:
        typedef boost::asio::buffers_iterator<boost::asio::streambuf::const_buffers_type> matcherIterator;

        roboteq_serial();
        virtual ~roboteq_serial() {} //need to implement closing of the port here

    protected:
        ros::NodeHandlePtr nh;

        messages::ActuatorOut latestActuatorCmd;
        controller_t roboteqType;

        bool subPluginInit(ros::NodeHandlePtr nhPtr);
        void setInterfaceOptions();
        bool interfaceReadHandler(const long &length, int arrayStartPost);
        bool verifyChecksum();

        bool pluginStart()
        {
            return implStart();
        }

        bool pluginStop()
        {
            return implStop();
        }

        bool implInit();
        void rosMsgCallback(const messages::ActuatorOut::ConstPtr &msgIn);
        std::string m_command;
        std::string m_commandVal1;
        std::string m_commandVal2;

        virtual bool implStart() = 0;
        virtual bool implStop() = 0;
        virtual bool implDataHandler() = 0;

        std::pair<matcherIterator, bool> matchFooter(matcherIterator begin, matcherIterator end, const char *sequence);


   };
}

#endif
