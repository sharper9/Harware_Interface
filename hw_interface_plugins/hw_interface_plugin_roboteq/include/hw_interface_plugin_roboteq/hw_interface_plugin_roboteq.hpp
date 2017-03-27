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
#include <map>

#include <messages/ActuatorOut.h>
#include <messages/encoder_data.h>
#include <messages/GrabberFeedback.h>

#include <hw_interface_plugin_roboteq/Roboteq_Data.h>
#include <hw_interface_plugin_roboteq/Analog_Input_Conversion_Info.h>

namespace hw_interface_plugin_roboteq {

    enum controller_t { Other, Left_Drive_Roboteq, Right_Drive_Roboteq, Bucket_Roboteq, Arm_Roboteq, Scoop_Roboteq };

   class roboteq_serial : public base_classes::base_serial_interface
   {
    public:
        typedef boost::asio::buffers_iterator<boost::asio::streambuf::const_buffers_type> matcherIterator;

        roboteq_serial();
        virtual ~roboteq_serial() {} //need to implement closing of the port here

    protected:
        ros::NodeHandlePtr nh;

        typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
        std::string getInitCommands(std::string initializationCmd, int initCmdCycle);

        messages::ActuatorOut latestActuatorCmd;
        controller_t roboteqType;

        bool subPluginInit(ros::NodeHandlePtr nhPtr);
        void setInterfaceOptions();
        bool interfaceReadHandler(const size_t &length, int arrayStartPost);
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

        hw_interface_plugin_roboteq::Roboteq_Data roboteqData;

        virtual bool implStart() = 0;
        virtual bool implStop() = 0;
        virtual bool implDataHandler() = 0;

        std::map <std::string, std::string> command_list = {
          {"motor_amps", "A"},
          {"analog_inputs", "AI"},
          {"analog_inputs_conversion", "AIC"},
          {"bl_motor_speed_rpm", "BS"},
          {"individual_digital_inputs", "DI"},
          {"destination_reached", "DR"},
          {"fault_flags", "FF"},
          {"absolute_brushless_counter", "CB"},
          {"brushless_count_relative", "BCR"},
          {"battery_amps", "BA"}
        };

        std::pair<matcherIterator, bool> matchFooter(matcherIterator begin, matcherIterator end, const char *sequence);

      private:
        int m_numInitCmds; // # of commands from launch file
        int m_numCmdsMatched; // # of commands matched regex from roboteqs
        bool dataHandler(tokenizer::iterator tok_iter, tokenizer tokens);

   };
}

#endif
