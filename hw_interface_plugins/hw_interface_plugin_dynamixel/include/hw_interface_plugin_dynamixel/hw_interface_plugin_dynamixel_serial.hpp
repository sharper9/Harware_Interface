#ifndef HW_INTERFACE_PLUGIN_EXAMPLE_HPP__
#define HW_INTERFACE_PLUGIN_EXAMPLE_HPP__

//always inlclude these
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <hw_interface/base_interface.hpp>
#include <hw_interface/base_serial_interface.hpp>


#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>
#include <queue>


#include <hw_interface_plugin_dynamixel/dynamixel_forwards.hpp>
#include <hw_interface_plugin_dynamixel/dynamixel_servo.hpp>

#include <hw_interface_plugin_dynamixel/dynamixel_factory.hpp>

#include <hw_interface_plugin_dynamixel/servoUpdate.h>
#include <hw_interface_plugin_dynamixel/groupServoUpdate.h>


#include "std_msgs/String.h"

#define RESPONSE_TIMEOUT_MILLI 500

#define INIT_READ_SIZE 8


namespace hw_interface_plugin_dynamixel {


    enum interfaceState_t {DISCOVERY, EXECUTION, ARBITRATION_LOSS} ;


    class dynamixel_serial : public base_classes::base_serial_interface
    {
        public:

            dynamixel_serial();
            ~dynamixel_serial() {
                //may need destructors for held pointers
//                for(std::size_t i = 0; i < servoDirectory.size(); i++)
//                {

//                }
            }
 
            interfaceState_t getInterfaceState() { return currentState; }
            bool continueDiscovery = false;
            bool regReadSent = false;
            bool retry = false;
            int numServoOn = 1;
            int numInWheelMode = 0;
            dynamixel_types::comm_version_t commVDiscovery;

            size_t numQueryRead = 0;
            dynamixel_types::dynamixel_ability_mode_t lastMode = hw_interface_plugin_dynamixel::dynamixel_types::MODE_NOP;
            dynamixel_types::dynamixel_ability_request_t *lastSent = 0;
            dynamixel_types::comm_version_t lastComm;

        protected:

            //these methods are abstract as defined by the base_serial_interface
                //they must be defined
            bool subPluginInit(ros::NodeHandlePtr nhPtr);
            void setInterfaceOptions();
            bool interfaceReadHandler(const size_t &length, int arrayStartPos, const boost::system::error_code &ec);
            bool verifyChecksum();

            bool pluginStart();
            bool pluginStop();

            std::size_t dynamixelStreamMatcher(const boost::system::error_code &error, long totalBytesInBuffer,
                                               const char *header, int headerLength);


            void updateServoCallback(const hw_interface_plugin_dynamixel::servoUpdate::ConstPtr &msg);
            void groupUpdateServoCallback(const hw_interface_plugin_dynamixel::groupServoUpdate::ConstPtr &msg);

            void timeoutHandler(const boost::system::error_code& error);

            ros::Subscriber rosDataSubRequest;
            ros::Subscriber rosDataSubGroupRequest;

            boost::shared_ptr<boost::asio::deadline_timer> servoResponseTimer;

            dynamixel_types::comm_v1_packet_t testpkt;

            int scanMaxId;

        private:
            int responseTimeout;

            bool sentCommV2Ping;

            interfaceState_t currentState;

            std::vector<boost::shared_ptr<dynamixel_servo> > servoDirectory;

            boost::mutex requestQueueMtx;
            std::queue<dynamixel_types::dynamixel_ability_request_t> requestQueue;
            boost::mutex queryQueueMtx;
            std::queue<dynamixel_types::dynamixel_ability_request_t> queryQueue;

    };

}

//put the fully qualified type including its namespace here
//for more info: http://wiki.ros.org/pluginlib#pluginlib.2BAC8-pluginlib_groovy.Registering.2BAC8-Exporting_a_Plugin
PLUGINLIB_EXPORT_CLASS(hw_interface_plugin_dynamixel::dynamixel_serial, base_classes::base_interface)



#endif //HW_INTERFACE_PLUGIN_EXAMPLE_HPP__

