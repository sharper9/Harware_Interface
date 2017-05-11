#ifndef HW_INTERFACE_PLUGIN_AGENT_HPP__
#define HW_INTERFACE_PLUGIN_AGENT_HPP__

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>

#include <hw_interface/base_interface.hpp>
#include <hw_interface/base_UDP_interface.hpp>

#include <sensor_msgs/Joy.h>

#include <hw_interface_plugin_agent/LOS.h>
//#include <msgs_and_srvs/WebcamCommands.h>

#include <topic_tools/shape_shifter.h>

#include <sensor_msgs/LaserScan.h>

#include <image_transport/image_transport.h>

#define MSG_TIMESTAMP_OFFSET    0
#define MSG_TYPE_OFFSET         8 //1 after the double timestep which occupies [0-7], at byte index [8]
#define MSG_OFFSET              9 //1 after the type at byte [8], this indicates the start of the serialized ros msg if the pkt contains one

#define MSG_TYPE_JOY            1
#define MSG_TYPE_LASER_SCAN     2
#define MSG_TYPE_ARM_POSITION   3
#define MSG_TYPE_BUMPER_STATUS  4
#define MSG_TYPE_DRIVE_CMDS     5
#define MSG_TYPE_ARM_MODE       6
#define MSG_TYPE_BUMPER_PARAMS  7
#define MSG_TYPE_CAMERA_IMAGE   8
#define MSG_TYPE_HEARTBEAT      0xff


namespace hw_interface_plugin_agent {

    class const_shared_buf_agent : public hw_interface_support_types::shared_const_buffer
    {
        public :
            const_shared_buf_agent(const topic_tools::ShapeShifter &msg, uint8_t type)
            {
                init(msg, ros::Time::now().toSec(), type);
            }
            const_shared_buf_agent(uint8_t type)
            {
                init(ros::Time::now().toSec(), type);
            }

        private:
            void init(const topic_tools::ShapeShifter &msg, double timeSec, uint8_t type)
            {
                //ROS_DEBUG("counter %u :: type %u", counter, type);
                data_.reset(new std::vector<char>(ros::serialization::serializationLength(msg)+9));
                *((double*) &((*data_)[MSG_TIMESTAMP_OFFSET])) = timeSec;
                (*data_)[MSG_TYPE_OFFSET]=type;
                ros::serialization::IStream is((uint8_t*)(data_->data()+9), msg.size());
                msg.write(is);
                ROS_DEBUG("msg size: %u, %u", msg.size(), ros::serialization::serializationLength(msg));
                ROS_DEBUG("Buffer Size %lu", data_->size());
                buffer_.reset(new boost::asio::const_buffer(boost::asio::buffer(*data_)));
            }
            void init(double timeSec, uint8_t type)
            {
                //ROS_DEBUG("counter %u :: type %u", counter, type);
                data_.reset(new std::vector<char>());
                data_->insert(data_->begin(), &timeSec, &timeSec+sizeof(timeSec));
                data_->insert(data_->begin()+sizeof(timeSec), &type, &type+1);
                ROS_DEBUG("Buffer Size %lu", data_->size());
                buffer_.reset(new boost::asio::const_buffer(boost::asio::buffer(*data_)));
            }
    };

    class agent_UDP : public base_classes::base_UDP_interface
    {
    public:
        agent_UDP();
        virtual ~agent_UDP() {}

    protected:
        bool subPluginInit(ros::NodeHandlePtr nhPtr);
        bool interfaceReadHandler(const size_t &bufferSize, int arrayStartPos);
        bool verifyChecksum();

        bool pluginStart();

        void LOSTimeoutHandler(const boost::system::error_code& error);
        void msgCallback(const topic_tools::ShapeShifter::ConstPtr &msg, uint8_t type);

        //void webCamCmdCallback(const msgs_and_srvs::WebcamCommands::ConstPtr &msg);

        void compressedImageTransportCallback(const sensor_msgs::ImageConstPtr &msg);


        bool operatorConnected;
        double lastMsgTimeStamp;
        double agentTimeOnLastMsg;
        double opTimeOnFirstReceipt;
        double agentTimeOnFirstReceipt;
        double agentOpTimeOffset;

        double instantLatency;

        int expectedHeartBeatRate_ms;

        ros::Publisher LOSPub;
        ros::Publisher joyPublisher;
        ros::Publisher webCamCmdPub;

        ros::Subscriber webCamCmdSub;
        ros::Subscriber scanSub;
        ros::Subscriber bumperParamsSub;
        ros::Subscriber armPositionSub;
        ros::Subscriber bumperStatusSub;
        ros::Subscriber driveCommandsSub;
        ros::Subscriber armModeSub;
        ros::Subscriber cameraImageSub;

        image_transport::Subscriber imageTransportSub;

        sensor_msgs::Joy newJoyVal;
        //msgs_and_srvs::WebcamCommands webCamCmdFromCommand;


        boost::mutex LOSTimeMutex;
        boost::mutex webCamCmdMutex;
        ros::Time lastLOStime;

        boost::shared_ptr<boost::asio::deadline_timer> LOSUpdateTimer;
    };

}


PLUGINLIB_EXPORT_CLASS(hw_interface_plugin_agent::agent_UDP, base_classes::base_interface)



#endif //HW_INTERFACE_PLUGIN_AGENT_HPP__
