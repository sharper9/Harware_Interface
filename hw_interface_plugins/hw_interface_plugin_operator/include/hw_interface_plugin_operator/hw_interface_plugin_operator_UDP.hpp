#ifndef HW_INTERFACE_PLUGIN_OPERATOR_HPP__
#define HW_INTERFACE_PLUGIN_OPERATOR_HPP__

#include <ros/ros.h>

#include <pluginlib/class_list_macros.h>

#include <boost/asio.hpp>
#include <topic_tools/shape_shifter.h>

#include <hw_interface/base_interface.hpp>
#include <hw_interface/base_UDP_interface.hpp>

#include <sensor_msgs/Joy.h>
#include <image_transport/camera_common.h>
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

namespace hw_interface_plugin_operator {


    class const_shared_buf_operator : public hw_interface_support_types::shared_const_buffer
    {
        public :
            const_shared_buf_operator(const topic_tools::ShapeShifter &msg, uint8_t type)
            {
                init(msg, ros::Time::now().toSec(), type);
            }
            const_shared_buf_operator(uint8_t type)
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
                ROS_DEBUG("msg size: %u", msg.size());
                ROS_DEBUG("Buffer Size %lu", data_->size());
                buffer_.reset(new boost::asio::const_buffer(boost::asio::buffer(*data_)));
            }
            void init(double timeSec, uint8_t type)
            {
                //ROS_DEBUG("counter %u :: type %u", counter, type);
                data_.reset(new std::vector<char>());
                data_->insert(data_->begin(), ((char*)&timeSec), ((char*)&timeSec)+sizeof(timeSec));
                data_->insert(data_->begin()+sizeof(timeSec), &type, &type+1);
                ROS_DEBUG("Pkt Start Time %2.9f", timeSec);
                ROS_DEBUG("Pkt hex time %lx", *((uint64_t*) &timeSec));
                ROS_DEBUG("Buffer Size %lu", data_->size());
                buffer_.reset(new boost::asio::const_buffer(boost::asio::buffer(*data_)));
            }
    };

    class operator_UDP : public base_classes::base_UDP_interface
    {
    public:
        operator_UDP();
        virtual ~operator_UDP() {}

    protected:
        bool subPluginInit(ros::NodeHandlePtr nhPtr);
        bool interfaceReadHandler(const size_t &bufferSize, int arrayStartPos, const boost::system::error_code &ec);
        bool verifyChecksum();

        bool pluginStart();

        int joyUpdateRate_ms;
        int heartbeatRate_ms;

        double lastMsgTimeStamp;
        double agentTimeOnLastMsg;
        double opTimeOnFirstReceipt;
        double agentTimeOnFirstReceipt;
        double agentOpTimeOffset;

        double instantLatency;

        bool agentConnected;



        void rosMsgCallback(const topic_tools::ShapeShifter::ConstPtr &msg, std::string &topicName);
        ros::Publisher rosPub;

        sensor_msgs::LaserScan lastScan;
        ros::Publisher scanPublisher;

        std::vector<ros::Subscriber> subscriberVector;

        bool gotOneJoy;
        topic_tools::ShapeShifter lastJoyMsg;


        void joyTimeoutHandler(const boost::system::error_code& error);
        void heartbeatTimeoutHandler(const boost::system::error_code& error);

        boost::mutex msgMutex;
        boost::shared_ptr<boost::asio::deadline_timer> joyUpdateTimer;
        boost::shared_ptr<boost::asio::deadline_timer> heartbeatUpdateTimer;

    };

}


PLUGINLIB_EXPORT_CLASS(hw_interface_plugin_operator::operator_UDP, base_classes::base_interface)



#endif //HW_INTERFACE_PLUGIN_OPERATOR_HPP__
