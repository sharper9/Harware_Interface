#ifndef HW_INTERFACE_PLUGIN_NB_HPP__
#define HW_INTERFACE_PLUGIN_NB_HPP__

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>

#include <hw_interface/base_interface.hpp>
#include <hw_interface/base_UDP_interface.hpp>

#include <messages/nb1_to_i7_msg.h>



namespace hw_interface_plugin_netburner {

    typedef struct s_nb_to_i7 nbDataMsg_t;
    struct s_nb_to_i7
    {
        uint16_t header;
        uint8_t  pktID;
        uint16_t counter;
        uint32_t clock_reg_count;
        uint32_t clock_reg_reset_count;
        
        int32_t  accX1;
        int32_t  accY1;
        int32_t  accZ1;
        int32_t  rateP1;
        int32_t  rateQ1;
        int32_t  rateR1;
        
        uint8_t  imuStatus;
        uint8_t  pauseSwitch;
        uint16_t mainLoopCounter;
        uint8_t  checksum;
    }__attribute__((packed));


    class netburner_UDP : public base_classes::base_UDP_interface
    {
    public:
        netburner_UDP();
        virtual ~netburner_UDP() {}

    protected:
        bool subPluginInit(ros::NodeHandlePtr nhPtr);
        bool interfaceReadHandler(const size_t &bufferSize, int arrayStartPos);
        bool verifyChecksum();
        
        ros::Publisher nbDataPub;
        
        messages::nb1_to_i7_msg nbROSMsg;


    };

}


PLUGINLIB_EXPORT_CLASS(hw_interface_plugin_netburner::netburner_UDP, base_classes::base_interface)



#endif //HW_INTERFACE_PLUGIN_NB_HPP__
