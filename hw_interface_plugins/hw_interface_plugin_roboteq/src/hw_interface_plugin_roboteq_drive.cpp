#include <hw_interface_plugin_roboteq/hw_interface_plugin_roboteq_drive.hpp>

// void hw_interface_plugin_roboteq::roboteq_drive::rosMsgCallback(const messages::ActuatorOut::ConstPtr &msgIn)
// {
//     std::string motorSpeedCmds = "";
//
//     if(roboteqType == controller_t::Right_Drive_Roboteq)
//     {
//         motorSpeedCmds += "!G 1 " + boost::lexical_cast<std::string>(msgIn->fr_speed_cmd) + "\r";
//         motorSpeedCmds += "!G 2 " + boost::lexical_cast<std::string>(msgIn->mr_speed_cmd) + "\r";
//         postInterfaceWriteRequest(hw_interface_support_types::shared_const_buffer(motorSpeedCmds));
//     }
//     else if(roboteqType == controller_t::Left_Drive_Roboteq)
//     {
//         motorSpeedCmds += "!G 1 " + boost::lexical_cast<std::string>(msgIn->fl_speed_cmd) + "\r";
//         motorSpeedCmds += "!G 2 " + boost::lexical_cast<std::string>(msgIn->ml_speed_cmd) + "\r";
//         postInterfaceWriteRequest(hw_interface_support_types::shared_const_buffer(motorSpeedCmds));
//     }
//     else
//     {
//         ROS_WARN("%s:: No Data written because of Incorrect Roboteq Type", pluginName.c_str());
//     }
//
//     //need to add monitoring facilities to monitor health
// }

// bool hw_interface_plugin_roboteq::roboteq_drive::implInit()
// {
//     std::string tempString;
//     if(ros::param::get(pluginName+"/subscribeToTopic", tempString))
//     {
//         rosDataSub = nh->subscribe(tempString, 1, &roboteq_drive::rosMsgCallback, this);
//     }
//     else
//     {
//         ROS_ERROR("%s:: Could not find topic subscription name", pluginName.c_str());
//     }
//
//     if(ros::param::get(pluginName+"/publishToTopic", tempString))
//     {
//         rosDataPub = nh->advertise<messages::encoder_data>(tempString, 1, false);
//     }
//     else
//     {
//         ROS_ERROR("%s:: Could not find topic advertisment name", pluginName.c_str());
//     }
//
//     //need to start async timers here for grabber monitoring
//     return true;
// }


//this alerts the implementation the plugin is about to start, at this point the interface is setup and data can begin to flow
bool hw_interface_plugin_roboteq::roboteq_drive::implStart()
{
    ROS_INFO("%s:: Plugin start!", pluginName.c_str());

    /*
     * Roboteq usage
     * 1. send '# C' to clear history buffer
     * 2. send '?CB' to get absolute brushless encoder count
     * 3. send '# 20' to have runtime queries repeated at 20 ms delta (50 hz)
     */

    std::string initializationCmd = "";
    if(!(ros::param::get(pluginName+"/initializationCmd", initializationCmd)))
    {
        ROS_WARN("Roboteq Initialization Command Unspecified, defaulting");
        initializationCmd = "\r# C\r?CB\r# 20\r";
    }
    ROS_INFO("Roboteq Init Cmd %s", initializationCmd.c_str());
    postInterfaceWriteRequest(hw_interface_support_types::shared_const_buffer(initializationCmd));

    return true;
}

bool hw_interface_plugin_roboteq::roboteq_drive::implStop()
{
    ROS_INFO("%s:: Plugin stop!", pluginName.c_str());
    return true;
}

bool hw_interface_plugin_roboteq::roboteq_drive::implDataHandler(const long &length, int arrayStartPos)
{
    ROS_DEBUG("%s :: Roboteq Drive Implementation Data Handler", pluginName.c_str());
    //should check size of buffer is equal to size of msg, just in case.

    messages::encoder_data encoderData;
    encoderData.motor_1_encoder_count = boost::lexical_cast<int32_t>(m_commandVal1);
    encoderData.motor_2_encoder_count = boost::lexical_cast<int32_t>(m_commandVal2);
    rosDataPub.publish(encoderData);

    return true;
}
