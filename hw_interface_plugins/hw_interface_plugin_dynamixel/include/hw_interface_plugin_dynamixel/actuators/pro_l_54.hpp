#ifndef pro_l_54_HPP__
#define pro_l_54_HPP__

#include <boost/asio.hpp>

#include <hw_interface_plugin_dynamixel/dynamixel_forwards.hpp>
#include <hw_interface_plugin_dynamixel/dynamixel_servo.hpp>

/*
 *
 * use #defines to macro the class instantiation
 *
 * needs initilize methods. handshake with manager for which regs need to be read for initialization
 *
 * need ros interfaces. any default messages and services
 */
 
#define L_54_MODEL_NUMBER_INIT_OFFSET 0
#define L_54_FIRMWARE_REVISION_INIT_OFFSET 6
#define L_54_ID_INIT_OFFSET 7

namespace hw_interface_plugin_dynamixel
{
    namespace dynamixel_actuators
    {
        class pro_l_54 : public dynamixel_servo
        {
            public:

                /*
                 * use the header to derive protocol version
                 */
                pro_l_54(dynamixel_types::dynamixel_id_number_t id,
                        std::vector<uint8_t> parameters,
                      dynamixel_types::comm_version_t commVersion=dynamixel_types::COMM_V2);

                ~pro_l_54(){}

            protected:

                dynamixel_types::dynamixel_ability_result_t _implAbilityRequest(dynamixel_types::dynamixel_ability_request_t &request);
                dynamixel_types::dynamixel_ability_info_t * _implAbilityInfoGet(dynamixel_types::dynamixel_ability_request_t &requestedAbility);


        };
    }
}



#endif //pro_l_54_HPP__
