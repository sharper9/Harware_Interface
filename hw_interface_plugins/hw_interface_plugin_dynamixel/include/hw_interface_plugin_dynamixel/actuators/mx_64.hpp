#ifndef MX_64_HPP__
#define MX_64_HPP__

#include <boost/asio.hpp>

#include <hw_interface_plugin_dynamixel/dynamixel_forwards.hpp>
#include <hw_interface_plugin_dynamixel/dynamixel_servo.hpp>
#include <hw_interface_plugin_dynamixel/actuators/mx_series.hpp>

/*
 *
 * use #defines to macro the class instantiation
 *
 * needs initilize methods. handshake with manager for which regs need to be read for initialization
 *
 * need ros interfaces. any default messages and services
 */

#define MODEL_NUMBER_INIT_OFFSET 0
#define FIRMWARE_REVISION_INIT_OFFSET 2
#define ID_INIT_OFFSET 3 

namespace hw_interface_plugin_dynamixel
{
    namespace dynamixel_actuators
    {
        class mx_64 : public dynamixel_servo
        {
            public:

                /*
                 * use the header to derive protocol version
                 */
                mx_64(std::vector<uint8_t> parameters,
                      dynamixel_types::comm_version_t commVersion=dynamixel_types::COMM_V1);

                ~mx_64(){}

            protected:

                dynamixel_types::dynamixel_ability_result_t _implAbilityRequest(dynamixel_types::dynamixel_ability_request_t &request);
                dynamixel_types::dynamixel_ability_info_t * _implAbilityInfoGet(dynamixel_types::dynamixel_ability_request_t &requestedAbility);


        };
    }
}



#endif //MX_64_HPP__
