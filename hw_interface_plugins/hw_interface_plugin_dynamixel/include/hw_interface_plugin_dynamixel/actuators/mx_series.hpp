#ifndef mx_series_HPP__
#define mx_series_HPP__

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
 
#define MODEL_NUMBER_INIT_OFFSET 0
#define FIRMWARE_REVISION_INIT_OFFSET 2
#define ID_INIT_OFFSET 3

namespace hw_interface_plugin_dynamixel
{
    namespace dynamixel_actuators
    {
        class mx_series : public dynamixel_servo
        {
            public:
                virtual ~mx_series(){}
                mx_series(){}

            protected:

                /*
                 * use the header to derive protocol version
                 */
                mx_series(std::vector<uint8_t> parameters);

                virtual dynamixel_types::dynamixel_ability_result_t _implAbilityRequest(dynamixel_types::dynamixel_ability_request_t &request);
                virtual dynamixel_types::dynamixel_ability_info_t * _implAbilityInfoGet(dynamixel_types::dynamixel_ability_request_t &requestedAbility);


        };
    }
}



#endif //mx_series_HPP__
