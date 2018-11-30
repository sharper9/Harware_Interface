#ifndef DYNAMIXEL_FACTORY_HPP__
#define DYNAMIXEL_FACTORY_HPP__

#include <boost/tuple/tuple.hpp>
#include <boost/shared_ptr.hpp>

#include <hw_interface_plugin_dynamixel/dynamixel_forwards.hpp>
#include <hw_interface_plugin_dynamixel/dynamixel_servo.hpp>


/* BEGIN SERVO HEADERS */
#include <hw_interface_plugin_dynamixel/actuators/mx_28.hpp>
#include <hw_interface_plugin_dynamixel/actuators/mx_64.hpp>
#include <hw_interface_plugin_dynamixel/actuators/pro_l_54.hpp>

namespace hw_interface_plugin_dynamixel
{
    namespace dynamixel_factory
    {

        boost::tuple<hw_interface_plugin_dynamixel::dynamixel_types::dynamixel_actuator_series_t,
                                hw_interface_plugin_dynamixel::dynamixel_types::dynamixel_actuator_version_t>
                                    checkModelNumCompatibility(dynamixel_types::dynamixel_model_number_t modelNumToCheck,
                                                                    const std::vector<uint8_t> &servoResponse);
        
        boost::shared_ptr<dynamixel_servo> createServo(dynamixel_types::dynamixel_id_number_t idNum, const std::vector<uint8_t> &servoResponse, dynamixel_types::comm_version_t commVersion);
        boost::shared_ptr<dynamixel_servo> createServo(const dynamixel_types::comm_v1_status_packet_t pkt, dynamixel_types::comm_version_t commVersion)
        {
            return createServo(pkt.id, pkt.parameters, commVersion);
        }

        boost::shared_ptr<dynamixel_servo> createServo(const dynamixel_types::comm_v2_status_packet_t pkt, dynamixel_types::comm_version_t commVersion)
        {
            return createServo(pkt.id, pkt.parameters, commVersion);
        }

    }
}

 

#endif //DYNAMIXEL_FACTORY_HPP__
