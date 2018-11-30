#include <hw_interface_plugin_dynamixel/dynamixel_factory.hpp>

/*
 *
 * swap these with boost tupless
 */

boost::tuple<hw_interface_plugin_dynamixel::dynamixel_types::dynamixel_actuator_series_t,
                hw_interface_plugin_dynamixel::dynamixel_types::dynamixel_actuator_version_t>
hw_interface_plugin_dynamixel::dynamixel_factory::checkModelNumCompatibility(dynamixel_types::dynamixel_model_number_t modelNumToCheck,
                                                                                const std::vector<uint8_t> &servoResponse)
{
    ROS_INFO("checkModel %u", modelNumToCheck);
    switch(modelNumToCheck)
    {
        case 0x0140: //mx-106
        case 0x0136: //MX-64
            return boost::make_tuple(dynamixel_types::MX, dynamixel_types::MX_64);
        case 0x001d: //MX_28
            return boost::make_tuple(dynamixel_types::MX, dynamixel_types::MX_28);
        case 0xD208://H54
        case 0xD308:
            return boost::make_tuple(dynamixel_types::PRO_H, dynamixel_types::PRO_H_54);
        case 0xC800://H42
            return boost::make_tuple(dynamixel_types::PRO_H, dynamixel_types::PRO_H_42);
        case 0xB510://M54
        case 0xB410:
            return boost::make_tuple(dynamixel_types::PRO_M, dynamixel_types::PRO_M_54);
        case 0xA918://M42
            return boost::make_tuple(dynamixel_types::PRO_M, dynamixel_types::PRO_M_42);
        case 0x9508: //l54
        case 0x9408:
        case 0x9520:
        case 0x9428:
            return boost::make_tuple(dynamixel_types::PRO_L, dynamixel_types::PRO_L_54);

        default:
            return boost::make_tuple(dynamixel_types::UNKNOWN_SERIES,
                                        dynamixel_types::UNKNOWN_ACTUATOR);
    }
} 


boost::shared_ptr<hw_interface_plugin_dynamixel::dynamixel_servo>
hw_interface_plugin_dynamixel::dynamixel_factory::createServo(dynamixel_types::dynamixel_id_number_t idNum,
                                                              const std::vector<uint8_t> &servoResponse,
                                                              dynamixel_types::comm_version_t commVersion)
{
    boost::tuple<dynamixel_types::dynamixel_actuator_series_t,
                    dynamixel_types::dynamixel_actuator_version_t> actuatorIdentification;

    actuatorIdentification = checkModelNumCompatibility(*((dynamixel_types::dynamixel_model_number_t*) &servoResponse.data()[0]),
                                            servoResponse);

    switch(boost::tuples::get<1>(actuatorIdentification))
    {
        case dynamixel_types::MX_64:
            ROS_INFO("Identified Servo as MX_64 based on initial EEPROM Dump");
            return boost::shared_ptr<dynamixel_servo>(new dynamixel_actuators::mx_64(servoResponse,commVersion));

        case dynamixel_types::MX_28:
            ROS_WARN("Identified Servo as MX_28 based on initial EEPROM Dump.");
            return boost::shared_ptr<dynamixel_servo>(new dynamixel_actuators::mx_28(servoResponse,commVersion));

        case dynamixel_types::MX_12:
            ROS_WARN("Identified Servo as MX_12 based on initial EEPROM Dump. Not Implemented");
            return boost::shared_ptr<dynamixel_servo>(0);

        case dynamixel_types::PRO_H_54:
            ROS_WARN("Identified Servo as PRO_H_54 based on initial EEPROM Dump. Not Implemented");
            return boost::shared_ptr<dynamixel_servo>(0);

        case dynamixel_types::PRO_H_42:
            ROS_WARN("Identified Servo as PRO_H_42 based on initial EEPROM Dump. Not Implemented");
            return boost::shared_ptr<dynamixel_servo>(0);

        case dynamixel_types::PRO_M_54:
            ROS_WARN("Identified Servo as PRO_M_54 based on initial EEPROM Dump. Not Implemented");
            return boost::shared_ptr<dynamixel_servo>(0);

        case dynamixel_types::PRO_M_42:
            ROS_WARN("Identified Servo as PRO_M_42 based on initial EEPROM Dump. Not Implemented");
            return boost::shared_ptr<dynamixel_servo>(0);

        case dynamixel_types::PRO_L_54:
            ROS_WARN("Identified Servo as PRO_L_54 based on initial EEPROM Dump.");
            return boost::shared_ptr<dynamixel_servo>(new dynamixel_actuators::pro_l_54(idNum, servoResponse, commVersion));

        default:
            ROS_WARN("Unidentified servo based on initial EEPROM Dump. Not Implemented");
            return boost::shared_ptr<dynamixel_servo>(0);
    }

    return 0;
}
