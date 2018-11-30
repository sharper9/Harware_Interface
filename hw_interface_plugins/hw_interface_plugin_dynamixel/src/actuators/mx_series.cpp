#include <hw_interface_plugin_dynamixel/actuators/mx_series.hpp>

hw_interface_plugin_dynamixel::dynamixel_actuators::mx_series::mx_series(std::vector<uint8_t> parameters):
    dynamixel_servo(
        *( (dynamixel_types::dynamixel_id_number_t*)&parameters.data()[ID_INIT_OFFSET]),
        *( (dynamixel_types::dynamixel_model_number_t*)&parameters.data()[MODEL_NUMBER_INIT_OFFSET]),
        *( (dynamixel_types::dynamixel_firmware_version_t*)&parameters.data()[FIRMWARE_REVISION_INIT_OFFSET])
        )
{

    packedAbilities|=    dynamixel_types::ABILITY_ANGLE_UNITS   |   dynamixel_types::ABILITY_SPEED_UNITS    |   dynamixel_types::ABILITY_RESOLUTION_DIVIDER
                        |dynamixel_types::ABILITY_WHEEL_MODE    |   dynamixel_types::ABILITY_JOINT_MODE     |   dynamixel_types::ABILITY_CCW_LIMIT
                        |dynamixel_types::ABILITY_CW_LIMIT;
    /*
     * if (version num)
     *  {select this protocol}
     */ 

}
