#include <hw_interface_plugin_dynamixel/actuators/mx_64.hpp>


hw_interface_plugin_dynamixel::dynamixel_actuators::mx_64::mx_64(std::vector<uint8_t> parameters,
                                                                 dynamixel_types::comm_version_t commVersion):
    dynamixel_servo(
        *( (dynamixel_types::dynamixel_id_number_t*)&parameters.data()[ID_INIT_OFFSET]),
        *( (dynamixel_types::dynamixel_model_number_t*)&parameters.data()[MODEL_NUMBER_INIT_OFFSET]),
        *( (dynamixel_types::dynamixel_firmware_version_t*)&parameters.data()[FIRMWARE_REVISION_INIT_OFFSET])
        )
{ 
    /*
     * if (version num)
     *  {select this protocol}
     */
    packedAbilities|=    dynamixel_types::ABILITY_ANGLE_UNITS   |   dynamixel_types::ABILITY_SPEED_UNITS    |   dynamixel_types::ABILITY_RESOLUTION_DIVIDER
                        |dynamixel_types::ABILITY_WHEEL_MODE    |   dynamixel_types::ABILITY_JOINT_MODE     |   dynamixel_types::ABILITY_CCW_LIMIT
                        |dynamixel_types::ABILITY_CW_LIMIT      |   dynamixel_types::ABILITY_MOVING_SPEED   |   dynamixel_types::ABILITY_GOAL_POSITION
                        |dynamixel_types::ABILITY_PRESENT_POSITION| dynamixel_types::ABILITY_PRESENT_SPEED  | dynamixel_types::ABILITY_STATUS_RETURN_LEVEL;

    protocolVersion = commVersion;
    actuatorSeries = dynamixel_types::MX;
    actuatorVersion = dynamixel_types::MX_64;

    underLyingActuatorMemory.assign(0x49, 0);

    abilityInfoList.emplace_back(dynamixel_types::ABILITY_MOVING_SPEED,     dynamixel_types::MODE_GET | dynamixel_types::MODE_SET,
                                 32, dynamixel_types::UNIT_RAW,         dynamixel_types::UNSIGNED_WORD, 0, dynamixel_types::MODE_NOP);
    abilityInfoList.emplace_back(dynamixel_types::ABILITY_CCW_LIMIT,        dynamixel_types::MODE_GET | dynamixel_types::MODE_SET,
                                 0x08, dynamixel_types::UNIT_RAW,       dynamixel_types::UNSIGNED_WORD, 0, dynamixel_types::MODE_NOP);
    abilityInfoList.emplace_back(dynamixel_types::ABILITY_CW_LIMIT,         dynamixel_types::MODE_GET | dynamixel_types::MODE_SET,
                                 0x06, dynamixel_types::UNIT_RAW,       dynamixel_types::UNSIGNED_WORD, 0, dynamixel_types::MODE_NOP);
    abilityInfoList.emplace_back(dynamixel_types::ABILITY_WHEEL_MODE,       dynamixel_types::MODE_GET | dynamixel_types::MODE_SET,
                                 0x06, dynamixel_types::UNIT_RAW,       dynamixel_types::UNSIGNED_DWORD, 0, dynamixel_types::MODE_NOP);
    abilityInfoList.emplace_back(dynamixel_types::ABILITY_GOAL_POSITION,    dynamixel_types::MODE_GET | dynamixel_types::MODE_SET,
                                 0x1e, dynamixel_types::UNIT_RAW,       dynamixel_types::UNSIGNED_WORD, 0, dynamixel_types::MODE_NOP);

    abilityInfoList.emplace_back(dynamixel_types::ABILITY_JOINT_MODE, dynamixel_types::MODE_GET | dynamixel_types::MODE_SET,
                                 0x08, dynamixel_types::UNIT_RAW, dynamixel_types::UNSIGNED_WORD, 0, dynamixel_types::MODE_NOP);

    abilityInfoList.emplace_back(dynamixel_types::ABILITY_PRESENT_SPEED, dynamixel_types::MODE_QUERY | dynamixel_types::MODE_GET | dynamixel_types::MODE_SET,
                                 36, dynamixel_types::UNIT_RAW, dynamixel_types::UNSIGNED_WORD, 0, dynamixel_types::MODE_NOP);
    abilityInfoList.emplace_back(dynamixel_types::ABILITY_PRESENT_POSITION, dynamixel_types::MODE_QUERY | dynamixel_types::MODE_GET | dynamixel_types::MODE_SET,
                                 38, dynamixel_types::UNIT_RAW, dynamixel_types::UNSIGNED_WORD, 0, dynamixel_types::MODE_NOP);

    abilityInfoList.emplace_back(dynamixel_types::ABILITY_STATUS_RETURN_LEVEL, dynamixel_types::MODE_QUERY | dynamixel_types::MODE_GET | dynamixel_types::MODE_SET,
                                 0x10, dynamixel_types::UNIT_RAW, dynamixel_types::UNSIGNED_WORD, 0, dynamixel_types::MODE_NOP);



}


hw_interface_plugin_dynamixel::dynamixel_types::dynamixel_ability_result_t
    hw_interface_plugin_dynamixel::dynamixel_actuators::mx_64::_implAbilityRequest(dynamixel_types::dynamixel_ability_request_t &request)
{


}

hw_interface_plugin_dynamixel::dynamixel_types::dynamixel_ability_info_t *
    hw_interface_plugin_dynamixel::dynamixel_actuators::mx_64::_implAbilityInfoGet(dynamixel_types::dynamixel_ability_request_t &requestedAbility)
{
    if(requestedAbility.mode == dynamixel_types::MODE_GET)
    {
        switch(requestedAbility.abilityRequested)
        {
            case dynamixel_types::ABILITY_MOVING_SPEED:
                return &(abilityInfoList[0]);
            case dynamixel_types::ABILITY_CCW_LIMIT:
                return &(abilityInfoList[1]);
            case dynamixel_types::ABILITY_CW_LIMIT:
                return &(abilityInfoList[2]);
            case dynamixel_types::ABILITY_WHEEL_MODE:
                return &(abilityInfoList[3]);
            case dynamixel_types::ABILITY_GOAL_POSITION:
                return &(abilityInfoList[4]);
            case dynamixel_types::ABILITY_JOINT_MODE:
                return &(abilityInfoList[5]);
            case dynamixel_types::ABILITY_PRESENT_SPEED:
                return &(abilityInfoList[6]);
            case dynamixel_types::ABILITY_PRESENT_POSITION:
                return &(abilityInfoList[7]);
            case dynamixel_types::ABILITY_STATUS_RETURN_LEVEL:
                return &(abilityInfoList[8]);
            default:
                return 0;


        }
    }
    else if(requestedAbility.mode == dynamixel_types::MODE_SET)
    {
        switch(requestedAbility.abilityRequested)
        {
            case dynamixel_types::ABILITY_MOVING_SPEED:
                return &(abilityInfoList[0]);
            case dynamixel_types::ABILITY_CCW_LIMIT:
                return &(abilityInfoList[1]);
            case dynamixel_types::ABILITY_CW_LIMIT:
                return &(abilityInfoList[2]);
            case dynamixel_types::ABILITY_WHEEL_MODE:
                return &(abilityInfoList[3]);
            case dynamixel_types::ABILITY_GOAL_POSITION:
                return &(abilityInfoList[4]);
            case dynamixel_types::ABILITY_JOINT_MODE:
                return &(abilityInfoList[5]);
            case dynamixel_types::ABILITY_PRESENT_SPEED:
                return &(abilityInfoList[6]);
            case dynamixel_types::ABILITY_PRESENT_POSITION:
                return &(abilityInfoList[7]);
            case dynamixel_types::ABILITY_STATUS_RETURN_LEVEL:
                return &(abilityInfoList[8]);
            default:
                return 0;
        }
    }
    else if(requestedAbility.mode == dynamixel_types::MODE_QUERY)
    {
        switch(requestedAbility.abilityRequested)
        {
            case dynamixel_types::ABILITY_PRESENT_SPEED:
                return &(abilityInfoList[6]);
            case dynamixel_types::ABILITY_PRESENT_POSITION:
                return &(abilityInfoList[7]);
            default:
                return 0;
        }
    }

    return 0;
}
