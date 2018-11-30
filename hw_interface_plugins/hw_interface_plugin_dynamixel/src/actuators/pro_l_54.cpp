#include <hw_interface_plugin_dynamixel/actuators/pro_l_54.hpp>


hw_interface_plugin_dynamixel::dynamixel_actuators::pro_l_54::pro_l_54(dynamixel_types::dynamixel_id_number_t id, std::vector<uint8_t> parameters,
                                                                 dynamixel_types::comm_version_t commVersion):
    dynamixel_servo(
        id,
        *( (dynamixel_types::dynamixel_model_number_t*)&parameters.data()[L_54_MODEL_NUMBER_INIT_OFFSET]),
        *( (dynamixel_types::dynamixel_firmware_version_t*)&parameters.data()[L_54_FIRMWARE_REVISION_INIT_OFFSET])
        )
{ 
    /*
     * if (version num)
     *  {select this protocol}
     */
    packedAbilities|=    dynamixel_types::ABILITY_ANGLE_UNITS   |   dynamixel_types::ABILITY_SPEED_UNITS    |   dynamixel_types::ABILITY_RESOLUTION_DIVIDER
                        |dynamixel_types::ABILITY_WHEEL_MODE    |   dynamixel_types::ABILITY_JOINT_MODE     |   dynamixel_types::ABILITY_CCW_LIMIT
                        |dynamixel_types::ABILITY_CW_LIMIT      |   dynamixel_types::ABILITY_MOVING_SPEED   |   dynamixel_types::ABILITY_GOAL_POSITION
                        |dynamixel_types::ABILITY_PRESENT_POSITION| dynamixel_types::ABILITY_PRESENT_SPEED  |   dynamixel_types::ABILITY_STATUS_RETURN_LEVEL
                        |dynamixel_types::ABILITY_OPERATING_MODE|   dynamixel_types::ABILITY_TORQUE         |   dynamixel_types::ABILITY_PRESENT_CURRENT
                        |dynamixel_types::ABILITY_TORQUE_LIMIT  |   dynamixel_types::ABILITY_GOAL_TORQUE;

    protocolVersion = commVersion;
    actuatorSeries = dynamixel_types::MX;
    actuatorVersion = dynamixel_types::PRO_L_54;

    underLyingActuatorMemory.assign(950, 0);
    //0
    abilityInfoList.emplace_back(dynamixel_types::ABILITY_MOVING_SPEED,     dynamixel_types::MODE_GET | dynamixel_types::MODE_SET,
                                 600, dynamixel_types::UNIT_RAW,         dynamixel_types::SIGNED_DWORD, 0, dynamixel_types::MODE_NOP);
    //1
    abilityInfoList.emplace_back(dynamixel_types::ABILITY_CCW_LIMIT,        dynamixel_types::MODE_GET | dynamixel_types::MODE_SET,
                                 0x08, dynamixel_types::UNIT_RAW,       dynamixel_types::UNSIGNED_WORD, 0, dynamixel_types::MODE_NOP);
    //2
    abilityInfoList.emplace_back(dynamixel_types::ABILITY_CW_LIMIT,         dynamixel_types::MODE_GET | dynamixel_types::MODE_SET,
                                 0x06, dynamixel_types::UNIT_RAW,       dynamixel_types::UNSIGNED_WORD, 0, dynamixel_types::MODE_NOP);
    //3
    abilityInfoList.emplace_back(dynamixel_types::ABILITY_WHEEL_MODE,       dynamixel_types::MODE_GET | dynamixel_types::MODE_SET,
                                 0x06, dynamixel_types::UNIT_RAW,       dynamixel_types::UNSIGNED_DWORD, 0, dynamixel_types::MODE_NOP);
    //4
    abilityInfoList.emplace_back(dynamixel_types::ABILITY_GOAL_POSITION,    dynamixel_types::MODE_GET | dynamixel_types::MODE_SET,
                                 596, dynamixel_types::UNIT_RAW,       dynamixel_types::SIGNED_DWORD, 0, dynamixel_types::MODE_NOP);
    //5
    abilityInfoList.emplace_back(dynamixel_types::ABILITY_JOINT_MODE, dynamixel_types::MODE_GET | dynamixel_types::MODE_SET,
                                 0x08, dynamixel_types::UNIT_RAW, dynamixel_types::UNSIGNED_WORD, 0, dynamixel_types::MODE_NOP);
    //6
    abilityInfoList.emplace_back(dynamixel_types::ABILITY_PRESENT_SPEED, dynamixel_types::MODE_QUERY | dynamixel_types::MODE_GET | dynamixel_types::MODE_SET,
                                 615, dynamixel_types::UNIT_RAW, dynamixel_types::SIGNED_DWORD, 0, dynamixel_types::MODE_NOP);
    //7
    abilityInfoList.emplace_back(dynamixel_types::ABILITY_PRESENT_POSITION, dynamixel_types::MODE_QUERY | dynamixel_types::MODE_GET | dynamixel_types::MODE_SET,
                                 611, dynamixel_types::UNIT_RAW, dynamixel_types::SIGNED_DWORD, 0, dynamixel_types::MODE_NOP);
    //8
    abilityInfoList.emplace_back(dynamixel_types::ABILITY_STATUS_RETURN_LEVEL, dynamixel_types::MODE_QUERY | dynamixel_types::MODE_GET | dynamixel_types::MODE_SET,
                                 891, dynamixel_types::UNIT_RAW, dynamixel_types::UNSIGNED_BYTE, 0, dynamixel_types::MODE_NOP);
    //9
    abilityInfoList.emplace_back(dynamixel_types::ABILITY_OPERATING_MODE, dynamixel_types::MODE_GET | dynamixel_types::MODE_SET,
                                 11, dynamixel_types::UNIT_RAW, dynamixel_types::UNSIGNED_BYTE, 0, dynamixel_types::MODE_NOP);
    //10
    abilityInfoList.emplace_back(dynamixel_types::ABILITY_TORQUE, dynamixel_types::MODE_QUERY | dynamixel_types::MODE_GET | dynamixel_types::MODE_SET,
                                 562, dynamixel_types::UNIT_RAW, dynamixel_types::UNSIGNED_BYTE, 0, dynamixel_types::MODE_NOP);
    //11
    abilityInfoList.emplace_back(dynamixel_types::ABILITY_PRESENT_CURRENT, dynamixel_types::MODE_QUERY | dynamixel_types::MODE_GET,
                                 621, dynamixel_types::UNIT_RAW, dynamixel_types::UNSIGNED_WORD, 0, dynamixel_types::MODE_NOP);
    //12
    abilityInfoList.emplace_back(dynamixel_types::ABILITY_TORQUE_LIMIT, dynamixel_types::MODE_GET | dynamixel_types::MODE_SET,
                                 30, dynamixel_types::UNIT_RAW, dynamixel_types::UNSIGNED_WORD, 0, dynamixel_types::MODE_NOP);
    //13
    abilityInfoList.emplace_back(dynamixel_types::ABILITY_GOAL_TORQUE, dynamixel_types::MODE_GET | dynamixel_types::MODE_SET,
                                 604, dynamixel_types::UNIT_RAW, dynamixel_types::UNSIGNED_WORD, 0, dynamixel_types::MODE_NOP);



}


hw_interface_plugin_dynamixel::dynamixel_types::dynamixel_ability_result_t
    hw_interface_plugin_dynamixel::dynamixel_actuators::pro_l_54::_implAbilityRequest(dynamixel_types::dynamixel_ability_request_t &request)
{


}

hw_interface_plugin_dynamixel::dynamixel_types::dynamixel_ability_info_t *
    hw_interface_plugin_dynamixel::dynamixel_actuators::pro_l_54::_implAbilityInfoGet(dynamixel_types::dynamixel_ability_request_t &requestedAbility)
{
    if(requestedAbility.mode == dynamixel_types::MODE_GET)
    {
        switch(requestedAbility.abilityRequested)
        {
            case dynamixel_types::ABILITY_MOVING_SPEED:             return &(abilityInfoList[0]);
            case dynamixel_types::ABILITY_CCW_LIMIT:                return &(abilityInfoList[1]);
            case dynamixel_types::ABILITY_CW_LIMIT:                 return &(abilityInfoList[2]);
            case dynamixel_types::ABILITY_WHEEL_MODE:               return &(abilityInfoList[3]);
            case dynamixel_types::ABILITY_GOAL_POSITION:            return &(abilityInfoList[4]);
            case dynamixel_types::ABILITY_JOINT_MODE:               return &(abilityInfoList[5]);
            case dynamixel_types::ABILITY_PRESENT_SPEED:            return &(abilityInfoList[6]);
            case dynamixel_types::ABILITY_PRESENT_POSITION:         return &(abilityInfoList[7]);
            case dynamixel_types::ABILITY_STATUS_RETURN_LEVEL:      return &(abilityInfoList[8]);
            case dynamixel_types::ABILITY_OPERATING_MODE:           return &(abilityInfoList[9]);
            case dynamixel_types::ABILITY_TORQUE:                   return &(abilityInfoList[10]);
            case dynamixel_types::ABILITY_PRESENT_CURRENT:          return &(abilityInfoList[11]);
            case dynamixel_types::ABILITY_TORQUE_LIMIT:             return &(abilityInfoList[12]);
            case dynamixel_types::ABILITY_GOAL_TORQUE:              return &(abilityInfoList[13]);
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
            case dynamixel_types::ABILITY_OPERATING_MODE:
                return &(abilityInfoList[9]);
            case dynamixel_types::ABILITY_TORQUE:
                return &(abilityInfoList[10]);
            case dynamixel_types::ABILITY_TORQUE_LIMIT:
                return &(abilityInfoList[12]);
            case dynamixel_types::ABILITY_GOAL_TORQUE:              return &(abilityInfoList[13]);
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
            case dynamixel_types::ABILITY_TORQUE:
                return &(abilityInfoList[10]);
            case dynamixel_types::ABILITY_PRESENT_CURRENT:
                return &(abilityInfoList[11]);
            default:
                return 0;
        }
    }

    return 0;
}
