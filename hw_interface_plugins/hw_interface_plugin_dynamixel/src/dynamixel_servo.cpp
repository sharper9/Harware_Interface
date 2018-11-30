#include <hw_interface_plugin_dynamixel/dynamixel_servo.hpp>

hw_interface_plugin_dynamixel::dynamixel_types::dynamixel_ability_result_t
    hw_interface_plugin_dynamixel::dynamixel_servo::abilityRequest(dynamixel_types::dynamixel_ability_request_t &request)
{
    request.requestResult = verifyAbilityRequest(request);

    if(request.requestResult == dynamixel_types::REQUEST_VALID)
    {
        dynamixel_types::dynamixel_ability_info_t *tableEntry = 0;
        if(!(tableEntry = _implAbilityInfoGet(request)))
        {
            return request.requestResult = dynamixel_types::REQUEST_ERROR_ABILITY_NOT_SUPPORTED;
        }

        tableEntry->flaggedForUpdate = request.mode;
        request.requestResult = dynamixel_types::REQUEST_SUCCESS;
        if(request.mode == dynamixel_types::MODE_GET)
        {
            //take request buffer and convert it to
            request.requestResult = dynamixel_types::REQUEST_SUCCESS;
        }
        else if(request.mode == dynamixel_types::MODE_SET)
        { 
            //check for ranges, units, and other boundaries here
            switch (tableEntry->abilityDataType)
            {
                case dynamixel_types::UNSIGNED_BYTE:
                case dynamixel_types::SIGNED_BYTE:
                    *((uint8_t*) &(underLyingActuatorMemory.data()[tableEntry->tableAddress])) = request.data.uBYTE; break;

                case dynamixel_types::UNSIGNED_WORD:
                case dynamixel_types::SIGNED_WORD:
                    *((uint16_t*) &(underLyingActuatorMemory.data()[tableEntry->tableAddress])) = request.data.uWORD; break;

                case dynamixel_types::UNSIGNED_DWORD:
                case dynamixel_types::SIGNED_DWORD:
                    *((uint32_t*) &(underLyingActuatorMemory.data()[tableEntry->tableAddress])) = request.data.uDWORD; break;

                default:
                    break;
            }
        }
    }

    return request.requestResult;
}

hw_interface_plugin_dynamixel::dynamixel_types::dynamixel_ability_result_t
    hw_interface_plugin_dynamixel::dynamixel_servo::verifyAbilityRequest(const dynamixel_types::dynamixel_ability_request_t &request)
{
    ROS_DEBUG_EXTRA_ID("Abilities for ID: %d || %lx", id, packedAbilities);
    ROS_DEBUG_EXTRA_ID("Ability %s::%d requested", dynamixel_helpers::abilityToString(request.abilityRequested).c_str(), (int)request.abilityRequested);
    if(!queryActuatorAbilitySupport(request.abilityRequested))
    {
        ROS_ERROR_EXTRA_ID("Ability Requested to ID: %d || Actuator model: %x not supported!", id, modelNumber);
        return dynamixel_types::REQUEST_ERROR_ABILITY_NOT_SUPPORTED;
    }
    ROS_DEBUG_EXTRA_ID("ID: %d :: request valid", id);
    return dynamixel_types::REQUEST_VALID;
}

hw_interface_plugin_dynamixel::dynamixel_types::dynamixel_ability_result_t
    hw_interface_plugin_dynamixel::dynamixel_servo::requestOutputUpdate(dynamixel_types::dynamixel_ability_request_t &request)
{
    requestOutputUpdate(request, request.outputDataBuf);
}

hw_interface_plugin_dynamixel::dynamixel_types::dynamixel_ability_result_t
    hw_interface_plugin_dynamixel::dynamixel_servo::requestOutputUpdate(dynamixel_types::dynamixel_ability_request_t &request, std::vector<uint8_t> &outputDataBuf)
{
    //probably should loop over flagged abilities, maybe

    dynamixel_types::dynamixel_ability_info_t *tableEntry = 0;
    if(!(tableEntry = _implAbilityInfoGet(request)))
    {
        return dynamixel_types::REQUEST_ERROR_ABILITY_NOT_SUPPORTED;
    }

    ROS_DEBUG_EXTRA_ID("Ability %s requested, found table with %s", dynamixel_helpers::abilityToString(request.abilityRequested).c_str(),
                                                            dynamixel_helpers::abilityToString(tableEntry->ability).c_str());
    ROS_DEBUG_EXTRA_ID("Table data type %d", (int)tableEntry->abilityDataType);
    ROS_DEBUG_EXTRA_ID("Request mode %d", (int)request.mode);
    ROS_DEBUG_EXTRA_ID("instruction %d", (int)request.instruction);

    if(request.mode == dynamixel_types::MODE_SET)
    {
        switch(getCommVersion())
        {
            case dynamixel_types::COMM_V1:
                ROS_DEBUG_EXTRA_ID("table entry-> address %d, dataType %d", tableEntry->tableAddress, (int)tableEntry->abilityDataType);
                outputDataBuf.push_back((uint8_t)(tableEntry->tableAddress & 0xff));
                goto dataInsertJump;

            case dynamixel_types::COMM_V2:
                outputDataBuf.push_back((uint8_t)(tableEntry->tableAddress & 0xff));
                outputDataBuf.push_back((uint8_t)((tableEntry->tableAddress & 0xff00)>>8));

dataInsertJump:

                switch (tableEntry->abilityDataType)
                {
                    case dynamixel_types::SIGNED_BYTE:
                    case dynamixel_types::UNSIGNED_BYTE:
                        outputDataBuf.push_back((uint8_t)(underLyingActuatorMemory.data()[tableEntry->tableAddress])); break;

                    case dynamixel_types::SIGNED_WORD:
                    case dynamixel_types::UNSIGNED_WORD:
                        outputDataBuf.push_back((uint8_t)(underLyingActuatorMemory.data()[tableEntry->tableAddress]));
                        outputDataBuf.push_back((uint8_t)(underLyingActuatorMemory.data()[tableEntry->tableAddress+1])); break;

                    case dynamixel_types::SIGNED_DWORD:
                    case dynamixel_types::UNSIGNED_DWORD:
                        outputDataBuf.push_back((uint8_t)(underLyingActuatorMemory.data()[tableEntry->tableAddress]));
                        outputDataBuf.push_back((uint8_t)(underLyingActuatorMemory.data()[tableEntry->tableAddress+1]));
                        outputDataBuf.push_back((uint8_t)(underLyingActuatorMemory.data()[tableEntry->tableAddress+2]));
                        outputDataBuf.push_back((uint8_t)(underLyingActuatorMemory.data()[tableEntry->tableAddress+3])); break;
                    default:
                        break;
                }

            default:
                break;
        }
    }
    else if((request.mode == dynamixel_types::MODE_GET) || (request.mode == dynamixel_types::MODE_QUERY))
    {
        switch(getCommVersion())
        {
            case dynamixel_types::COMM_V1:
                outputDataBuf.push_back((uint8_t)(tableEntry->tableAddress & 0xff));
                outputDataBuf.push_back((uint8_t)(tableEntry->abilityDataType & 0xff));
                lastReadOffset = tableEntry->tableAddress;
                lastAbility = tableEntry->ability;
                lastDataType = tableEntry->abilityDataType;
                break;
            case dynamixel_types::COMM_V2:
                outputDataBuf.push_back((uint8_t)(tableEntry->tableAddress & 0xff));
                outputDataBuf.push_back((uint8_t)((tableEntry->tableAddress & 0xff00)>>8));
                outputDataBuf.push_back((uint8_t)(tableEntry->abilityDataType & 0xff)); //the ability data type is the length.
                outputDataBuf.push_back((uint8_t)(tableEntry->abilityDataType & 0));
                lastReadOffset = tableEntry->tableAddress;
                lastAbility = tableEntry->ability;
                lastDataType = tableEntry->abilityDataType;
                break;

            default:
                break;
        }
    }
}

void hw_interface_plugin_dynamixel::dynamixel_servo::updateFromActuatorResponse(hw_interface_plugin_dynamixel::dynamixel_types::comm_v1_status_packet_t &response)
{
    ROS_DEBUG("%u :: Update Actuator %u", getID(),response.id);
    ROS_DEBUG("%x || %lu || %x || %x", response.header, response.parameters.size(), response.errorStatus, response.checksum);
//    for(size_t i = 0; i < response.parameters.size(); i++)
//    {
//        std::printf("%x || ", response.parameters[i]);
//    }
//    std::printf("\r\n");
    ROS_DEBUG("%u out", getID());
    if(getID() == response.id)
    {
        std::copy(response.parameters.begin(), response.parameters.end(), underLyingActuatorMemory.begin()+lastReadOffset);
        servoStateResponse latestState; latestState.id=getID(); latestState.queryAbility=lastAbility;
        switch(lastDataType)
        {
            case dynamixel_types::SIGNED_BYTE:
            case dynamixel_types::UNSIGNED_BYTE:
                latestState.uDATA=*((uint8_t*) &(underLyingActuatorMemory.data()[lastReadOffset])); break;

            case dynamixel_types::SIGNED_WORD:
            case dynamixel_types::UNSIGNED_WORD:
                latestState.uDATA=*((uint16_t*) &(underLyingActuatorMemory.data()[lastReadOffset])); break;

            case dynamixel_types::SIGNED_DWORD:
            case dynamixel_types::UNSIGNED_DWORD:
                latestState.uDATA=*((uint32_t*) &(underLyingActuatorMemory.data()[lastReadOffset])); break;
            default:
                latestState.uDATA=0; break;
        }
        servoStateUpdate.publish(latestState);
    }
}

void hw_interface_plugin_dynamixel::dynamixel_servo::updateFromActuatorResponse(hw_interface_plugin_dynamixel::dynamixel_types::comm_v2_status_packet_t &response)
{
    ROS_DEBUG("%u :: Update Actuator %u", getID(),response.id);
    ROS_DEBUG("%x || %lu || %x || %x", response.h1, response.parameters.size(), response.errorStatus, response.crc16);
//    for(size_t i = 0; i < response.parameters.size(); i++)
//    {
//        std::printf("%x || ", response.parameters[i]);
//    }
//    std::printf("\r\n");
    ROS_DEBUG("%u out", getID());
    if(getID() == response.id)
    {
        std::copy(response.parameters.begin(), response.parameters.end(), underLyingActuatorMemory.begin()+lastReadOffset);
        servoStateResponse latestState; latestState.id=getID(); latestState.queryAbility=lastAbility;
        switch(lastDataType)
        {
            case dynamixel_types::SIGNED_BYTE:
                latestState.sDATA=*((int8_t*) &(underLyingActuatorMemory.data()[lastReadOffset])); break;
            case dynamixel_types::UNSIGNED_BYTE:
                latestState.uDATA=*((uint8_t*) &(underLyingActuatorMemory.data()[lastReadOffset])); break;

            case dynamixel_types::SIGNED_WORD:
                latestState.sDATA=*((int16_t*) &(underLyingActuatorMemory.data()[lastReadOffset])); break;
            case dynamixel_types::UNSIGNED_WORD:
                latestState.uDATA=*((uint16_t*) &(underLyingActuatorMemory.data()[lastReadOffset])); break;

            case dynamixel_types::SIGNED_DWORD:
                latestState.sDATA=*((int32_t*) &(underLyingActuatorMemory.data()[lastReadOffset])); break;
            case dynamixel_types::UNSIGNED_DWORD:
                latestState.uDATA=*((uint32_t*) &(underLyingActuatorMemory.data()[lastReadOffset])); break;
            default:
                latestState.uDATA=0; break;
        }
        servoStateUpdate.publish(latestState);
    }
}
