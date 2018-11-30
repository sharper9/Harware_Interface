#ifndef DYNAMIXEL_SERVO_HPP
#define DYNAMIXEL_SERVO_HPP

#include <ros/ros.h>
#include <hw_interface_plugin_dynamixel/dynamixel_forwards.hpp>
#include <hw_interface_plugin_dynamixel/dynamixel_servo_group.hpp>

#include <hw_interface_plugin_dynamixel/servoUpdate.h>
#include <hw_interface_plugin_dynamixel/servoStateResponse.h>

#include <sstream>
#include <string>
 
#define ROS_INFO_EXTRA_ID(arg1, args...)  ROS_INFO("%s:: " arg1 , std::to_string(id).c_str(), args)
#define ROS_DEBUG_EXTRA_ID(arg1, args...) ROS_DEBUG("%s:: " arg1 , std::to_string(id).c_str(), args)
#define ROS_WARN_EXTRA_ID(arg1, args...)  ROS_WARN("%s:: " arg1 , std::to_string(id).c_str(), args)
#define ROS_ERROR_EXTRA_ID(arg1, args...) ROS_ERROR("%s:: " arg1 , std::to_string(id).c_str(), args)

/*
 *
 * DO NOT SEND COMMANDS LONGER THAN THIS
 * BREAK UP THE COMMANDS
 */
#define RX_64_BUFF_SIZE 143
/*
 *
 * keep track of updated table values, compile identical updates between
 *  servos, send mass write with servo changes that match
 *
 * once groups are implemented, use sync write or reg_write modes to sync
 *  movements
 *
 * stopped at figuring out final inheritance diagram. need to figure
 *  out how to understand derived class's abilities.
 *
 * eventually want to support redirection registers so entire servo control
 *  tables can be configured to overlap and we can update the entire
 *  bus at once.
 *
 * need a ability to control table value and length for value writing and
 *  matching control tables
 *
 */

namespace hw_interface_plugin_dynamixel {

    class dynamixel_servo
    {
        public:

            virtual ~dynamixel_servo(){}

            virtual std::string to_string()
            {
                std::ostringstream os;
                os << "ID: ";
                os << std::dec << (unsigned int)id;
                os << " || Model #: ";
                os << std::hex << modelNumber;
                os << " || Firmware Revision: ";
                os << std::dec << (unsigned int)firmwareRevision;
                os << " || Protocol: " << dynamixel_helpers::commVersionToString(protocolVersion) << std::endl;
                return os.str();
            }

            std::string getRefName();
            void setRefName(std::string newRefName);

            dynamixel_types::comm_version_t     getCommVersion()                                           { return protocolVersion; }
            void                                setCommVersion(dynamixel_types::comm_version_t version)    { protocolVersion=version; }

            dynamixel_types::dynamixel_id_number_t          getID()                 { return id; }
            dynamixel_types::dynamixel_model_number_t       getModelNumber()        { return modelNumber; }
            dynamixel_types::dynamixel_firmware_version_t   getFirmwareRevision()   { return firmwareRevision; }
            dynamixel_types::dynamixel_actuator_version_t   getActuatorVersion()    { return actuatorVersion; }
            dynamixel_types::dynamixel_actuator_series_t    getActuatorSeries()     { return actuatorSeries; }

            //determines if the actuator AND driver supports the type of ability
            bool queryActuatorAbilitySupport(dynamixel_types::dynamixel_abilities_t ability){ return (packedAbilities & ability) != 0; }

            const dynamixel_types::dynamixel_ability_info_t&
                                queryActuatorAbilityInfo(dynamixel_types::dynamixel_abilities_t ability);

            //void addToGroup

            dynamixel_types::dynamixel_ability_result_t abilityRequest(dynamixel_types::dynamixel_ability_request_t &request);
            dynamixel_types::dynamixel_ability_result_t requestOutputUpdate(dynamixel_types::dynamixel_ability_request_t &request);
            dynamixel_types::dynamixel_ability_result_t requestOutputUpdate(dynamixel_types::dynamixel_ability_request_t &request, std::vector<uint8_t> &outputDataBuf);

            void updateFromActuatorResponse(dynamixel_types::comm_v1_status_packet_t &response);
            void updateFromActuatorResponse(dynamixel_types::comm_v2_status_packet_t &response);

            virtual dynamixel_types::dynamixel_ability_info_t * _implAbilityInfoGet(dynamixel_types::dynamixel_ability_request_t &requestedAbility)
            {
                return 0;
            }


        protected:

            std::string refName;

            std::vector<uint8_t> underLyingActuatorMemory;
            uint8_t lastReadOffset;
            dynamixel_types::dynamixel_abilities_t lastAbility; //remove later
            dynamixel_types::dynamixel_ability_data_type_t lastDataType; //remove later

            ros::Publisher servoStateUpdate;

            dynamixel_types::comm_version_t                 protocolVersion;

            dynamixel_types::dynamixel_model_number_t       modelNumber;
            dynamixel_types::dynamixel_id_number_t          id;
            dynamixel_types::dynamixel_actuator_version_t   actuatorVersion;
            dynamixel_types::dynamixel_actuator_series_t    actuatorSeries;
            dynamixel_types::dynamixel_firmware_version_t   firmwareRevision;
            dynamixel_types::dynamixel_torque_enable_t      torqueEnabled;

            dynamixel_types::dynamixel_packed_abilites_t    packedAbilities;

            std::vector<dynamixel_types::dynamixel_ability_info_t> abilityInfoList;

            dynamixel_servo(dynamixel_types::dynamixel_id_number_t          idFound,
                            dynamixel_types::dynamixel_model_number_t       model,
                            dynamixel_types::dynamixel_firmware_version_t   firmwareVersion,
                            dynamixel_types::comm_version_t                 commVersion
                            ):
                id(idFound),
                refName(std::to_string(id)),
                modelNumber(model),
                firmwareRevision(firmwareVersion),
                packedAbilities(0),
                protocolVersion(commVersion)
            {
                servoStateUpdate = ros::NodeHandle().advertise<hw_interface_plugin_dynamixel::servoStateResponse>(std::string("/servo/state/")+std::to_string(id), 1);
                torqueEnabled = 0;
            }

            dynamixel_servo(dynamixel_types::dynamixel_id_number_t          idFound,
                            dynamixel_types::dynamixel_model_number_t       model,
                            dynamixel_types::dynamixel_firmware_version_t   firmwareVersion
                            ):
                id(idFound),
                refName(std::to_string(id)),
                modelNumber(model),
                firmwareRevision(firmwareVersion),
                packedAbilities(0)
            {
                underLyingActuatorMemory.assign(1000, 0);
                servoStateUpdate = ros::NodeHandle().advertise<hw_interface_plugin_dynamixel::servoStateResponse>(std::string("/servo/state/")+std::to_string(id), 1);
                torqueEnabled = 0;
            }

            dynamixel_servo():
                dynamixel_servo(255,0,0)
            {}

            dynamixel_types::dynamixel_ability_result_t verifyAbilityRequest(const dynamixel_types::dynamixel_ability_request_t &request);

            virtual dynamixel_types::dynamixel_ability_result_t _implAbilityRequest(dynamixel_types::dynamixel_ability_request_t &request)=0;










    };

}



#endif //DYNAMIXEL_SERVO_HPP

