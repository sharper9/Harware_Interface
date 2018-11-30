#ifndef DYNAMIXEL_TYPES_HPP__
#define DYNAMIXEL_TYPES_HPP__

#include <hw_interface/base_interface.hpp>
#include <hw_interface/shared_const_buffer.hpp>

#include <hw_interface_plugin_dynamixel/servoUpdate.h>
#include <hw_interface_plugin_dynamixel/groupServoUpdate.h>

#define COMM_V1_16_BIT_HEADER 0xffff


 
namespace hw_interface_plugin_dynamixel
{
    //forwards for the forwards
    namespace dynamixel_types
    {
    //capabilities
        enum dynamixel_ability_mode_t {
            MODE_NOP = 0, MODE_GET=1, MODE_SET=1<<1, MODE_QUERY=1<<2
        };

        enum dynamixel_table_mode_t {
            TABLE_READ=1, TABLE_WRITE=1<<1,
        };
        enum dynamixel_ability_data_type_t {
            UNSIGNED_BYTE=0x01, UNSIGNED_WORD=0x02, UNSIGNED_DWORD=0x04,
            SIGNED_BYTE=0x11, SIGNED_WORD=0x12, SIGNED_DWORD=0x14,
        };

        enum dynamixel_units_t {
            UNIT_RAW, UNIT_RPM, UNIT_DEGREES
        };

        enum dynamixel_abilities_t {
            ABILITY_NOP = 0, ABILITY_ANGLE_UNITS=1, ABILITY_SPEED_UNITS=1<<1, ABILITY_RESOLUTION_DIVIDER=1<<2, ABILITY_WHEEL_MODE=1<<3,
            ABILITY_JOINT_MODE=1<<4, ABILITY_CCW_LIMIT=1<<5, ABILITY_CW_LIMIT=1<<6, ABILITY_INIT=1<<7, ABILITY_MOVING_SPEED=1<<8,
            ABILITY_GOAL_POSITION=1<<9, ABILITY_PRESENT_POSITION=1<<10, ABILITY_PRESENT_SPEED=1<<11, ABILITY_STATUS_RETURN_LEVEL=1<<12,
            ABILITY_OPERATING_MODE=1<<13, ABILITY_TORQUE=1<<14, ABILITY_PRESENT_CURRENT=1<<15, ABILITY_TORQUE_LIMIT=1<<16,
            ABILITY_GOAL_TORQUE=1<<17,
        };

        enum dynamixel_ability_result_t {
            REQUEST_SUCCESS = 0, REQUEST_VALID, REQUEST_INVALID, REQUEST_ERROR_UNKNOWN_INPUT, REQUEST_ERROR_INPUT_LIMIT_EXCEEDED,
            REQUEST_ERROR_ABILITY_NOT_SUPPORTED, REQUEST_GENERAL_ERROR, REQUEST_MODE_NOT_SUPPORTED
        };

        enum dynamixel_actuator_version_t {
            UNKNOWN_ACTUATOR, MX_12, MX_28, MX_64, MX_106, PRO_H_54, PRO_H_42, PRO_M_54, PRO_M_42, PRO_L_54,

            ACTUATOR_VER_LAST
        };

        enum dynamixel_actuator_series_t {
            UNKNOWN_SERIES, DX, AX, RX, EX, MX, XL, XM, XH, PRO_H, PRO_M, PRO_L,

            ACTUATOR_SERIES_LAST
        };

        enum comm_version_t {
            COMM_V1, COMM_V2,
        };

        enum comm_instruction_t {
            INT_PING=0x01, INT_READ_DATA=0x02, INT_WRITE_DATA=0x03, INT_REG_WRITE=0x04,
            INT_ACTION=0x05, INT_RESET=0x06, V2_INT_SYNC_READ=0x82, INT_SYNC_WRITE=0x83,
            V2_INT_BULK_READ=0x92, V2_INT_BULK_WRITE=0x93, V2_INT_STATUS=0x55
        };

        typedef struct s_comm_v1_packet        comm_v1_packet_t;
        typedef struct s_comm_v1_status_packet comm_v1_status_packet_t;

        typedef struct s_comm_v2_packet        comm_v2_packet_t;
        typedef struct s_comm_v2_status_packet comm_v2_status_packet_t;

        typedef union  u_ability_data           dynamixel_ability_data_t;
        typedef struct s_ability_request        dynamixel_ability_request_t;
        typedef struct s_ability_info           dynamixel_ability_info_t;

        typedef uint16_t dynamixel_model_number_t;
        typedef uint8_t  dynamixel_id_number_t;
        typedef uint8_t  dynamixel_torque_enable_t;
        typedef uint8_t  dynamixel_firmware_version_t;
        typedef uint32_t dynamixel_model_info_t;
        typedef uint64_t dynamixel_packed_abilites_t;
        typedef uint8_t  dynamixel_table_packed_mode_t;
        typedef uint16_t dynamixel_table_address_t;
        typedef uint32_t dynamixel_largest_data_container_t;

        typedef dynamixel_id_number_t dynamixel_shadow_id_number_t;

        class const_shared_buf_dynamixel;

    }
//need to move these of the header
    namespace dynamixel_helpers
    {
        static uint16_t calcChecksum(char* data, std::size_t length, dynamixel_types::comm_version_t commVersion = dynamixel_types::COMM_V1)
        {
            if(commVersion == dynamixel_types::COMM_V1)
            {
                uint16_t result = 0;
                for(int i = 2; i < length; i++)
                {
                    result+=data[i];
                }
                return ~result;
            }
            else
            {
                return boost::crc<16, 0x8005, 0x0000, 0x0000, false, false>(data, length);
            }
        }

        static std::string commVersionToString(dynamixel_types::comm_version_t commV)
        {
            switch(commV)
            {
                case dynamixel_types::COMM_V1:  return std::string("COMM_V1");
                case dynamixel_types::COMM_V2:  return std::string("COMM_V2");
                default:                        return std::string("Comm Version Unknown");
            }
        }

        static std::string abilityToString(dynamixel_types::dynamixel_abilities_t ability)
        {
            switch(ability)
            {
                case dynamixel_types::ABILITY_NOP: return std::string("dynamixel_types::ABILITY_NOP");
                case dynamixel_types::ABILITY_ANGLE_UNITS: return std::string("dynamixel_types::ABILITY_ANGLE_UNITS");
                case dynamixel_types::ABILITY_SPEED_UNITS: return std::string("dynamixel_types::ABILITY_SPEED_UNITS");
                case dynamixel_types::ABILITY_RESOLUTION_DIVIDER: return std::string("dynamixel_types::ABILITY_RESOLUTION_DIVIDER");
                case dynamixel_types::ABILITY_WHEEL_MODE: return std::string("dynamixel_types::ABILITY_WHEEL_MODE");
                case dynamixel_types::ABILITY_JOINT_MODE: return std::string("dynamixel_types::ABILITY_JOINT_MODE");
                case dynamixel_types::ABILITY_CCW_LIMIT: return std::string("dynamixel_types::ABILITY_CCW_LIMIT");
                case dynamixel_types::ABILITY_CW_LIMIT: return std::string("dynamixel_types::ABILITY_CW_LIMIT");
                case dynamixel_types::ABILITY_INIT: return std::string("dynamixel_types::ABILITY_INIT");
                case dynamixel_types::ABILITY_MOVING_SPEED: return std::string("dynamixel_types::ABILITY_MOVING_SPEED");
                case dynamixel_types::ABILITY_GOAL_POSITION: return std::string("dynamixel_types::ABILITY_GOAL_POSITION");
                case dynamixel_types::ABILITY_PRESENT_POSITION: return std::string("dynamixel_types::ABILITY_PRESENT_POSITION");
                case dynamixel_types::ABILITY_PRESENT_SPEED: return std::string("dynamixel_types::ABILITY_PRESENT_SPEED");
                case dynamixel_types::ABILITY_OPERATING_MODE: return std::string("dynamixel_types::ABILITY_OPERATING_MODE");

                default: return std::string("Ability Unknown");
            }
        }
    }
//need to move these out of the headers
    namespace dynamixel_types
    {
        typedef struct s_comm_v1_packet comm_v1_packet_t;
        struct s_comm_v1_packet
        {
            uint16_t header;
            uint8_t id;
            uint8_t length; //length of instruction + 2
            uint8_t instruction;
            std::vector<uint8_t> parameters;
            uint8_t checksum;
            s_comm_v1_packet():
                header(0xffff){}
        };

        typedef struct s_comm_v1_status_packet comm_v1_status_packet_t;
        struct s_comm_v1_status_packet
        {
            s_comm_v1_status_packet(){}
            s_comm_v1_status_packet(uint8_t* data)
            {
                header = *((uint16_t*) data); data+=2;
                    id = *data; data++;
                length = *data; data++;
                errorStatus = *data; data++;
                parameters.insert(parameters.end(), data, data+length-2); data+=length-2; //is this wrong?
                checksum = *data;
            }

            uint16_t header;
            uint8_t id;
            uint8_t length; //length of instruction + 2
            uint8_t errorStatus;
            std::vector<uint8_t> parameters;
            uint8_t checksum;
        };


        typedef struct s_comm_v2_packet        comm_v2_packet_t;
        struct s_comm_v2_packet
        {
            uint16_t h1; //0xff 0xff
            uint16_t h2; //0xfd 0x00
            uint8_t  id;
            uint16_t length;
            uint8_t  instruction;
            std::vector<uint8_t> parameters;
            uint16_t crc16;
        };

        typedef struct s_comm_v2_status_packet comm_v2_status_packet_t;
        struct s_comm_v2_status_packet
        {
            s_comm_v2_status_packet(){}
            s_comm_v2_status_packet(uint8_t* data)
            {
                h1 = *((uint16_t*) data); data+=2;
                h2 = *((uint16_t*) data); data+=2;
                id = *data; data++;
                length = *((uint16_t*) data); data+=2;
                instruction = *data; data++;
                errorStatus = *data; data++;
                parameters.insert(parameters.end(), data, data+length-3); data+=length-3; //is this wrong?
                crc16 = *((uint16_t*) data); data+=2;
            }
            uint16_t h1; //0xff 0xff
            uint16_t h2; //0xfd 0x00 or 0xfd 0xfd
            uint8_t  id;
            uint16_t length;
            uint8_t  instruction; //set at 0x55
            uint8_t  errorStatus;
            std::vector<uint8_t> parameters;
            uint16_t crc16;
        };



        typedef union  u_ability_data dynamixel_ability_data_t;
        union u_ability_data
        {
            uint32_t uDWORD;
            int32_t  sDWORD;
            uint16_t uWORD;
            int16_t  sWORD;
            uint8_t  uBYTE;
            int8_t   sBYTE;
            u_ability_data():uDWORD(0){}
            u_ability_data(dynamixel_largest_data_container_t _data
                           ): uDWORD(_data)
            {}
        };

        typedef struct s_ability_info dynamixel_ability_info_t;
        struct s_ability_info
        {
            dynamixel_abilities_t ability;
            dynamixel_table_packed_mode_t tableSupportedModes;

            dynamixel_table_address_t tableAddress;

            dynamixel_units_t abilityUnits;
            float unitConversion;

            dynamixel_ability_data_type_t abilityDataType;
            dynamixel_ability_data_t data;

            dynamixel_ability_mode_t flaggedForUpdate;
            s_ability_info(){}
            s_ability_info(dynamixel_abilities_t _ability, dynamixel_table_packed_mode_t _packedTableModes,
                           dynamixel_table_address_t address,
                           dynamixel_units_t _units  , dynamixel_ability_data_type_t _dataType,
                           dynamixel_largest_data_container_t _data, dynamixel_ability_mode_t _flagged
                           ): ability(_ability), tableSupportedModes(_packedTableModes), tableAddress(address),
                                abilityUnits(_units), abilityDataType(_dataType), data(_data),
                                flaggedForUpdate(_flagged)
            {}
        };

        typedef struct s_ability_request dynamixel_ability_request_t;
        struct s_ability_request
        {
            //ability inputs
            dynamixel_id_number_t       id;
            size_t vectorLocation;
            dynamixel_ability_mode_t    mode;
            dynamixel_abilities_t       abilityRequested;
            dynamixel_ability_data_t    data;

            comm_instruction_t instruction; //instructs the ability to include the servo id in the data output

            //ability outputs
            dynamixel_ability_result_t requestResult;
            std::vector<uint8_t> outputDataBuf;
            s_ability_request(dynamixel_ability_mode_t _mode, dynamixel_abilities_t _ability, dynamixel_largest_data_container_t _data,
                              comm_instruction_t _instruction):
                mode(_mode), abilityRequested(_ability), data(_data), instruction(_instruction) {}

            s_ability_request(const hw_interface_plugin_dynamixel::servoUpdate &msg, size_t vectorRef):
                id(msg.id), mode((dynamixel_ability_mode_t)msg.mode), abilityRequested((dynamixel_abilities_t)msg.abilityRequested),
//todo, remove instruction field
                data((msg.isDataSigned)?msg.sDATA:msg.uDATA), vectorLocation(vectorRef)
            {}

            s_ability_request(s_ability_request &&other):
                id(other.id), vectorLocation(other.vectorLocation), mode(other.mode), abilityRequested(other.abilityRequested),
                instruction(other.instruction),
                requestResult(other.requestResult), outputDataBuf(std::move(other.outputDataBuf))
            {}
            void moveCopy(s_ability_request &&other)
            {
                this->clear(true);
                id=other.id;
                vectorLocation=other.vectorLocation;
                mode=other.mode;
                abilityRequested=other.abilityRequested;
                instruction=other.instruction;
                requestResult=other.requestResult;
                outputDataBuf=other.outputDataBuf;
            }

            s_ability_request(const s_ability_request &other):
                id(other.id), vectorLocation(other.vectorLocation), mode(other.mode), abilityRequested(other.abilityRequested),
                instruction(other.instruction),
                requestResult(other.requestResult), outputDataBuf(other.outputDataBuf)
            {}

            s_ability_request(){}

            void clear(bool clearBuffer)
            {
                mode = MODE_NOP;
                abilityRequested = ABILITY_NOP;
                data.uDWORD = 0x00000000;
                requestResult = REQUEST_SUCCESS;
                if(clearBuffer){outputDataBuf.clear();}
            }
        };


        class const_shared_buf_dynamixel : public hw_interface_support_types::shared_const_buffer
        {
            public :
                const_shared_buf_dynamixel(const comm_v1_packet_t &data)
                {
                    initV1(data);
                }
                const_shared_buf_dynamixel(const comm_v2_packet_t &data)
                {
                    initV2(data);
                }

            private:
                void initV1(const comm_v1_packet_t &data)
                {
                    data_.reset(new std::vector<char>());
                    data_->push_back((char)data.header);data_->push_back((char)data.header);
                    data_->push_back(data.id);
                    data_->push_back(data.length);
                    data_->push_back(data.instruction);
                    data_->insert( data_->end(), data.parameters.begin(), data.parameters.end());
                    data_->push_back(dynamixel_helpers::calcChecksum(data_->data(), data_->size()));
                    buffer_.reset(new boost::asio::const_buffer(boost::asio::buffer(*data_)));
                    //std::printf("Const shared buf V1\r\n");
//                    for(int i = 0; i < data_->size(); i++)
//                    {
//                        std::printf("%x || ",0xff&data_->data()[i]);
//                    }
//                    std::printf("\r\n");
                }
                void initV2(const comm_v2_packet_t &data)
                {
                    data_.reset(new std::vector<char>());
                    data_->push_back((char)0xff);data_->push_back((char)0xff);data_->push_back((char)0xfd);data_->push_back((char)0x00);
                    data_->push_back(data.id);
                    data_->push_back(((uint8_t)data.length)&0xff);
                    data_->push_back((((uint8_t)data.length)>>8)&0xff);
                    data_->push_back(data.instruction);
                    data_->insert( data_->end(), data.parameters.begin(), data.parameters.end());
                    uint16_t checksum = dynamixel_helpers::calcChecksum(data_->data(), data_->size(), COMM_V2);
                    data_->push_back(((uint8_t)checksum)&0xff);
                    data_->push_back((uint8_t)((checksum>>8)&0xff));
                    buffer_.reset(new boost::asio::const_buffer(boost::asio::buffer(*data_)));
//                    std::printf("Const shared buf V1\r\n");
//                    for(int i = 0; i < data_->size(); i++)
//                    {
//                        std::printf("%x || ",0xff&data_->data()[i]);
//                    }
//                    std::printf("\r\n");
                }


                //void initV2(const comm)
        };
    }


}

#endif //DYNAMIXEL_TYPES_HPP__
