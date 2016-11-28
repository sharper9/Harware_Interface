/**
Contains a namspace of structure types for others processes to include and type convert topic array data to one of the appropriate
*/

namespace ranging_radio_types
{
    //see 320-0282E RCM API Specification.pdf
	//http://www.timedomain.com/datasheets/320-0282F%20RCM%20API%20Specification.pdf

	#define RCM_RANGE_INFO_MSGTYPE 0x0201
	struct RCM_RANGE_INFO_t
	{
	    uint32_t timestamp;
	    int32_t  coarseTOF;
	    uint16_t VPeak;
	    uint16_t channelRise;
	    uint16_t resFlags;
	    uint16_t reqFlags;
	    uint8_t  reserved;
	    uint8_t  rangeMeasType;
	    uint16_t FRVError;
	    uint16_t FRV;
	    uint16_t FREError;
	    uint16_t CREError;
	    uint16_t PRMError;
	    uint32_t FRE;
	    uint32_t CRE;
	    uint32_t PRM;
	    uint16_t durationOfConv;
	    uint8_t  antenMode;
	    uint8_t  rangeStatus;
	    uint32_t responderID;
	    uint16_t msgID;
	    uint16_t msgType;
	    uint16_t length;
	    uint16_t sync;
	} __attribute__ ((packed));
        const static int RCM_RANGE_INFO_SIZE = sizeof(RCM_RANGE_INFO_t);
	
	//constructed backwards

	#define SEND_RANGE_REQUEST_MSGTYPE 0x0003
	struct SEND_RANGE_REQUEST_t
	{
        uint8_t data;
        uint8_t dataSize;
        uint8_t reserved;
    	uint8_t antenMode;
    	uint32_t rspndNode;
    	uint16_t msgID;
    	uint16_t msgType;
    	uint16_t length; //length is size - 4
	    uint16_t sync;
	} __attribute__ ((packed));
        const static int SEND_RANGE_REQUEST_SIZE = sizeof(SEND_RANGE_REQUEST_t);
	
	#define SEND_RANGE_CONFIRM_MSGTYPE 0x0103

	struct SEND_RANGE_CONFIRM_t
	{
	    uint32_t status;
	    uint16_t msgID;
	    uint16_t msgType;
	    uint16_t length;
	    uint16_t sync;
	} __attribute__ ((packed));
        const static int SEND_RANGE_CONFIRM_SIZE = sizeof(SEND_RANGE_CONFIRM_t);
	

	struct PRE_READ_t
	{
	    uint16_t msgType;
	    uint16_t length;
	    uint16_t sync;
	} __attribute__ ((packed));
        const static int PRE_READ_SIZE = sizeof(PRE_READ_t);
	
	union Pre_Response_t
	{
	    PRE_READ_t msg;
            uint8_t msgData[PRE_READ_SIZE];
            char msgDataChar[PRE_READ_SIZE];
    };
	
	union Range_Request_t
	{
	    SEND_RANGE_REQUEST_t msg;
            uint8_t msgData[SEND_RANGE_REQUEST_SIZE];
            char msgDataChar[SEND_RANGE_REQUEST_SIZE];
	};
	
	union Ranging_Radio_Data_t
	{
	    RCM_RANGE_INFO_t msg;
            uint8_t msgData[RCM_RANGE_INFO_SIZE]; //add a few bytes, just in case.
            char msgDataChar[RCM_RANGE_INFO_SIZE]; //add a few bytes, just in case.
            boost::array<uint8_t, RCM_RANGE_INFO_SIZE> msgDataBoost;
	};
	
	union Range_Request_Confirm_t
	{
	    SEND_RANGE_CONFIRM_t msg;
            uint8_t msgData[SEND_RANGE_CONFIRM_SIZE];
            char msgDataChar[SEND_RANGE_CONFIRM_SIZE];
        };
    
    static inline void copyData(const uint8_t* dataToCopy, uint8_t* placeToPutIt, const int& size)
	{
	    for(int i = 0; i < size; i++)
	    {
	        placeToPutIt[i] = dataToCopy[i];
	    }
	};
		
        static inline void littleToBigEndian(const uint8_t* littleEndianData, uint8_t* outputBigEndianData, const int& size)
	{
	    for(int i = 0; i < size; i++)
	    {
	        outputBigEndianData[i] = littleEndianData[size-1-i];
        }
    };
    
    static inline void bigToLittleEndian(const uint8_t* bigEndianData, uint8_t* outputLittleEndianData, const int& size)
	{
	    for(int i = 0; i < size; i++)
	    {
	        outputLittleEndianData[i] = bigEndianData[size-1-i];
        }
    };
    
}

