#include <hw_interface_plugin_dynamixel/hw_interface_plugin_dynamixel_serial.hpp>

//int shit = 5;
hw_interface_plugin_dynamixel::dynamixel_serial::dynamixel_serial()
{
    //A debug message
    //ROS_INFO("val ptr %p", &shit);
    ROS_INFO_EXTRA_NAME("A Wild Dynamixel Plugin Appeared!%s"," ");

    //force the ROS Console level to Debug Level
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info) ) {
           ros::console::notifyLoggerLevelsChanged();
        }

    //enable automatic class metric collection.
    enableMetrics();

    currentState = DISCOVERY;
    testpkt.header = 0xffff;
    testpkt.id = 0;
    testpkt.instruction = 0x01;
    testpkt.length = testpkt.parameters.size()+2;
    responseTimeout = RESPONSE_TIMEOUT_MILLI;
    sentCommV2Ping=false;
    dynamixel_types::comm_version_t commVDiscovery = dynamixel_types::COMM_V1;
}

void hw_interface_plugin_dynamixel::dynamixel_serial::updateServoCallback(const hw_interface_plugin_dynamixel::servoUpdate::ConstPtr &msg)
{
    for(dynamixel_types::dynamixel_id_number_t i = 0; i < servoDirectory.size(); i++)
    {
        if((servoDirectory[i])->getID() == msg->id)
        {
            if(!(servoDirectory[i])->queryActuatorAbilitySupport((dynamixel_types::dynamixel_abilities_t)msg->abilityRequested))
            {
                ROS_ERROR_EXTRA_NAME("Servo ID: %u does not support requested Ability!", (servoDirectory[i])->getID());
                break;
            }
            if(msg->mode == msg->MODE_QUERY)
            {
                boost::lock_guard<boost::mutex> guard(queryQueueMtx);
                queryQueue.emplace(*msg, i); //constructs request at back of queue
                ROS_DEBUG_EXTRA_NAME("Query Queued, Queue Size: %lu", queryQueue.size());
            }
            else if(!(msg->mode == dynamixel_types::MODE_QUERY))
            {
                boost::lock_guard<boost::mutex> guard(requestQueueMtx);
                requestQueue.emplace(*msg, i); //constructs request at back of queue
                ROS_DEBUG_EXTRA_NAME("Request Queued, Queue Size: %lu", requestQueue.size());
            }
            break;
        }
        if( msg->id < (servoDirectory[i])->getID())
        {
            ROS_ERROR_EXTRA_NAME("Servo ID: %u not found!", msg->id);
            break;
        }
    }
}

void hw_interface_plugin_dynamixel::dynamixel_serial::groupUpdateServoCallback(const hw_interface_plugin_dynamixel::groupServoUpdate::ConstPtr &msg)
{
    for(dynamixel_types::dynamixel_id_number_t i = 0; (i < servoDirectory.size()); i++)
    {
        for(size_t j = 0; j < msg->updateVector.size(); j++)
        {
            ROS_DEBUG_EXTRA_NAME("msg length %lu dir size %lu vector pos %lu dir pos%u", msg->updateVector.size(), servoDirectory.size(), j,i);
            if((servoDirectory[i])->getID() == msg->updateVector[j].id)
            {
                if(!(servoDirectory[i])->queryActuatorAbilitySupport((dynamixel_types::dynamixel_abilities_t)msg->updateVector[j].abilityRequested))
                {
                    ROS_ERROR_EXTRA_NAME("Servo ID: %u does not support requested Ability!", i);
                    break;
                }
                else if(msg->updateVector[j].mode == dynamixel_types::MODE_QUERY)
                {
                    boost::lock_guard<boost::mutex> guard(queryQueueMtx);
                    queryQueue.emplace(msg->updateVector[j], i); //constructs request at back of queue
                    ROS_DEBUG_EXTRA_NAME("Query Queued, Queue Size: %lu", queryQueue.size());
                    break;
                }
                else
                {
                    boost::lock_guard<boost::mutex> guard(requestQueueMtx);
                    requestQueue.emplace(msg->updateVector[j], i); //constructs request at back of queue
                    ROS_DEBUG_EXTRA_NAME("Request Queued, Queue Size: %lu  %lu", requestQueue.size(), j);
                    break;
                }
                //continue;
            }
        }
    }
}

bool hw_interface_plugin_dynamixel::dynamixel_serial::subPluginInit(ros::NodeHandlePtr nhPtr)
{
    ROS_DEBUG_EXTRA_NAME("%s Plugin Init", pluginName.c_str());

    /*for Serial interfaces, 'deviceName' is an inherited member and must be defined.
        failing to define this variable will disable the plugin.
        opening of the device port is handled automatically */

    deviceName = "";
    ros::param::get(pluginName+"/deviceName", deviceName);

    //retrieve string from the ROS Parameter Server
        //of the format '<plugin_name>/<parameter>
    std::string tempString = "";

    rosDataSubRequest = nhPtr->subscribe("/servo/updateRequest", 20, &dynamixel_serial::updateServoCallback, this);
    rosDataSubGroupRequest = nhPtr->subscribe("/servo/updateGroupRequest", 20, &dynamixel_serial::groupUpdateServoCallback, this);

    if(!ros::param::get(pluginName+"/maxScanID", scanMaxId))
    {
        scanMaxId = 255;
    }

    //place to publish the data after reading from device
    if(ros::param::get(pluginName+"/publishToTopic", tempString))
    {
        //rosDataPub = nhPtr->advertise<hw_interface_plugin_timedomain::RCM_Range_Info>(tempString, 1, false);
    }
    else
    {
        ROS_ERROR_EXTRA_NAME("%s:: Could not find topic advertisment name", pluginName.c_str());
    }

    if(!(ros::param::get(pluginName+"/responseTimeout", responseTimeout)))
    {
        ROS_WARN_EXTRA_NAME("Using deafult response timeout%s"," ");
    }


    //the char buffer which contains the header must be valid for every call of the stream matcher
        //so we store the header string in the inherited member and its guarenteed to be valid until
        //destruction
    headerString = std::string({(char)0xff,(char)0xff,(char)0xfd,(char)0x00});

    //bind a functor to the streamCompletionChecker inherited member
    streamCompletionChecker = boost::bind(&hw_interface_plugin_dynamixel::dynamixel_serial::dynamixelStreamMatcher,
                                            this, _1, _2, headerString.c_str(), 3);
    //enable the completion functor if the bind succeeded
    enableCompletionFunctor = !streamCompletionChecker.empty();


    return true;
}

void hw_interface_plugin_dynamixel::dynamixel_serial::setInterfaceOptions()
{
    int tempBaudRate = 0;
    ros::param::get(pluginName+"/baudrate", tempBaudRate);
    setOption<boost::asio::serial_port_base::baud_rate>(
                new boost::asio::serial_port_base::baud_rate(tempBaudRate));
    //8 bits per character
    setOption<boost::asio::serial_port_base::character_size>(
                new boost::asio::serial_port_base::character_size( 8 ));

    //flow control
    setOption<boost::asio::serial_port_base::flow_control>(
                new boost::asio::serial_port_base::flow_control(
                                            boost::asio::serial_port_base::flow_control::type::none));

    //parity
    setOption<boost::asio::serial_port_base::parity>(
                new boost::asio::serial_port_base::parity(
                                            boost::asio::serial_port_base::parity::type::none));

    //stop-bits
    setOption<boost::asio::serial_port_base::stop_bits>(
                new boost::asio::serial_port_base::stop_bits(
                                            boost::asio::serial_port_base::stop_bits::type::one));

    ROS_INFO_EXTRA_NAME("%s :: Device: %s :: Baudrate %d", pluginName.c_str(), deviceName.c_str(), tempBaudRate);
}

//on timeout, cancel the read. do something about the error...
void hw_interface_plugin_dynamixel::dynamixel_serial::timeoutHandler(const boost::system::error_code& error)
{
    if(error != boost::asio::error::operation_aborted)
    {
        ROS_DEBUG_EXTRA_NAME("Response Timed out %s"," ");
        interfacePort->cancel();
        return;
    }
    ROS_DEBUG_EXTRA_NAME("Timer Canceled%s"," ");
}



bool hw_interface_plugin_dynamixel::dynamixel_serial::interfaceReadHandler(const size_t &length, int arrayStartPos,
                                                                                const boost::system::error_code &ec)
{
    bool ret = false;
    //cancel the timeout

    ROS_DEBUG_EXTRA_NAME("Cancelling pending timers %lu",servoResponseTimer->cancel());

    if (ec != boost::asio::error::operation_aborted)
    {
        ret = true;
        ROS_DEBUG_EXTRA_NAME("Response length %lu, startPos %d", length, arrayStartPos);
//        std::printf("Buffer Contents: ");
//        for(int i = arrayStartPos; i < length; i++)
//        {
//            std::printf("%x || ",receivedData[i]);
//        }
//        std::printf("\r\n");

        retryJump:

        /*else*/ if((currentState == DISCOVERY))
        {
            if(!regReadSent || retry )
            {

                if(commVDiscovery == dynamixel_types::COMM_V1)
                {
                    ROS_DEBUG_EXTRA_NAME("Found servo with ID: %d", testpkt.id);
                    ROS_DEBUG_EXTRA_NAME("Attempting reg read for Model Number%s"," ");

                    dynamixel_types::comm_v1_packet_t regRead; //discovery happens
                    regRead.header = COMM_V1_16_BIT_HEADER;
                    regRead.id=testpkt.id;
                    regRead.instruction = dynamixel_types::INT_READ_DATA;
                    regRead.parameters.push_back(0x00); //read starting at address 0
                    regRead.parameters.push_back(INIT_READ_SIZE);    //read 8 bytes
                    regRead.length=regRead.parameters.size()+2;
                    dynamixel_types::const_shared_buf_dynamixel test(regRead);
                    postInterfaceWriteRequest(test);
                    regReadSent=true;
                    continueDiscovery = false;
                    retry = false;
                }
            }
            else
            {
                retry = false;
                regReadSent=false; //in this case, we sent a reg read and we are now reading its response
                continueDiscovery = true;

                if(commVDiscovery == dynamixel_types::COMM_V1)
                {
                    dynamixel_types::comm_v1_status_packet_t test(&receivedData[arrayStartPos]);
                    ROS_DEBUG_EXTRA_NAME("Servo <%d> Responded to Reg Read for model num", test.id);
                    ROS_DEBUG_EXTRA_NAME("Servo <%d> :: Model Num %u | %x", test.id, *((uint16_t*) &test.parameters.data()[0]),*((uint16_t*) &test.parameters.data()[0]));

                    ROS_DEBUG_EXTRA_NAME("Instantiating new servo%s"," ");
                    boost::shared_ptr<dynamixel_servo> abstractServo = dynamixel_factory::createServo(test,dynamixel_types::COMM_V1);
                    if(abstractServo.get())
                    {
                        ROS_DEBUG_EXTRA_NAME("Servo Instantiation sucessful.%s"," ");
                        ROS_DEBUG_EXTRA_NAME("Adding new dynamixel_servo to directory%s"," ");
                        servoDirectory.push_back(abstractServo);
                        ROS_DEBUG_EXTRA_NAME("%s", abstractServo->to_string().c_str());
                    }
                    else
                    {
                        ROS_ERROR_EXTRA_NAME("Instantiation failed%s"," ");
                    }
                } 
                else
                {
                    regReadSent=true;
                    ROS_DEBUG("COMM V2 Init");
                    dynamixel_types::comm_v2_status_packet_t test(&receivedData[arrayStartPos]);
                    ROS_DEBUG_EXTRA_NAME("Servo <%d> Responded to Reg Read for model num", test.id);
                    ROS_DEBUG_EXTRA_NAME("Servo <%d> :: Model Num %u | %x", test.id, *((uint16_t*) &test.parameters.data()[0]),*((uint16_t*) &test.parameters.data()[0]));

                    ROS_DEBUG_EXTRA_NAME("Instantiating new servo%s"," ");
                    boost::shared_ptr<dynamixel_servo> abstractServo = dynamixel_factory::createServo(test,dynamixel_types::COMM_V2);
                    if(abstractServo.get())
                    {
                        ROS_DEBUG_EXTRA_NAME("Servo Instantiation sucessful.%s"," ");
                        ROS_DEBUG_EXTRA_NAME("Adding new dynamixel_servo to directory%s"," ");
                        servoDirectory.push_back(abstractServo);
                        ROS_DEBUG_EXTRA_NAME("%s", abstractServo->to_string().c_str());
                    }
                    else
                    {
                        ROS_ERROR_EXTRA_NAME("Instantiation failed%s"," ");
                    }

                }
            }
        }

    }
    else
    {
        ROS_DEBUG_EXTRA_NAME("Read was cancelled! Most likely due to a failed response within time%s"," ");
        ret = true;
        if(regReadSent)
        {
            ROS_WARN_EXTRA_NAME("Retrying Last Command%s"," ");
            retry = true;
            goto retryJump;
        }
        else
        {
            continueDiscovery=true;
            regReadSent=false;
            retry = false;
        }
    }
    if(currentState != EXECUTION && testpkt.id > (uint8_t)scanMaxId)
    {
        if(commVDiscovery == dynamixel_types::COMM_V2 && (ec == boost::asio::error::operation_aborted))
        {
            if(!(ros::param::get(pluginName+"/responseTimeout", responseTimeout)))
            {
                ROS_WARN_EXTRA_NAME("Using deafult response timeout%s"," ");
            }
            currentState=EXECUTION;
            ROS_INFO_EXTRA_NAME("Moving to Execution%s"," ");
            continueDiscovery=false;
            regReadSent=false;

            ROS_INFO_EXTRA_NAME("\r\nDiscovered Actuators\r\n%s"," ");
            for(std::vector<boost::shared_ptr<dynamixel_servo> >::iterator it = servoDirectory.begin(); it != servoDirectory.end(); it++)
            {
                ROS_INFO_EXTRA_NAME("%s", (*it)->to_string().c_str());
                dynamixel_types::dynamixel_ability_request_t test(dynamixel_types::MODE_SET, dynamixel_types::ABILITY_STATUS_RETURN_LEVEL, 1, dynamixel_types::INT_WRITE_DATA);
                (*it)->abilityRequest(test);
                //ROS_INFO_EXTRA_NAME("Request Result: %d", (int)test.requestResult);
                if(test.requestResult == dynamixel_types::REQUEST_SUCCESS)
                {
                    ROS_INFO_EXTRA_NAME("Setting Status Return Level%s"," ");
                    if((*it)->getCommVersion() == dynamixel_types::COMM_V2)
                    {
                        dynamixel_types::comm_v2_packet_t outputTest;
                        outputTest.id = (*it)->getID();
                        outputTest.instruction=test.instruction;
                        (*it)->requestOutputUpdate(test, outputTest.parameters);
                        outputTest.length=outputTest.parameters.size()+3;
                        dynamixel_types::const_shared_buf_dynamixel testOutBuf(outputTest);
                        postInterfaceWriteRequest(testOutBuf);
                    }
                    else
                    {
                        dynamixel_types::comm_v1_packet_t outputTest;
                        outputTest.id = (*it)->getID();
                        outputTest.instruction=test.instruction;
                        (*it)->requestOutputUpdate(test, outputTest.parameters);
                        outputTest.length=outputTest.parameters.size()+2;
                        dynamixel_types::const_shared_buf_dynamixel testOutBuf(outputTest);
                        postInterfaceWriteRequest(testOutBuf);
                    }
                }
            }
        }
        else
        {
            //discover V2
            ROS_INFO("Discovering on Version V2");

            if(!sentCommV2Ping)
            {
                responseTimeout=2000;
                regReadSent=true;retry = false; continueDiscovery=false;
                commVDiscovery=dynamixel_types::COMM_V2;
                dynamixel_types::comm_v2_packet_t output;
                continueDiscovery=false;
                output.id=0xFE;
                output.instruction=dynamixel_types::INT_PING;
                output.length=output.parameters.size()+3;
                dynamixel_types::const_shared_buf_dynamixel outputBuf(output);
                postInterfaceWriteRequest(outputBuf);
            }
            sentCommV2Ping=true;
        }
    }
    else if((currentState == DISCOVERY) && continueDiscovery)
    {
        testpkt.id++;
        ROS_INFO_EXTRA_NAME("Searching for ID %d with COMM_V1", testpkt.id);
        dynamixel_types::const_shared_buf_dynamixel test(testpkt);// test.init(testpkt);
        postInterfaceWriteRequest(test);

        continueDiscovery=false;
    }

    if((currentState == EXECUTION))
    {
        bool expectingResponse = false;
        while(!expectingResponse)
        {
            ROS_DEBUG_EXTRA_NAME("exec pop%s"," ");
            //if last cmd failed
                //retry last cmd?

            //pass status to servo here
            if (ec != boost::asio::error::operation_aborted && lastSent)
            {
                ROS_DEBUG("attempting update");
                if(lastComm == dynamixel_types::COMM_V1)
                {
                    ROS_DEBUG("Updating servo from response V1");
                    dynamixel_types::comm_v1_status_packet_t testUpdateS(&receivedData[arrayStartPos]);
                    (servoDirectory[lastSent->vectorLocation])->updateFromActuatorResponse(testUpdateS);
                }
                else
                {
                    dynamixel_types::comm_v2_status_packet_t testUpdateS(&receivedData[arrayStartPos]);
                    ROS_DEBUG("Updating servo from response V2 %d", testUpdateS.length);
                    (servoDirectory[lastSent->vectorLocation])->updateFromActuatorResponse(testUpdateS);
                }
            }


            boost::lock_guard<boost::mutex> guardR(requestQueueMtx);
            boost::lock_guard<boost::mutex> guardQ(queryQueueMtx);


            ROS_DEBUG_EXTRA_NAME("Query Size %lu", queryQueue.size());
            ROS_DEBUG_EXTRA_NAME("Request Size %lu", requestQueue.size());
            if(lastSent)
            {
                if(lastSent->mode == dynamixel_types::MODE_QUERY)
                {
                    ROS_DEBUG_EXTRA_NAME("query pop%s"," ");
                    queryQueue.push(queryQueue.front());
                    queryQueue.pop();
                }
                else
                {
                    ROS_DEBUG_EXTRA_NAME("request pop%s"," ");
                    requestQueue.pop();
                }
                lastSent=0;
            }

            ROS_DEBUG_EXTRA_NAME("after pop%s"," ");
            //if bulk read
              //check if that was the last response expected
              //if last servo, send next cmd
                //else return

            //if cmds are available && last cmd was query
                //lock respective queue
                //pop
                //move front to current
                //set last cmd came from cmd queue

            if(requestQueue.size() && ((!queryQueue.size())||((lastMode & dynamixel_types::MODE_NOP)||(!((lastMode & (dynamixel_types::MODE_GET|dynamixel_types::MODE_SET))!=0)))))
            {
                (servoDirectory[requestQueue.front().vectorLocation])->abilityRequest(requestQueue.front());
                if(requestQueue.front().requestResult == dynamixel_types::REQUEST_SUCCESS)
                {
                    ROS_DEBUG_EXTRA_NAME("request is ready to send%s"," ");
                    lastSent = &requestQueue.front();
                    lastSent->instruction = (lastSent->mode == dynamixel_types::MODE_SET) ?
                                                dynamixel_types::INT_WRITE_DATA :
                                                dynamixel_types::INT_READ_DATA;
                    lastMode=lastSent->mode;
                    lastComm=(servoDirectory[requestQueue.front().vectorLocation])->getCommVersion();
                }
                else
                {
                    ROS_ERROR_EXTRA_NAME("Ability set failed! ability %s failed with code %d", dynamixel_helpers::abilityToString(requestQueue.front().abilityRequested).c_str(), (int)requestQueue.front().requestResult);
                }
            }
            else if(queryQueue.size() && ((!requestQueue.size())||(lastMode & dynamixel_types::MODE_NOP)||(!((lastMode & (dynamixel_types::MODE_QUERY))!=0))))
            {
                (servoDirectory[queryQueue.front().vectorLocation])->abilityRequest(queryQueue.front());
                if(queryQueue.front().requestResult == dynamixel_types::REQUEST_SUCCESS)
                {
                    ROS_DEBUG_EXTRA_NAME("query is ready to send%s"," ");
                    lastSent = &queryQueue.front();
                    lastSent->instruction = dynamixel_types::INT_READ_DATA;
                    lastMode=lastSent->mode;
                    lastComm=(servoDirectory[queryQueue.front().vectorLocation])->getCommVersion();
                }
                else
                {
                    ROS_ERROR_EXTRA_NAME("Ability set failed! ability %s failed with code %d", dynamixel_helpers::abilityToString(queryQueue.front().abilityRequested).c_str(), (int)queryQueue.front().requestResult);
                }
            }

            //update servo with ability
            //run next ability
            //return
            if(lastSent)
            {
                if((servoDirectory[lastSent->vectorLocation])->getCommVersion()==dynamixel_types::COMM_V1)
                {
                    ROS_DEBUG_EXTRA_NAME("sending%s"," comm v1");
                    dynamixel_types::comm_v1_packet_t output;
                    output.id = lastSent->id;
                    lastSent->instruction=lastSent->instruction;
                    output.instruction=(uint8_t)lastSent->instruction;
                    ROS_DEBUG_EXTRA_NAME("Parameters size %lu", output.parameters.size());
                    (servoDirectory[lastSent->vectorLocation])->requestOutputUpdate(*lastSent, output.parameters);
                    ROS_DEBUG_EXTRA_NAME("Parameters size  %lu", output.parameters.size());
                    output.length=output.parameters.size()+2;
                    dynamixel_types::const_shared_buf_dynamixel outBuf(output);
                    postInterfaceWriteRequest(outBuf);
                }
                else
                {
                    ROS_DEBUG_EXTRA_NAME("sending%s"," comm v2");
                    dynamixel_types::comm_v2_packet_t output;
                    output.id = lastSent->id;
                    lastSent->instruction=lastSent->instruction;
                    output.instruction=(uint8_t)lastSent->instruction;
                    ROS_DEBUG_EXTRA_NAME("Parameters size %lu", output.parameters.size());
                    (servoDirectory[lastSent->vectorLocation])->requestOutputUpdate(*lastSent, output.parameters);
                    ROS_DEBUG_EXTRA_NAME("Parameters size  %lu", output.parameters.size());
                    output.length=output.parameters.size()+3;
                    dynamixel_types::const_shared_buf_dynamixel outBuf(output);
                    postInterfaceWriteRequest(outBuf);
                }
            }
            if(((lastSent)&&(lastSent->instruction == dynamixel_types::INT_READ_DATA))||(!requestQueue.size()))
            {
                ROS_DEBUG_EXTRA_NAME("Expecting Response%s"," ");
                expectingResponse = true;
            }
        }


    }

    servoResponseTimer->expires_from_now(boost::posix_time::milliseconds(responseTimeout));
    servoResponseTimer->async_wait(boost::bind(&hw_interface_plugin_dynamixel::dynamixel_serial::timeoutHandler, this, boost::asio::placeholders::error()));

    return ret;
}

bool hw_interface_plugin_dynamixel::dynamixel_serial::verifyChecksum()
{
    return true;
}

bool hw_interface_plugin_dynamixel::dynamixel_serial::pluginStart()
{
    testpkt.header = 0xffff;
    testpkt.id = 0;
    testpkt.instruction = 0x01;
    testpkt.length = testpkt.parameters.size()+2;
    ROS_INFO_EXTRA_NAME("Starting Discovery on ID %d", testpkt.id);

    dynamixel_types::const_shared_buf_dynamixel test(testpkt);
    postInterfaceWriteRequest(test);

    servoResponseTimer.reset(new boost::asio::deadline_timer(interfacePort->get_io_service(),
                                                                boost::posix_time::milliseconds(responseTimeout)));
    servoResponseTimer->expires_from_now(boost::posix_time::milliseconds(responseTimeout));
    servoResponseTimer->async_wait(boost::bind(&hw_interface_plugin_dynamixel::dynamixel_serial::timeoutHandler, this, boost::asio::placeholders::error()));

    return true;
}

bool hw_interface_plugin_dynamixel::dynamixel_serial::pluginStop()
{
    return true;
}



std::size_t hw_interface_plugin_dynamixel::dynamixel_serial::dynamixelStreamMatcher(
                        const boost::system::error_code &error, long totalBytesInBuffer,
                        const char *header, int headerLength)
{
    ROS_DEBUG_EXTRA_NAME("Entered Stream Matcher %ld",totalBytesInBuffer);
//    for(long i = 0; i < totalBytesInBuffer; i++)
//    {
//        std::printf("%x %c || ",receivedData[i],receivedData[i]);
//    }
//    std::printf("\r\n");
    long headerCounter=0;
    dataArrayStart=0;
    dynamixel_types::comm_version_t commVersion = dynamixel_types::COMM_V2;
    if(error != boost::asio::error::operation_aborted)
    {
        if(totalBytesInBuffer < 4)
        {
            ROS_DEBUG_EXTRA_NAME("Returning Early, %ld", 4-totalBytesInBuffer);
            return 4-totalBytesInBuffer;
        }
        for(long i = 0; i<totalBytesInBuffer; i++)
        {
            if(headerCounter != headerLength) //change this
            {
                if((receivedData[i]&0xff) == (header[headerCounter]&0xff))
                {
                    headerCounter++;
                    if(headerCounter==headerLength)
                    {
                        ROS_DEBUG("mathcer:: COMM V2");
                        dataArrayStart= i - (headerLength - 1); // change 2 to headerLength
                    }
                }
                else
                {
                    if(headerCounter==2) //change this
                    {
                        ROS_DEBUG("mathcer:: COMM V1");
                        headerLength==2;
                        commVersion=dynamixel_types::COMM_V1;
                        dataArrayStart= i - (headerLength - 1); // change 2 to headerLength
                        ROS_DEBUG_EXTRA_NAME("Continuing %ld", i);
                        continue;
                    }
                    ROS_DEBUG_EXTRA_NAME("Didn't match header %ld", i);
                    headerCounter=0;
                    headerLength==3;
                    commVersion=dynamixel_types::COMM_V2;
                }
            }
            else
            {
                if(commVersion==dynamixel_types::COMM_V1)
                {
                    if((totalBytesInBuffer - dataArrayStart) >= 4) //if we have enough data to get length
                    {
                        ROS_DEBUG_EXTRA_NAME("Returning , %ld , %d , %u , %ld", totalBytesInBuffer, dataArrayStart, receivedData[dataArrayStart+3], (dataArrayStart + receivedData[dataArrayStart+3] + 4 - totalBytesInBuffer));
                        return (dataArrayStart + receivedData[dataArrayStart+3] + 4 - totalBytesInBuffer);
                    }
                    else
                    {
                        //ROS_DEBUG_EXTRA_NAME("Not Enough Data, Returning , %ld , %d , %ld", totalBytesInBuffer, dataArrayStart, (totalBytesInBuffer-dataArrayStart));
                        return 2;
                    }
                }
                else
                {
                    if((totalBytesInBuffer - dataArrayStart) >= 7) //if we have enough data to get length
                    {
                        ROS_DEBUG_EXTRA_NAME("Returning , %ld , %d , %u , %ld", totalBytesInBuffer, dataArrayStart,*((uint16_t*)&receivedData[dataArrayStart+5]) , (dataArrayStart + *((uint16_t*)&receivedData[dataArrayStart+5]) + 7 - totalBytesInBuffer));
                        return (dataArrayStart + *((uint16_t*)&receivedData[dataArrayStart+5]) + 7 - totalBytesInBuffer);
                    }
                    else
                    {
                        //ROS_DEBUG_EXTRA_NAME("Not Enough Data, Returning , %ld , %d , %ld", totalBytesInBuffer, dataArrayStart, (totalBytesInBuffer-dataArrayStart));
                        return 2;
                    }
                }
            }
        }
        ROS_WARN_EXTRA_NAME("Stream Matching Fell Through! Read Length = %ld", totalBytesInBuffer);
        for(long i = 0; i < totalBytesInBuffer; i++)
        {
            std::printf("%x %c || ",receivedData[i],receivedData[i]);
        }
        std::printf("\r\n");
        return 2;
    }
    else
    {
        ROS_DEBUG_EXTRA_NAME("Stream Matching Op Aborted.%s"," ");
        return 0; //operation aborted
    }
}
