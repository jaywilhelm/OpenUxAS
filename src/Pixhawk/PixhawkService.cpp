
// include header for this service
#include "PixhawkService.h"
#include "uxas/messages/uxnative/AutopilotKeepAlive.h"


#warning "Building with Pixhawk"
#define COUT_INFO(MESSAGE) std::cout << "PX: " << MESSAGE << std::endl;std::cout.flush();

// namespace definitions
namespace uxas
{
namespace service
{
PixhawkService::ServiceBase::CreationRegistrar<PixhawkService>
PixhawkService::s_registrar(PixhawkService::s_registryServiceTypeNames());// service constructor
// service constructor
PixhawkService::PixhawkService() : ServiceBase(PixhawkService::s_typeName(), PixhawkService::s_directoryName()) 
{ 
    //Test the MAVLink interface
    mavlink_message_t r_message;
    mavlink_status_t r_mavlink_status;
    char newc = 0;
    uint8_t res = mavlink_parse_char(0,newc, &r_message, &r_mavlink_status);
    
    
    std::cout << "PixhawkService called";
    std::memset(&m_Attitude,0,sizeof(m_Attitude));
}

PixhawkService::~PixhawkService() { }
bool PixhawkService::configure(const pugi::xml_node& ndComponent)
{
    bool isSuccess(true);
    std::cout <<  "PX Configure"<<std::endl;

    // process options from the XML configuration node:
    /*if (!ndComponent.attribute(STRING_XML_STRING_TO_SEND).empty())
    {
        m_stringToSend = ndComponent.attribute(STRING_XML_STRING_TO_SEND).value();
    }
    if (!ndComponent.attribute(STRING_XML_SEND_PERIOD_MS).empty())
    {
        m_sendPeriod_ms = ndComponent.attribute(STRING_XML_SEND_PERIOD_MS).as_int64();
    }

    // subscribe to messages::
    addSubscriptionAddress(afrl::cmasi::KeyValuePair::Subscription);*/
    ////////////////////////////////////////////////////////
    // subscribe to messages
    ////////////////////////////////////////////////////////
    addSubscriptionAddress(afrl::cmasi::MissionCommand::Subscription);
    addSubscriptionAddress(uxas::messages::uxnative::AutopilotKeepAlive::Subscription);
    addSubscriptionAddress(afrl::cmasi::VehicleActionCommand::Subscription);
    //addSubscriptionAddress(afrl::cmasi::GimbalStareAction::Subscription);
    //addSubscriptionAddress(afrl::cmasi::GimbalAngleAction::Subscription);
    //addSubscriptionAddress(afrl::cmasi::CameraAction::Subscription);
    {
        std::lock_guard<std::mutex> lock(m_AirvehicleStateMutex);
        m_ptr_CurrentAirVehicleState.reset(new afrl::cmasi::AirVehicleState());
    }
    //sendSharedLmcpObjectBroadcastMessage(m_ptr_CurrentAirVehicleState);
    return (isSuccess);
}

bool PixhawkService::initialize()
{
    // create send timer
    //m_sendMessageTimerId = uxas::common::TimerManager::getInstance().createTimer(
    //    std::bind(&HelloWorld::OnSendMessage, this), "HelloWorld::OnSendMessage");
    bool bSuccess(true);

    std::cout <<  "PX init"<<std::endl;

    if (m_useNetConnection)
    {
        // open tcp/ip socket for sending/receiving messages
        //assert(!m_tcpAddress.empty());
        //std::cout << "PX Connecting to " << m_tcpAddress << std::endl;
        if ((m_netSocketFD=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
        {
            bSuccess=false;
            COUT_INFO("create socket failed");
        }
        else
            COUT_INFO("socket created");

    }
    else
    {
        COUT_INFO("serial");
        // 0) initialize the serial connection
        m_serialConnectionPixhawk.reset(new serial::Serial(m_strTTyDevice, m_ui32Baudrate, serial::Timeout::simpleTimeout(m_serialTimeout_ms)));
        if (!m_serialConnectionPixhawk->isOpen())
        {
            //UXAS_LOG_ERROR(s_typeName(), ":: Initialize - serial connection failed:: m_strTTyDevice[", m_strTTyDevice, "m_ui32Baudrate[", m_ui32Baudrate);
            COUT_INFO("Serial open failed:"<<m_strTTyDevice);
            bSuccess = false;
        }
    }
    return (bSuccess);
}

bool PixhawkService::start()
{
    std::cout <<  "PX start"<<std::endl;
    m_receiveFromPixhawkProcessingThread = uxas::stduxas::make_unique<std::thread>(&PixhawkService::executePixhawkAutopilotCommProcessing, this);
    return (true);
    // start the timer
    return true;
    //return (uxas::common::TimerManager::getInstance().startPeriodicTimer(m_sendMessageTimerId,0,m_sendPeriod_ms));
};

bool PixhawkService::terminate()
{
    // kill the timer
    //uint64_t delayTime_ms{1000};
    //if (m_sendMessageTimerId && !uxas::common::TimerManager::getInstance().destroyTimer(m_sendMessageTimerId, delayTime_ms))
    {
        //UXAS_LOG_WARN(s_typeName(), "::HelloWorld::terminate() failed to destroy message send timer ",
         //        "with timer ID ", m_sendMessageTimerId, " within ", delayTime_ms, " millisecond timeout");
    }
    std::cout <<  "PX terminate"<<std::endl;

    if (m_useNetConnection)
    {
        uint32_t ui32LingerTime(0);
        //m_tcpConnectionSocket->setsockopt(ZMQ_LINGER, &ui32LingerTime, sizeof (ui32LingerTime));
        //m_tcpConnectionSocket->close();
        // make sure the file is closed
    }
    
    m_isTerminate = true;
    if (m_receiveFromPixhawkProcessingThread && m_receiveFromPixhawkProcessingThread->joinable())
    {
        m_receiveFromPixhawkProcessingThread->join();
        std::cout <<  "PX ::terminate calling thread completed m_receiveFromPixhawkProcessingThread join"<<std::endl;
    }
    else
    {
        std::cout << "PX::terminate unexpectedly could not join m_receiveFromPiccoloProcessingThread"<<std::endl;
    }
    return (true);
}

bool PixhawkService::processReceivedLmcpMessage(std::unique_ptr<uxas::communications::data::LmcpMessage> receivedLmcpMessage)
{
    COUT_INFO("LMCP " << receivedLmcpMessage->m_object->getLmcpTypeName());
    if (afrl::cmasi::isKeyValuePair(receivedLmcpMessage->m_object))
    {
        //receive message
        //auto keyValuePairIn = std::static_pointer_cast<afrl::cmasi::KeyValuePair> (receivedLmcpMessage->m_object);
        //std::cout << "*** RECEIVED:: Received Id[" << m_serviceId << "] Sent Id[" << keyValuePairIn->getKey() << "] Message[" << keyValuePairIn->getValue() << "] *** " << std::endl;
    }
    return false;
}

void
PixhawkService::executePixhawkAutopilotCommProcessing()
{
    COUT_INFO("executePixhawkAutopilotCommProcessing");
    std::string strInputFromPixhawk;
    if (m_bServer)
    {
        // zero out the structure
        memset((char *) &m_listenSocket, 0, sizeof(m_listenSocket));

        m_listenSocket.sin_family = AF_INET;
        m_listenSocket.sin_port = htons(m_netPort);
        m_listenSocket.sin_addr.s_addr = htonl(INADDR_ANY);
        COUT_INFO("binding to port " << m_netPort);

        //bind socket to port
        if(bind(m_netSocketFD , (struct sockaddr*)&m_listenSocket, sizeof(m_listenSocket) ) == -1)
        {
            COUT_INFO("bind failed");
        }
        else
            COUT_INFO("bind good");
    }
    else
    {
        //m_tcpConnectionSocket->connect(m_tcpAddress.c_str());
    }
    uint8_t buf[1024];
    int recv_len=0;

    while (!m_isTerminate)
    {
        std::string strInputFromPiccolo;
        strInputFromPiccolo.clear();
        if (m_useNetConnection)
        {                 
            memset((char *) &m_remoteSocket, 0, sizeof(m_remoteSocket));
            socklen_t slen = sizeof(m_remoteSocket);
            recv_len=0;
            //int ret = recv(m_netSocketFD,buf,sizeof(buf), 0);//non-blocking, also will drop bytes if packet is smaller than buffer
            //(ret == 0)

            recv_len = recvfrom(m_netSocketFD, buf, sizeof(buf), 0, (struct sockaddr *) &m_remoteSocket, &slen);
            if (recv_len == -1)
            {
                COUT_INFO("bad recv " << recv_len);
            }
            else
            {
                
            } 
        }
        else
        {
            assert(m_serialConnectionPixhawk);
            int read_ret = m_serialConnectionPixhawk->read(buf,sizeof(buf));
            if(read_ret < 0)
            {
                COUT_INFO("Serial read error:"<<read_ret);
            }
            else
                COUT_INFO("Serial read length:" << read_ret);

            //assert(m_serialConnectionPiccolo);
            //strInputFromPiccolo = m_serialConnectionPiccolo->read(m_serialReadSize);
            //UXAS_LOG_DEBUG_VERBOSE("PiccoloAutopilotAdapterService::executePiccoloAutopilotSerialProcessing", " bytes on serial port: ", strInputFromPiccolo.length());
            //UXAS_LOG_DEBUG_VERBOSE("PiccoloAutopilotAdapterService::executePiccoloAutopilotSerialProcessing", " all bytes read from serial port: ", strInputFromPiccolo);
        }
        if(recv_len <= 0)
            continue;
        
        buf[recv_len]=0;
        //COUT_INFO("recv " << buf << " " << recv_len);
        int chan = 0;
        mavlink_message_t msg;
        mavlink_status_t status;
        //std::cout << std::hex << (uint16_t) 0xFD << std::endl;
        for(uint32_t i=0;i<recv_len;i++)
        {
            uint8_t mvp_ret = mavlink_parse_char(chan,buf[i],&msg,&status);
            uint16_t data = 0x00FF & buf[i];
            //std::cout << "p_ret " << std::hex << data << std::endl;
            if(mvp_ret != 0)
            {
                switch (msg.msgid)
                {
                    case MAVLINK_MSG_ID_HEARTBEAT:
                    {
                        mavlink_heartbeat_t heartbeat;
                        mavlink_msg_heartbeat_decode(&msg, &heartbeat);
                        std::cout << "HB " << (uint16_t) heartbeat.autopilot << " - " << (uint16_t) heartbeat.mavlink_version << std::endl;
                        break;
                    }
                    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
                    {
                        mavlink_global_position_int_t gpsi;
                        mavlink_msg_global_position_int_decode(&msg,&gpsi);
                        float newAlt_m = (float)gpsi.alt/1000.0f;//AMSL
                        float cog_d = (float)gpsi.hdg/1000.0f;//deg
                        double lat_d = (double)gpsi.lat/10000000.0;//deg
                        double lon_d = (double)gpsi.lon/10000000.0;//deg

                        //COUT_INFO("GLOBAL POS INT " << newAlt_m); 
                        if(m_AirvehicleStateMutex.try_lock())
                        {
                            if(m_Attitude.time_boot_ms!=0)
                            {
                                m_ptr_CurrentAirVehicleState->setPitch(m_Attitude.pitch);
                                m_ptr_CurrentAirVehicleState->setRoll(m_Attitude.roll);
                            }
                            m_ptr_CurrentAirVehicleState->setAirspeed(m_Airspeed);//m/s
                            
                            m_ptr_CurrentAirVehicleState->getLocation()->setAltitudeType(afrl::cmasi::AltitudeType::MSL);
                            m_ptr_CurrentAirVehicleState->getLocation()->setAltitude(newAlt_m);
                            m_ptr_CurrentAirVehicleState->getLocation()->setLatitude(lat_d);
                            m_ptr_CurrentAirVehicleState->getLocation()->setLongitude(lon_d);
                            m_ptr_CurrentAirVehicleState->setCourse(cog_d);
                            // u, v, w, udot, vdot, wdot
                            m_ptr_CurrentAirVehicleState->setU(0.0);
                            m_ptr_CurrentAirVehicleState->setV(0.0);
                            m_ptr_CurrentAirVehicleState->setW(0.0);
                            m_ptr_CurrentAirVehicleState->setUdot(0.0);
                            m_ptr_CurrentAirVehicleState->setVdot(0.0);
                            m_ptr_CurrentAirVehicleState->setWdot(0.0);
                             // ActualEnergyRate, EnergyAvailable
                            m_ptr_CurrentAirVehicleState->setActualEnergyRate(0.0);
                            m_ptr_CurrentAirVehicleState->setEnergyAvailable(0.0);
                            
                            m_ptr_CurrentAirVehicleState->setCurrentWaypoint(m_CurrentWaypoint);
                            sendSharedLmcpObjectBroadcastMessage(m_ptr_CurrentAirVehicleState);
                        }
                        break;
                    }
                    case MAVLINK_MSG_ID_MISSION_CURRENT:
                    {
                        mavlink_mission_current_t mcur;
                        mavlink_msg_mission_current_decode(&msg,&mcur);
                        //COUT_INFO("Mission Curr: "<<mcur.seq);
                        m_CurrentWaypoint=mcur.seq;
                        break;
                    }
                    case MAVLINK_MSG_ID_VFR_HUD:
                    {
                        mavlink_vfr_hud_t vfr;
                        mavlink_msg_vfr_hud_decode(&msg,&vfr);
                        //COUT_INFO("Speed VFR "<<vfr.airspeed);
                        //m_Airspeed = vfr.airspeed;
                        m_Airspeed = vfr.groundspeed;
                        #warning "Using ground speed"
                        break;
                    }
                    case MAVLINK_MSG_ID_WIND_COV://#231
                    {
                        break;
                    }
                    case MAVLINK_MSG_ID_ALTITUDE:
                    {
                        break;
                    }
                    case MAVLINK_MSG_ID_ATTITUDE: //#30
                    {
                        mavlink_attitude_t att;
                        mavlink_msg_attitude_decode(&msg,&att);
                        memcpy(&m_Attitude,&att,sizeof(m_Attitude));
                        //pitch roll yaw
                        break;
                    }
                    case MAVLINK_MSG_ID_SYS_STATUS://#1
                    {
                        break;
                    }
                    case MAVLINK_MSG_ID_SYSTEM_TIME://#2
                    {
                        break;
                    }
                    case MAVLINK_MSG_ID_BATTERY_STATUS://#147
                    {
                        break;
                    }
                    case MAVLINK_MSG_ID_EXTENDED_SYS_STATE://#245
                    {
                        break;
                    }
                    case MAVLINK_MSG_ID_ATTITUDE_QUATERNION:
                    {
                        break;
                    }
                    case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
                    {
                        break;
                    }
                    case MAVLINK_MSG_ID_ATTITUDE_TARGET:
                    {
                        //Reports the current commanded attitude of the vehicle as specified by the autopilot. This should match the commands sent in a SET_ATTITUDE_TARGET message if the vehicle is being controlled this way.
                        break;
                    }
                    case MAVLINK_MSG_ID_PARAM_VALUE:
                    {
                        
                    }
                    case MAVLINK_MSG_ID_HIGHRES_IMU://#105
                    {
                        mavlink_highres_imu_t highimu;
                        mavlink_msg_highres_imu_decode(&msg,&highimu);
                        //COUT_INFO("HighResIMU");
                        break;
                    }
                    case MAVLINK_MSG_ID_GPS_RAW_INT://#24
                    {
                        mavlink_gps_raw_int_t rgpsint;
                        mavlink_msg_gps_raw_int_decode(&msg,&rgpsint);
                        //COUT_INFO("HighResIMU");
                        break;
                    }
                    case MAVLINK_MSG_ID_ESTIMATOR_STATUS://#230
                    {
                        break;
                    }
                    case MAVLINK_MSG_ID_HOME_POSITION://#242
                    {
                        break;
                    }
                    case MAVLINK_MSG_ID_VIBRATION://#241
                    {
                        break;
                    }
                    case MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED:
                    case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW:
                        break;
                    default:
                    {
                        COUT_INFO("Msg:"<<msg.msgid);
                        break;
                    }
                    //#85
                    //#36
                }
            }         
        }
    }
};



};};