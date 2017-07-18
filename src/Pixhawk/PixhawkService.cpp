
// include header for this service
#include "PixhawkService.h"
#include "uxas/messages/uxnative/AutopilotKeepAlive.h"
#include "uxas/messages/uxnative/StartupComplete.h"
#include "afrl/cmasi/MissionCommand.h"
#include "afrl/cmasi/GoToWaypointAction.h"

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
    /*mavlink_message_t r_message;
    mavlink_status_t r_mavlink_status;
    char newc = 0;
    uint8_t res = mavlink_parse_char(0,newc, &r_message, &r_mavlink_status);*/
    
    
    
    COUT_INFO("PixhawkService called");
    std::memset(&m_Attitude,0,sizeof(m_Attitude));
}

PixhawkService::~PixhawkService() { }
bool PixhawkService::configure(const pugi::xml_node& ndComponent)
{
    bool isSuccess(true);
    COUT_INFO("PX Configure");

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
    addSubscriptionAddress(uxas::messages::uxnative::StartupComplete::Subscription);
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
    m_SafetyTimerId = uxas::common::TimerManager::getInstance().createTimer(
        std::bind(&PixhawkService::SafetyTimer, this), "PixhawkService::SafetyTimer");
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
    COUT_INFO("PX start");
    m_receiveFromPixhawkProcessingThread = uxas::stduxas::make_unique<std::thread>(&PixhawkService::executePixhawkAutopilotCommProcessing, this);
    //return true;
    return (uxas::common::TimerManager::getInstance().startPeriodicTimer(m_SafetyTimerId,0,m_sendPeriod_ms));
};

bool PixhawkService::terminate()
{
    // kill the timer
    uint64_t delayTime_ms{1000};
    if (m_SafetyTimerId && !uxas::common::TimerManager::getInstance().destroyTimer(m_SafetyTimerId, delayTime_ms))
    {
        //UXAS_LOG_WARN(s_typeName(), "::HelloWorld::terminate() failed to destroy message send timer ",
         //        "with timer ID ", m_sendMessageTimerId, " within ", delayTime_ms, " millisecond timeout");
    }
    COUT_INFO("PX terminate");

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
        COUT_INFO("PX ::terminate calling thread completed m_receiveFromPixhawkProcessingThread join");
    }
    else
    {
        COUT_INFO("PX::terminate unexpectedly could not join m_receiveFromPiccoloProcessingThread");
    }
    return (true);
}

bool PixhawkService::processReceivedLmcpMessage(std::unique_ptr<uxas::communications::data::LmcpMessage> receivedLmcpMessage)
{
    COUT_INFO("LMCP ");//<< receivedLmcpMessage->m_object->getLmcpTypeName());
    if (afrl::cmasi::isKeyValuePair(receivedLmcpMessage->m_object))
    {
        //receive message
        //auto keyValuePairIn = std::static_pointer_cast<afrl::cmasi::KeyValuePair> (receivedLmcpMessage->m_object);
        //std::cout << "*** RECEIVED:: Received Id[" << m_serviceId << "] Sent Id[" << keyValuePairIn->getKey() << "] Message[" << keyValuePairIn->getValue() << "] *** " << std::endl;
    }
    else if(uxas::messages::uxnative::StartupComplete::TypeName == receivedLmcpMessage->m_object->getLmcpTypeName())
    {
        COUT_INFO("Startup Complete");
        m_bStartupComplete=true;
    }
    else if (afrl::cmasi::isVehicleActionCommand(receivedLmcpMessage->m_object))
    {
        //COUT_INFO("VehicleActionCommand");
        auto actionCmd = std::static_pointer_cast<afrl::cmasi::VehicleActionCommand>(receivedLmcpMessage->m_object);
        std::vector<afrl::cmasi::VehicleAction*> va = actionCmd->getVehicleActionList();
        //should be one only?
        for(int i=0;i<(int)va.size();i++)
        {
            afrl::cmasi::VehicleAction* thisva = va[i];
            //contains the WP # from list
            if (thisva->getLmcpType() == afrl::cmasi::CMASIEnum::GOTOWAYPOINTACTION)
            {
                COUT_INFO("Go To Waypoint Action");

                afrl::cmasi::GoToWaypointAction* goToAction = static_cast<afrl::cmasi::GoToWaypointAction*> (thisva);
                std::shared_ptr<avtas::lmcp::Object> goToActionObj(goToAction->clone());
                //HandleGoToAction(goToActionObj);
                int32_t wp_num_command = -1;
                int64_t wp_num_to_find = goToAction->getWaypointNumber();
                COUT_INFO("New WP to find# "<<wp_num_to_find);

                for(int ii=0;ii<(int)m_newWaypointList.size();ii++)
                {

                    if(m_newWaypointList[ii]->getNumber() == wp_num_to_find)
                    {
                        wp_num_command = ii;
                        COUT_INFO("New WP# "<<wp_num_to_find << "PX WP# "<<wp_num_command);
                        break;
                    }          
                }
                //todo set active WP here
            }   
            else if (thisva->getLmcpType() == afrl::cmasi::CMASIEnum::LOITERACTION)
            {
                //afrl::cmasi::LoiterAction* loiterAction = static_cast<afrl::cmasi::LoiterAction*> (action);
                //std::shared_ptr<avtas::lmcp::Object> loiterActionObj(loiterAction->clone());
                //HandleLoiterAction(loiterActionObj);
                COUT_INFO("Loiter Action");

            }
            else
                COUT_INFO("Unhandled Vechile Action Command");
        }
    }
    else if (afrl::cmasi::isMissionCommand(receivedLmcpMessage->m_object))
    {
        COUT_INFO("Mission Command: " << receivedLmcpMessage->m_object->getLmcpTypeName());
        auto missionCmd = std::static_pointer_cast<afrl::cmasi::MissionCommand> (receivedLmcpMessage->m_object);
        if (missionCmd)
        {
            //if (missionCmd->getVehicleID() != m_ui16AutopilotID)
            //    return;

            //DEBUG
            
            //m_newMissionCommand = missionCmd;
            int num_wp = missionCmd->getWaypointList().size();
            
            //std::vector<afrl::cmasi::Waypoint*> testlist = missionCmd->getWaypointList();
            //auto wpList = missionCmd->getWaypointList();
            //m_newWaypointList = std::move(wpList);
            std::cout << "HandleMissionCommand with size " << missionCmd->getWaypointList().size() << std::endl;
            afrl::cmasi::Waypoint* wp;
            m_newWaypointList.clear();
            
            //todo slip home position + takeoff command (for quads)
            //PX4 in quad mode ignores the first two if not home + takeoff
            
            //home = current position
            //takeoff at home position
            
            for (int i = 0; i < (int)missionCmd->getWaypointList().size(); i++)
            {
                wp = missionCmd->getWaypointList().at(i);
                /*std::cout << "waypoint[" << i << "]:" << std::endl;
                std::cout << "CMASI ID: " << (int) wp->getNumber() << std::endl;
                std::cout << "Next ID: " << (int) wp->getNextWaypoint() << std::endl;
                std::cout << "lat: " << wp->getLatitude() << std::endl;
                std::cout << "lon: " << wp->getLongitude() << std::endl;*/
                
                std::shared_ptr<afrl::cmasi::Waypoint> newWP(new afrl::cmasi::Waypoint);
                newWP->setLatitude(wp->getLatitude());
                newWP->setLongitude(wp->getLongitude());
                newWP->setAltitude(wp->getAltitude());
                newWP->setNumber(wp->getNumber());
                m_newWaypointList.push_back(newWP);
            }
            wp = NULL;
            //std::cout << "Start at C#" << (int) missionCmd->getFirstWaypoint() << std::endl;   
            
            //start the waypoint update process to Pixhawk
            this->MissionUpdate_ClearAutopilotWaypoints();//MissionUpdate_SendNewWayPointCount();

        }
    }
    /*else if (afrl::cmasi::isAutomationResponse(receivedLmcpMessage->m_object))//isAutomationResponse(receivedLmcpMessage->m_object))
    {
        auto automationResponse = std::static_pointer_cast<afrl::cmasi::AutomationResponse> (receivedLmcpMessage->m_object);
        if (automationResponse && automationResponse->getMissionCommandList().empty())
        {
            for (auto& missionCommand : automationResponse->getMissionCommandList())
            {
                std::shared_ptr<avtas::lmcp::Object> obj(missionCommand->clone());
                COUT_INFO("Mission: " << obj->getLmcpTypeName());
                //HandleMissionCommand(obj);
                break;
            }
        }
    }*/
    else
        COUT_INFO("Type: " << receivedLmcpMessage->m_object->getLmcpTypeName());
    return false;
}
void PixhawkService::SafetyTimer()
{
    {
        std::lock_guard<std::mutex> lock(m_AirvehicleStateMutex);
        if(bAVSReady)
        {
            afrl::cmasi::Location3D* where = m_ptr_CurrentAirVehicleState->getLocation();
            double lat = where->getLatitude();
            double lon = where->getLongitude();
            auto alt = where->getAltitude();
            std::cout.precision(8);
            //COUT_INFO("Pos (lat,lon,alt): " << lat << ", " << lon << ", " << alt);
            //sendSharedLmcpObjectBroadcastMessage(m_ptr_CurrentAirVehicleState);
            bAVSReady=false;
            m_ptr_CurrentAirVehicleState.reset(new afrl::cmasi::AirVehicleState());
        }
    }
    static int call_count=0;
    int rport = m_remoteSocket.sin_port;
    /*if(m_bStartupComplete && call_count == 0 && rport != 0)
    {
        //uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
        //uint8_t target_system, uint8_t target_component, uint16_t count, uint8_t mission_type
        //mavlink_msg_mission_count_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
        //                       uint8_t target_system, uint8_t target_component, uint16_t count, uint8_t mission_type)
        uint8_t system_id=255;
        uint8_t component_id=MAV_COMP_ID_MISSIONPLANNER;
        mavlink_message_t msg;
        uint8_t buf[MAVLINK_MAX_PACKET_LEN];

        uint8_t target_system=1;
        uint8_t target_component=0;
        uint8_t mission_type=0;
        mavlink_msg_mission_request_list_pack(system_id, component_id, &msg,
                               target_system, target_component, mission_type);
        uint16_t slen = mavlink_msg_to_send_buffer(buf, &msg);
        struct sockaddr_in servaddr;    
        memset((char*)&servaddr, 0, sizeof(servaddr));      
        servaddr.sin_family = AF_INET;
        servaddr.sin_port = htons(m_netPort);
        ssize_t send_len = sendto(m_netSocketFD, buf, sizeof(buf), 0, (struct sockaddr *) &m_remoteSocket, sizeof(m_remoteSocket));
        if (send_len == -1)
        {
            COUT_INFO("bad send " << send_len);
        }
        else
        {
            COUT_INFO("Req List sent " << send_len);
        }
        call_count++;
        
    }
    else if(m_bStartupComplete && rport == 0 && call_count==0)
        COUT_INFO("rport = 0");*/
    /*mavlink_system_t mavlink_system;
 
    mavlink_system.sysid = 255;                   ///
    mavlink_system.compid = MAV_COMP_ID_IMU;     ///
    uint8_t     system_type=MAV_TYPE_QUADROTOR;
    uint8_t     autopilot_type=MAV_AUTOPILOT_GENERIC;
    uint8_t     system_mode=MAV_MODE_PREFLIGHT;
    uint32_t    custom_mode=0;
    uint8_t     system_state=MAV_STATE_STANDBY;

    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    mavlink_msg_heartbeat_pack(mavlink_system.sysid, mavlink_system.compid, &msg, system_type, autopilot_type, system_mode, custom_mode, system_state);
    uint16_t slen = mavlink_msg_to_send_buffer(buf, &msg);
    uint16_t send_len = sendto(m_netSocketFD, buf, sizeof(buf), 0, (struct sockaddr *) &m_remoteSocket, slen);
    if (send_len == -1)
    {
        COUT_INFO("bad send " << send_len);
    }
    else
    {

    }*/
    //else
    //    COUT_INFO("Failed mAVS lock");
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
            m_isTerminate=true;
            return;
        }
        else
            COUT_INFO("bind good");
        memset((char *) &m_remoteSocket, 0, sizeof(m_remoteSocket));
    }
    else
    {
        //m_tcpConnectionSocket->connect(m_tcpAddress.c_str());
    }
    uint8_t buf[1024];
    int32_t recv_len=0;
    static uint64_t packets_recv = 0;
    while (!m_isTerminate)
    {
        std::string strInputFromPiccolo;
        strInputFromPiccolo.clear();
        if (m_useNetConnection)
        {                 
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
                if(packets_recv == 0)
                    COUT_INFO("RPORT: " << m_remoteSocket.sin_port);
                packets_recv++;
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
        for(int32_t i=0;i<recv_len;i++)
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
                        //std::cout << "HB " << (uint16_t) heartbeat.autopilot << " - " << (uint16_t) heartbeat.mavlink_version << std::endl;
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
                        //m_AirvehicleStateMutex.lock();
                        {
                            std::lock_guard<std::mutex> lock(m_AirvehicleStateMutex);

                            if(m_Attitude.time_boot_ms!=0)
                            {
                                m_ptr_CurrentAirVehicleState->setPitch(m_Attitude.pitch);
                                m_ptr_CurrentAirVehicleState->setRoll(m_Attitude.roll);
                            }
                            m_ptr_CurrentAirVehicleState->setID(400);
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

                            // heading, pitch, roll
                            //m_ptr_CurrentAirVehicleState->setHeading((telemetryPkt->GetYaw() / 10000.0) * RAD_TO_DEG); // heading = yaw
                            //m_ptr_CurrentAirVehicleState->setPitch((telemetryPkt->GetPitch() / 10000.0) * RAD_TO_DEG);
                            //m_ptr_CurrentAirVehicleState->setRoll((telemetryPkt->GetRoll() / 10000.0) * RAD_TO_DEG);

                            // yaw, pitch, roll rates
                            //m_ptr_CurrentAirVehicleState->setR((telemetryPkt->GetYawRate() / 10000.0) * RAD_TO_DEG); // from (1/10,000)r/s -> rad/s
                            //m_ptr_CurrentAirVehicleState->setQ((telemetryPkt->GetPitchRate() / 10000.0) * RAD_TO_DEG);
                            //m_ptr_CurrentAirVehicleState->setP((telemetryPkt->GetRollRate() / 10000.0) * RAD_TO_DEG);
                             // ActualEnergyRate, EnergyAvailable
                            //m_ptr_CurrentAirVehicleState->setActualEnergyRate(0.0);
                            //m_ptr_CurrentAirVehicleState->setEnergyAvailable(0.0);
                            
                            // wind speed/heading
                            //m_ptr_CurrentAirVehicleState->setWindSpeed(telemetryPkt->GetWindSpeed()); // m/s
                            //m_ptr_CurrentAirVehicleState->setWindDirection(telemetryPkt->GetWindFromDirection()); // degrees from north (wind is coming from)

                            // ground speed, ground track
                            //m_ptr_CurrentAirVehicleState->setGroundspeed(telemetryPkt->GetGroundSpeed()); // m/s
                            
                            //float vEast_mps = static_cast<float> (telemetryPkt->GetVEast()) / 100.0;
                            //float vNorth_mps = static_cast<float> (telemetryPkt->GetVNorth()) / 100.0;
                            //float vDown_mps = static_cast<float> (telemetryPkt->GetVDown()) / 100.0;
                            //m_ptr_CurrentAirVehicleState->setCourse(atan2(vEast_mps, vNorth_mps) * RAD_TO_DEG);
                            
                            
                            m_ptr_CurrentAirVehicleState->setCurrentWaypoint(m_CurrentWaypoint);
                            //sendSharedLmcpObjectBroadcastMessage(m_ptr_CurrentAirVehicleState);
                            bAVSReady=true;
                            //COUT_INFO("Broadcasting AVS");
                        }
                        //else
                        //    COUT_INFO("Failed mAVS lock");
                        //m_AirvehicleStateMutex.unlock();
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
                        //COUT_INFO("GPS RAW INT");
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
                    case MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT:
                    {
                        //COUT_INFO("TARGET GLOBAL INT");
                        break;
                    }
                    case MAVLINK_MSG_ID_STATUSTEXT:
                    {
                        break;
                    }
                    case MAVLINK_MSG_ID_COMMAND_LONG:
                    {
                        break;
                    }
                    case MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED:
                    case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW:
                        break;
                    case MAVLINK_MSG_ID_MISSION_COUNT:
                        mavlink_mission_count_t mcount;
                        mavlink_msg_mission_count_decode(&msg,&mcount);
                        COUT_INFO("RX COUNT " << mcount.count);
                        
                        //m_recvMissionCount=mcount.count;//only need if we are going to read waypoint list
                        break;
                    case MAVLINK_MSG_ID_MISSION_ACK:
                    {
                        mavlink_mission_ack_t mack;
                        mavlink_msg_mission_ack_decode(&msg,&mack);
                        if(m_missionSendState == SENT_CLEAR)
                        {
                            COUT_INFO("Finished clear (ack), sending count");
                            MissionUpdate_SendNewWayPointCount();
                        }
                        else if(m_missionSendState == SENT_LAST_WAYPOINT)
                        {
                            COUT_INFO("Finished waypoint write, got ACK");
                            //Set active waypoint here
                            MissionUpdate_SetActiveWaypoint(0);
                        }
                        break;
                    }
                    case MAVLINK_MSG_ID_MISSION_REQUEST:
                    {
                        if(m_missionSendState == SENT_COUNT)
                        {
                            //send first waypoint in my list
                            COUT_INFO("Rq: Sending 1st WP # " << m_wpIterator);
                            MissionUpdate_SendWayPoint();

                        }
                        else if(m_missionSendState == SENT_WAYPOINT && m_wpIterator < m_newWaypointCount)
                        {
                            //send next waypoint
                            this->m_wpIterator++;
                            COUT_INFO("Rq: Sending next WP # " << m_wpIterator);
                            MissionUpdate_SendWayPoint();
                        }
                        else if(m_missionSendState == SENT_ACTIVE_WAYPOINT)
                        {
                            COUT_INFO("Mission send done");
                        }
                        else
                        {
                            COUT_INFO("MISSION ACK ERROR");
                        }
                        break;
                    }
                    /*case MAVLINK_MSG_ID_MISSION_ITEM_REQUEST:
                    {
                        mavlink_mission_item_request_t mreq;
                        avlink_mission_item_request_decode(&msg,&mreq);
                        COUT_INFO("MISSION ITEM REQUEST");
                        break;
                    }*/
                    default:
                    {
                        COUT_INFO("####################################### Msg:"<<msg.msgid);
                        break;
                    }
                    //#85
                    //#36
                }
            }         
        }
    }
    
}
void PixhawkService::MissionUpdate_ClearAutopilotWaypoints(void)
{
    m_newWaypointCount = m_newWaypointList.size();   
    if(m_newWaypointCount <= 0)
    {
        COUT_INFO("Error, not clearing");
        return;
    }
    else
        COUT_INFO("Clearing px4 waypoints");
    
    uint8_t system_id=255;
    uint8_t component_id=MAV_COMP_ID_MISSIONPLANNER;
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    uint8_t target_system=1;
    uint8_t target_component=0;
    uint8_t mission_type=0;

    //uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
    //                           uint8_t target_system, uint8_t target_component, uint8_t mission_type
    mavlink_msg_mission_clear_all_pack(system_id,component_id,&msg,
                                        target_system,target_component,mission_type);
    uint16_t slen = mavlink_msg_to_send_buffer(buf, &msg);
    ssize_t send_len = sendto(m_netSocketFD, buf, sizeof(buf), 0, (struct sockaddr *) &m_remoteSocket, sizeof(m_remoteSocket));
    if (send_len == -1)
    {
        COUT_INFO("clear waypoint send error");
    }
    else
    {
        COUT_INFO("Cleared waypoint command sent");
        m_wpIterator = 0;
        m_missionSendState = SENT_CLEAR;
    }
}
void PixhawkService::MissionUpdate_SendNewWayPointCount(void)
{                
    m_newWaypointCount = m_newWaypointList.size();   
    if(m_newWaypointCount <= 0)
    {
        COUT_INFO("Error, no waypoints to send");
        return;
    }
    else
        COUT_INFO("Sending " << m_newWaypointCount << " waypoints");
    uint8_t system_id=255;
    uint8_t component_id=MAV_COMP_ID_MISSIONPLANNER;
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    uint8_t target_system=1;
    uint8_t target_component=0;
    uint8_t mission_type=0;
    //uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
    //                           uint8_t target_system, uint8_t target_component, uint16_t count, uint8_t mission_type
    mavlink_msg_mission_count_pack(system_id, component_id, &msg,
                           target_system, target_component, this->m_newWaypointCount,mission_type);
    uint16_t slen = mavlink_msg_to_send_buffer(buf, &msg);
    ssize_t send_len = sendto(m_netSocketFD, buf, sizeof(buf), 0, (struct sockaddr *) &m_remoteSocket, sizeof(m_remoteSocket));
    if (send_len == -1)
    {
        COUT_INFO("bad send count " << send_len);
    }
    else
    {
        COUT_INFO("New count sent (" << send_len << ") " << m_newWaypointCount);
        m_wpIterator = 0;
        m_missionSendState = SENT_COUNT;
    }
}
void PixhawkService::MissionUpdate_SendWayPoint(void)
{
    auto wp = m_newWaypointList[m_wpIterator];
    //afrl::cmasi::Waypoint* wp = m_newWaypointList[m_wpIterator];
    uint8_t     system_id=255;
    uint8_t     component_id=MAV_COMP_ID_MISSIONPLANNER;
    mavlink_message_t msg;
    uint8_t     target_system=1;
    uint8_t     target_component=0; 
    uint16_t    seq=m_wpIterator;
    uint8_t     frame=MAV_FRAME_GLOBAL_RELATIVE_ALT;
    uint16_t    command=MAV_CMD_NAV_WAYPOINT;
    uint8_t     current=0; 
    uint8_t     autocontinue=1;
    float   param1=0.0f;
    float   param2=0.0f;
    float   param3=0.0f; 
    float   param4=0.0f; 
    float   x=wp->getLatitude();
    float   y=wp->getLongitude(); 
    float   z=wp->getAltitude();
    uint8_t mission_type=0;
    
    m_missionSendState = SENT_WAYPOINT;

    if(m_wpIterator == 0)
    {
        command = 22;
        current = 1;
        param1 = 15.0;
        COUT_INFO("About to send 1st WP");
    }
    else if(m_wpIterator == this->m_newWaypointCount-1)
    {
        //frame = MAV_FRAME_MISSION;
        m_missionSendState = SENT_LAST_WAYPOINT;
        COUT_INFO("About to send LAST WP");

    }
    else 
    {
        COUT_INFO("About to send n WP");
    }
    //(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
    //                           uint8_t target_system, uint8_t target_component, 
    //        uint16_t seq, uint8_t frame, uint16_t command, uint8_t current, 
    //        uint8_t autocontinue, float param1, float param2, float param3, 
    //        float param4, float x, float y, float z, uint8_t mission_type)
    mavlink_msg_mission_item_pack(system_id,component_id,&msg,target_system,
            target_component,seq,frame,command,current,autocontinue,
            param1,param2,param3,param4,x,y,z,mission_type);
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t slen = mavlink_msg_to_send_buffer(buf, &msg);
    ssize_t send_len = sendto(m_netSocketFD, buf, sizeof(buf), 0, (struct sockaddr *) &m_remoteSocket, sizeof(m_remoteSocket));
    if (send_len == -1)
    {
        COUT_INFO("bad WP sent " << send_len);
    }
    else
    {
        COUT_INFO("New WP sent " << send_len << " (x,y,z) " << x << ", " << y << ", " << z);
    }
}
void PixhawkService::MissionUpdate_SetActiveWaypoint(uint32_t newWP_px)
{
    m_newWaypointCount = m_newWaypointList.size();   
    if(m_newWaypointCount <= 0)
    {
        COUT_INFO("Error, not clearing");
        return;
    }
    else
        COUT_INFO("Clearing px4 waypoints");
    
    uint8_t system_id=255;
    uint8_t component_id=MAV_COMP_ID_MISSIONPLANNER;
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    uint8_t target_system=1;
    uint8_t target_component=0;

    //uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
    //                           uint8_t target_system, uint8_t target_component, uint16_t seq
    mavlink_msg_mission_set_current_pack(system_id,component_id,&msg,
                                        target_system,target_component,newWP_px);
    uint16_t slen = mavlink_msg_to_send_buffer(buf, &msg);
    ssize_t send_len = sendto(m_netSocketFD, buf, sizeof(buf), 0, (struct sockaddr *) &m_remoteSocket, sizeof(m_remoteSocket));
    if (send_len == -1)
    {
        COUT_INFO("Active waypoint send error");
    }
    else
    {
        COUT_INFO("Active waypoint command sent");
        m_wpIterator = 0;
        m_missionSendState = SENT_ACTIVE_WAYPOINT;
    }
}

};};
