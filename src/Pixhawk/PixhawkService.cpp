#include <fstream>      // std::ifstream

// include header for this service
#include "PixhawkService.h"

#define STRING_XML_LISTEN_PORT_MAVLINK  "MAVLinkListenPort"
#define STRING_XML_VEHICLE_ID           "VehicleIDToWatch"
//#define USE_MISSION_INT
#define MinWPDistCheck 16000.0

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
    COUT_INFO("PixhawkService called");
    std::memset(&m_Attitude,0,sizeof(m_Attitude));
    std::memset(&m_SavedHomePositionMsg,0,sizeof(m_SavedHomePositionMsg));
    COUT_INFO("PixhawkService ready");
    COUT_INFO("Make sure cfg xml has \"<Service Type=\"PixhawkService\" />\"");
    //    <Service Type="PixhawkService" />
}

PixhawkService::~PixhawkService() { }
bool PixhawkService::configure(const pugi::xml_node& ndComponent)
{
    COUT_INFO("PX Configure");

    bool isSuccess(true);

    // process options from the XML configuration node:
    if (!ndComponent.attribute(STRING_XML_LISTEN_PORT_MAVLINK).empty())
    {
        m_configListenPortMavlink = ndComponent.attribute(STRING_XML_LISTEN_PORT_MAVLINK).as_uint();
        COUT_INFO("XML Port: " << m_configListenPortMavlink)
    }
    else
    {
        COUT_INFO("USING default port " << this->m_netPort);
    }

    if (!ndComponent.attribute(STRING_XML_VEHICLE_ID).empty())
    {
        m_VehicleIDtoWatch = ndComponent.attribute(STRING_XML_VEHICLE_ID).as_uint();
        COUT_INFO("Vehicle ID to watch: " << m_VehicleIDtoWatch)

    }
    
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
    
    COUT_INFO("PX Configure done");

    return (isSuccess);
}

bool PixhawkService::initialize()
{
    COUT_INFO("PX initialize");

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
    }
    else
    {
        /*COUT_INFO("serial");
        // 0) initialize the serial connection
        m_serialConnectionPixhawk.reset(new serial::Serial(m_strTTyDevice, m_ui32Baudrate, serial::Timeout::simpleTimeout(m_serialTimeout_ms)));
        if (!m_serialConnectionPixhawk->isOpen())
        {
            //UXAS_LOG_ERROR(s_typeName(), ":: Initialize - serial connection failed:: m_strTTyDevice[", m_strTTyDevice, "m_ui32Baudrate[", m_ui32Baudrate);
            COUT_INFO("Serial open failed:"<<m_strTTyDevice);
            bSuccess = false;
        }*/
		COUT_INFO("ERROR NO SERIAL?");

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

    /*if (m_useNetConnection)
    {
        //uint32_t ui32LingerTime(0);
        //m_tcpConnectionSocket->setsockopt(ZMQ_LINGER, &ui32LingerTime, sizeof (ui32LingerTime));
        //m_tcpConnectionSocket->close();
        // make sure the file is closed
    }*/
    
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
        COUT_INFO("VehicleActionCommand");
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
        auto missionCmd = std::static_pointer_cast<afrl::cmasi::MissionCommand> (receivedLmcpMessage->m_object);
        Process_isMissionCommand(missionCmd);
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
void PixhawkService::Process_isMissionCommand(std::shared_ptr<afrl::cmasi::MissionCommand> missionCmd)
{
    //auto missionCmd = std::static_pointer_cast<afrl::cmasi::MissionCommand> (receivedLmcpMessage->m_object);
    COUT_INFO("Mission Command: " << missionCmd->getLmcpTypeName() << " ID: " << missionCmd->getVehicleID());
    if (missionCmd)
    {
        if (missionCmd->getVehicleID() != this->m_VehicleIDtoWatch)
            return;

        //DEBUG
        
        //m_newMissionCommand = missionCmd;
        int num_wp = missionCmd->getWaypointList().size();
        
        //std::vector<afrl::cmasi::Waypoint*> testlist = missionCmd->getWaypointList();
        //auto wpList = missionCmd->getWaypointList();
        //m_newWaypointList = std::move(wpList);
        std::cout << "HandleMissionCommand with size " << num_wp << std::endl;
        afrl::cmasi::Waypoint* wp;
        m_newWaypointList.clear();
        std::shared_ptr<afrl::cmasi::Waypoint> newTWP(new afrl::cmasi::Waypoint);

        bool saved_takeoff_pos = false;
        double lat,lon,alt;
        {
            std::lock_guard<std::mutex> lock(m_AirvehicleStateMutex);
            if(this->m_ptr_CurrentAirVehicleState && this->m_ptr_CurrentAirVehicleState->getLocation()
                    && this->m_ptr_CurrentAirVehicleState->getLocation()->getLatitude() != 0.0)
            {
                lat = this->m_ptr_CurrentAirVehicleState->getLocation()->getLatitude(); 
                lon = this->m_ptr_CurrentAirVehicleState->getLocation()->getLongitude();
                alt =  this->m_ptr_CurrentAirVehicleState->getLocation()->getAltitude();
                saved_takeoff_pos=true;
            }
        }
        
        if(saved_takeoff_pos)
        {
            newTWP->setLatitude(lat);
            newTWP->setLongitude(lon);
            newTWP->setAltitude(alt);
            newTWP->setNumber(0);
            m_newWaypointList.push_back(newTWP);
        }
        else
        {
            COUT_INFO("NO GLOBAL position for WP list");
        }
        
        //copy over new waypoints internally
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
            newWP->setNumber(wp->getNumber()+1);
            m_newWaypointList.push_back(newWP);
        }
        wp = NULL;
        COUT_INFO("Saved " << m_newWaypointList.size() << " new waypoints internally");
        //dump list
        /*for (int i = 0; i < (int)m_newWaypointList.size(); i++)
        {
            auto wp = m_newWaypointList[i];
            std::cout << "waypoint[" << i << "]:" << std::endl;
            std::cout << "CMASI ID: " << (int) wp->getNumber() << std::endl;
            std::cout << "Next ID: " << (int) wp->getNextWaypoint() << std::endl;
            std::cout << "lat: " << wp->getLatitude() << std::endl;
            std::cout << "lon: " << wp->getLongitude() << std::endl;
        }*/
        std::cout << "Start at C#" << (int) missionCmd->getFirstWaypoint() << std::endl;   
        
        //start the waypoint update process to Pixhawk
        if(saved_takeoff_pos)
        {
            m_missionStateLastSendCount=-1;
            this->MissionUpdate_ClearAutopilotWaypoints();
        }
        else
        {
            m_missionSendState = WAIT_GLOBAL_POSITION;
            COUT_INFO("Waiting on global position");
        }
    }
}
void PixhawkService::SafetyTimer()
{
    //COUT_INFO("Saftey timer tick")
    //COUT_INFO("TIME: " << uxas::common::Time::getInstance().getUtcTimeSinceEpoch_ms())

    if(m_missionSendState == this->SENT_CLEAR)
    {
        if(uxas::common::Time::getInstance().getUtcTimeSinceEpoch_ms() > m_missionStateLastSendTime + 5000)
        {
            m_missionStateLastSendCount++;
            if(m_missionStateLastSendCount > 3)
            {
                COUT_INFO("SafetyTimer caught waypoint clear not getting ack")
                COUT_INFO("PX4 is not responding to clear!!! ERROR")
                m_missionStateLastSendCount=-1;
                MissionUpdate_ClearAutopilotWaypoints();
            }
            else
            {
                COUT_INFO("Sending clear again from safety timer")
                MissionUpdate_ClearAutopilotWaypoints();
            }
        }
    }
    if(m_missionSendState == this->SENT_COUNT)
    {
        if(uxas::common::Time::getInstance().getUtcTimeSinceEpoch_ms() > m_missionStateLastSendTime + 5000)
        {
            m_missionStateLastSendCount++;
            if(m_missionStateLastSendCount > 3)
            {
                COUT_INFO("SafetyTimer caught waypoint count not getting ack")
                COUT_INFO("SafetyTimer starting wp process over")
                m_missionStateLastSendCount=-1;
                MissionUpdate_ClearAutopilotWaypoints();
                MavlinkDisconnect();
                MavlinkConnect();
            }
            else
            {
                COUT_INFO("Sending count again from safety timer")
                //Start the whole process over or retry?
                MissionUpdate_SendNewWayPointCount();
            }
        }
    }

    //meant to check the waypoints loaded on autopilot match uxas waypoints
    {
        /*std::lock_guard<std::mutex> lock(m_AirvehicleStateMutex);
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
        }*/
    }
    //static int call_count=0;
    //int rport = m_remoteSocket.sin_port;
    /*if(m_bStartupComplete && call_count == 0 && rport != 0)
    {
        //uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
        //uint8_t target_system, uint8_t target_component, uint16_t count, uint8_t mission_type
        //mavlink_msg_mission_count_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
        //                       uint8_t target_system, uint8_t target_component, uint16_t count, uint8_t mission_type)
        uint8_t system_id=m_MAVLinkID_UxAS;
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
    mavlink_system_t mavlink_system;
    mavlink_system.sysid    = 255;
    mavlink_system.compid   = MAV_COMP_ID_MISSIONPLANNER;
    uint8_t     system_type =MAV_TYPE_GENERIC;
    uint8_t     autopilot_type=MAV_AUTOPILOT_GENERIC;
    uint8_t     system_mode =MAV_MODE_PREFLIGHT;
    uint32_t    custom_mode =0;
    uint8_t     system_state=MAV_STATE_STANDBY;

    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    mavlink_msg_heartbeat_pack(mavlink_system.sysid, mavlink_system.compid, &msg, system_type, autopilot_type, system_mode, custom_mode, system_state);
    uint16_t slen = mavlink_msg_to_send_buffer(buf, &msg);
    uint16_t send_len = sendto(m_netSocketFD, buf, sizeof(buf), 0, (struct sockaddr *) &m_remoteSocket, slen);
    if (send_len == -1)
    {
        COUT_INFO("bad HB send " << send_len);
    }
    else
    {

    }
    //else
    //    COUT_INFO("Failed mAVS lock");
}
int PixhawkService::MavlinkConnect(void)
{
    int success=0; 
    if ((m_netSocketFD=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
    {
        success=-1;
        COUT_INFO("create socket failed");
        return success;
    }
    else
        COUT_INFO("socket created");
    // zero out the structure
    memset((char *) &m_listenSocket, 0, sizeof(m_listenSocket));

    m_listenSocket.sin_family = AF_INET;
    uint16_t listentPort = 0;
    if(m_configListenPortMavlink != 0)
        listentPort = m_configListenPortMavlink;
    else
        listentPort = m_netPort;
    m_listenSocket.sin_port = htons(listentPort);    
    m_listenSocket.sin_addr.s_addr = htonl(INADDR_ANY);
    COUT_INFO("binding to port " << listentPort);

    int enableopt = 1;
    if (setsockopt(m_netSocketFD, SOL_SOCKET, SO_REUSEADDR, &enableopt, sizeof(int)) < 0)
        COUT_INFO("setsockopt(SO_REUSEADDR) failed")

    //bind socket to port
    if(bind(m_netSocketFD , (struct sockaddr*)&m_listenSocket, sizeof(m_listenSocket) ) == -1)
    {
        COUT_INFO("bind failed");
        m_isTerminate=true;
        return -1;
    }
    else
        COUT_INFO("bind good");
        
    memset((char *) &m_remoteSocket, 0, sizeof(m_remoteSocket));
    return success;
}
int PixhawkService::MavlinkDisconnect()
{
    shutdown(m_netSocketFD, SHUT_RDWR);
    //memset((char *) &m_listenSocket, 0, sizeof(m_listenSocket));
}
void
PixhawkService::executePixhawkAutopilotCommProcessing()
{
    COUT_INFO("executePixhawkAutopilotCommProcessing");
    std::string strInputFromPixhawk;
    if (m_bServer)
    {
        MavlinkConnect();
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
        //std::string strInputFromPiccolo;
        //strInputFromPiccolo.clear();
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
            /*assert(m_serialConnectionPixhawk);
            int read_ret = m_serialConnectionPixhawk->read(buf,sizeof(buf));
            if(read_ret < 0)
            {
                COUT_INFO("Serial read error:"<<read_ret);
            }
            else
                COUT_INFO("Serial read length:" << read_ret);*/
			COUT_INFO("ERROR NO SERIAL SUPPORT?");

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
            //uint16_t data = 0x00FF & buf[i];
            //std::cout << "p_ret " << std::hex << data << std::endl;
            if(mvp_ret != 0)
            {
                //COUT_INFO(msg.msgid);
                if(msg.sysid != m_VehicleIDtoWatch)
                {
                    if(msg.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT)
                    {
                        //broadcast this to AirVehicleState
                        mavlink_global_position_int_t gpsi;
                        mavlink_msg_global_position_int_decode(&msg,&gpsi);
                        float newAlt_m = (float)gpsi.alt/1000.0f;//AMSL
                        float cog_d = (float)gpsi.hdg/1000.0f;//deg
                        double lat_d = (double)gpsi.lat/10000000.0;//deg
                        double lon_d = (double)gpsi.lon/10000000.0;//deg

                        if(m_PX4EpocTimeDiff != 0)
                        {
                            //wait until the time is set by my vehicle time
                            uint32_t timems = gpsi.time_boot_ms - m_PX4EpocTimeDiff;
                            uint64_t vID = msg.msgid;
                            MAVLINK_ProcessNewPosition(vID, newAlt_m, cog_d, lat_d, lon_d, timems);
                            COUT_INFO("Mavlink Diff ID -> Send AVS @ ID: " << vID);
                        }
                    }
                    else if(msg.msgid == MAVLINK_MSG_ID_HEARTBEAT ||
                       msg.msgid == MAVLINK_MSG_ID_SYS_STATUS)
                       {

                       }
                    else
                        COUT_INFO("Msg not for my vehicle ID " << int(msg.sysid) << " ID: " << int(msg.msgid))
                    continue;
                }         
                switch (msg.msgid)
                {
                    case MAVLINK_MSG_ID_HEARTBEAT://#0
                    {
                        mavlink_heartbeat_t heartbeat;
                        mavlink_msg_heartbeat_decode(&msg, &heartbeat);
                        std::cout << "HB @ ID: " << int(msg.sysid) << "/" << (uint16_t) heartbeat.autopilot << " - V: " << (uint16_t) heartbeat.mavlink_version << std::endl;
                        //send back heartbeat???
                        std::cout << "TIME: " << uxas::common::Time::getInstance().getUtcTimeSinceEpoch_ms() << std::endl;
                        if(!this->mWaypointDistCheck)
                        {
                            COUT_INFO("ASKING WP DIST...")
                            this->mWaypointDistCheck=true;
                            char param_id[16]="MIS_DIST_WPS";
                            int16_t param_index=-1;
                            uint8_t target_system=this->m_VehicleIDtoWatch;
                            uint8_t target_component=0;

                            mavlink_message_t msg;
                            uint8_t buf[MAVLINK_MAX_PACKET_LEN];
                            uint8_t system_id=m_MAVLinkID_UxAS;
                            uint8_t component_id=0;

                            mavlink_msg_param_request_read_pack(system_id,component_id,&msg,target_system,target_component,param_id,param_index);
                            /*uint16_t slen =*/ mavlink_msg_to_send_buffer(buf, &msg);
                            ssize_t send_len = sendto(m_netSocketFD, buf, sizeof(buf), 0, (struct sockaddr *) &m_remoteSocket, sizeof(m_remoteSocket));  
                        }
                        break;
                    }
                    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT://#33 //SITL only
                    {
                        mavlink_global_position_int_t gpsi;
                        mavlink_msg_global_position_int_decode(&msg,&gpsi);
                        float newAlt_m = (float)gpsi.alt/1000.0f;//AMSL
                        float cog_d = (float)gpsi.hdg/1000.0f;//deg
                        double lat_d = (double)gpsi.lat/10000000.0;//deg
                        double lon_d = (double)gpsi.lon/10000000.0;//deg
                        if(m_PX4EpocTimeDiff == 0)
                            this->m_PX4EpocTimeDiff = gpsi.time_boot_ms;

                        uint32_t timems = gpsi.time_boot_ms - m_PX4EpocTimeDiff;

                        MAVLINK_ProcessNewPosition(this->m_VehicleIDtoWatch, newAlt_m, cog_d, lat_d, lon_d, timems);
                        //COUT_INFO("GLOBAL_POSITION_INT")
                        break;
                    }
                    case MAVLINK_MSG_ID_GPS_RAW_INT://#24 //HITL and real
                    {
                        #warning HITL use only "MAVLINK_MSG_ID_GPS_RAW_INT"
                        /*mavlink_gps_raw_int_t rgpsi;
                        mavlink_msg_gps_raw_int_decode(&msg,&rgpsi);
                        float newAlt_m = (float)rgpsi.alt/1000.0f;//AMSL
                        float cog_d = (float)rgpsi.cog/100.0f;//deg
                        double lat_d = (double)rgpsi.lat/10000000.0;//deg
                        double lon_d = (double)rgpsi.lon/10000000.0;//deg
                        uint32_t timems = rgpsi.time_usec/1000;
                        //CHECK FOR VALID FIX???
                        //MAVLINK_ProcessNewPosition(newAlt_m, cog_d, lat_d, lon_d, timems);
                        MAVLINK_ProcessNewPosition(newAlt_m, cog_d, lat_d, lon_d, timems);*/
                        //COUT_INFO("GPS RAW INT");
                        break;
                    }
                    case MAVLINK_MSG_ID_MISSION_CURRENT://#224
                    {
                        mavlink_mission_current_t mcur;
                        mavlink_msg_mission_current_decode(&msg,&mcur);
                        //COUT_INFO("Mission Curr: "<<mcur.seq);
                        m_CurrentWaypoint=mcur.seq;
                        break;
                    }
                    case MAVLINK_MSG_ID_MISSION_ITEM_REACHED: //#46
                    {
                        COUT_INFO("MISSION_ITEM_REACHED")
                        break;
                    }
                    case MAVLINK_MSG_ID_VFR_HUD://#74
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
                    case MAVLINK_MSG_ID_ALTITUDE://#141
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
                    case MAVLINK_MSG_ID_PING://#4
                    {
                        COUT_INFO("MAV PING")
                        //Send ping back???
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
                    case MAVLINK_MSG_ID_ATTITUDE_QUATERNION://#31
                    {
                        break;
                    }
                    case MAVLINK_MSG_ID_LOCAL_POSITION_NED://#32
                    {
                        break;
                    }
                    case MAVLINK_MSG_ID_ATTITUDE_TARGET://#82
                    {
                        //Reports the current commanded attitude of the vehicle as specified by the autopilot. This should match the commands sent in a SET_ATTITUDE_TARGET message if the vehicle is being controlled this way.
                        break;
                    }
                    case MAVLINK_MSG_ID_PARAM_VALUE://#22
                    {
                        COUT_INFO("MAVLINK_MSG_ID_PARAM_VALUE")
                        mavlink_param_value_t pvt;
                        mavlink_msg_param_value_decode(&msg,&pvt);
                        COUT_INFO(pvt.param_id)
                        if(std::strcmp(pvt.param_id,"MIS_DIST_WPS")==0)
                        {
                            float dist = pvt.param_value;
                            char buf[128];
                            sprintf(buf,"GOT Max Waypoint Distance parameter: %4.2f", dist);
                            COUT_INFO(buf)
                            if(dist < MinWPDistCheck)
                            {
                                sprintf(buf,"ERROR, Configure PX4 MIS_DIST_WPS to be larger at: %4.2f", MinWPDistCheck);
                                COUT_INFO(buf)
                            }
                        }
                        break;
                    }
                    case MAVLINK_MSG_ID_HIGHRES_IMU://#105
                    {
                        mavlink_highres_imu_t highimu;
                        mavlink_msg_highres_imu_decode(&msg,&highimu);
                        //COUT_INFO("HighResIMU");
                        break;
                    }
                    case MAVLINK_MSG_ID_ESTIMATOR_STATUS://#230
                    {
                        break;
                    }
                    case MAVLINK_MSG_ID_HOME_POSITION://#242
                    {
                        mavlink_home_position_t homet;
                        mavlink_msg_home_position_decode(&msg,&homet);
                        if(m_SavedHomePositionMsg.latitude != homet.latitude &&
                            m_SavedHomePositionMsg.longitude != homet.longitude &&
                            m_SavedHomePositionMsg.altitude != homet.altitude)
                        {
                            //COUT_INFO("New HP? "<< m_SavedHomePositionMsg.latitude<<"/"<<homet.latitude);

                            std::memset(&this->m_SavedHomePositionMsg,0,sizeof(homet));
                            std::memcpy(&this->m_SavedHomePositionMsg,&homet,sizeof(homet));
                            COUT_INFO("Saved NEW HOME POSITION");
                        }                        
                        break;
                    }
                    case MAVLINK_MSG_ID_VIBRATION://#241
                    {
                        break;
                    }
                    case MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT://#86
                    {
                        //COUT_INFO("TARGET GLOBAL INT");
                        break;
                    }
                    case MAVLINK_MSG_ID_STATUSTEXT://#253
                    {
                        COUT_INFO("STATUS TEXT")
                        char textresponse[512];
                        mavlink_msg_statustext_get_text(&msg,textresponse);
                        COUT_INFO(textresponse);
                        break;
                    }
                    case MAVLINK_MSG_ID_COMMAND_LONG://#76
                    {
                        COUT_INFO("MAVLINK_MSG_ID_COMMAND_LONG")
                        break;
                    }
                    case MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED://#85
                    {
                        //COUT_INFO("MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED")//this happens alot
                        break;
                    }
                    case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW://#36
                    {
                        //COUT_INFO("MAVLINK_MSG_ID_SERVO_OUTPUT_RAW")//this happens alot
                        break;
                    }
                    case MAVLINK_MSG_ID_MISSION_COUNT://#44
                    {
                        mavlink_mission_count_t mcount;
                        mavlink_msg_mission_count_decode(&msg,&mcount);
                        COUT_INFO("RX COUNT " << mcount.count);
                        
                        //m_recvMissionCount=mcount.count;//only need if we are going to read waypoint list
                        break;
                    }
                    case MAVLINK_MSG_ID_MISSION_ACK://#47
                    {
                        COUT_INFO("MAVLINK_MSG_ID_MISSION_ACK");
                        mavlink_mission_ack_t mack;
                        mavlink_msg_mission_ack_decode(&msg,&mack);
                        if(m_missionSendState == SENT_CLEAR)
                        {
                            COUT_INFO("Finished clear (ack), sending count");
                            //reset the safety count
                            this->m_missionStateLastSendCount = -1; 
                            MissionUpdate_SendNewWayPointCount();
                        }
                        else if(m_missionSendState == SENT_LAST_WAYPOINT)
                        {
                            COUT_INFO("Finished waypoint write, got ACK");
                            //Set active waypoint here ???
                            //MissionUpdate_SetActiveWaypoint(2);
                        }
                        else
                        {
                            COUT_INFO("GOT ACK FOR NO REASON???")
                        }
                        break;
                    }
                    case MAVLINK_MSG_ID_MISSION_REQUEST_INT://#51
                    {
                        COUT_INFO("MAVLINK_MSG_ID_MISSION_REQUEST_INT");
                        if(m_missionSendState == SENT_COUNT)
                        {
                            //send first waypoint in my list
                            COUT_INFO("Rq: Sending 1st WP INT # " << m_wpIterator);
                            MissionUpdate_SendWayPointInt();
                        }
                        else if(m_missionSendState == SENT_WAYPOINT && m_wpIterator < m_newWaypointCount)
                        {
                            //send next waypoint
                            this->m_wpIterator++;
                            COUT_INFO("Rq: Sending next INT WP # " << m_wpIterator);
                            MissionUpdate_SendWayPointInt();
                        }
                        else if(m_missionSendState == SENT_ACTIVE_WAYPOINT)
                        {
                            COUT_INFO("Mission send done");
                        }
                        else
                        {
                            COUT_INFO("MISSION ACK ERROR");
                            COUT_INFO("Wp i: " << m_wpIterator << " wpcount " << m_newWaypointCount << " state :" << m_missionSendState);
                        }
                        break;
                    }
                    case MAVLINK_MSG_ID_MISSION_REQUEST://#40
                    {
                        COUT_INFO("MAVLINK_MSG_ID_MISSION_REQUEST)");
                        if(m_missionSendState == SENT_COUNT)
                        {
                            //send first waypoint in my list
                            COUT_INFO("Rq: Sending 1st WP # " << m_wpIterator);
                            #ifdef USE_MISSION_INT
                            MissionUpdate_SendWayPointInt();
                            #else
                            MissionUpdate_SendWayPoint();
                            #endif

                        }
                        else if(m_missionSendState == SENT_WAYPOINT && m_wpIterator < m_newWaypointCount)
                        {
                            //send next waypoint
                            this->m_wpIterator++;
                            COUT_INFO("Rq: Sending next WP # " << m_wpIterator);
                            #ifdef USE_MISSION_INT
                            MissionUpdate_SendWayPointInt();
                            #else
                            MissionUpdate_SendWayPoint();
                            #endif
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
                        mavlink_mission_item_request_decode(&msg,&mreq);
                        COUT_INFO("MISSION ITEM REQUEST");
                        break;
                    }*/
                    case MAVLINK_MSG_ID_UTM_GLOBAL_POSITION://#340 "SITL sents this
                    {
                        break;
                    }
                    case MAVLINK_MSG_ID_TIMESYNC://#111
                    {
                        //COUT_INFO("TIMESYNC?")
                        break;
                    }
                    case MAVLINK_MSG_ID_SCALED_IMU://#26
                    {
                        break;
                    }
                    case MAVLINK_MSG_ID_HIL_SENSOR://#107
                    {
                        break;
                    }
                    case MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS://#93
                    {
                        break;
                    }
                    case MAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET://#140
                    {
                        break;
                    }
                    case MAVLINK_MSG_ID_ODOMETRY://#331
                    {
                        break;
                    }
                    case MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT://#62
                    {
                        break;
                    }
                    case MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL ://#110
                    {
                        break;
                    }
                    default:
                    {
                        COUT_INFO("####################################### Msg: "<<msg.msgid);
                        break;
                    }
                }
            }         
        }
    } 
}
void PixhawkService::MAVLINK_ProcessNewPosition(uint64_t vehicleID, float nAlt, float nCOG, double nLat, double nLon, uint32_t ntimems)
{
    float newAlt_m = nAlt;
    double lat_d = nLat;
    double lon_d = nLon;
    double cog_d = nCOG;
    {
        std::lock_guard<std::mutex> lock(m_AirvehicleStateMutex);
        if(m_Attitude.time_boot_ms!=0)
        {
            m_ptr_CurrentAirVehicleState->setPitch(m_Attitude.pitch);
            m_ptr_CurrentAirVehicleState->setRoll(m_Attitude.roll);
        }
        m_ptr_CurrentAirVehicleState->setID(vehicleID);
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

        //build this from the actual battery...
        m_ptr_CurrentAirVehicleState->setEnergyAvailable(100);
        m_ptr_CurrentAirVehicleState->setActualEnergyRate(0.0001);
        m_ptr_CurrentAirVehicleState->setTime(ntimems);        
        
        m_ptr_CurrentAirVehicleState->setCurrentWaypoint(m_CurrentWaypoint);
        sendSharedLmcpObjectBroadcastMessage(m_ptr_CurrentAirVehicleState);
        bAVSReady=true;
        //COUT_INFO("Broadcasting AVS");
        if(m_missionSendState == WAIT_GLOBAL_POSITION)
        {
            std::shared_ptr<afrl::cmasi::Waypoint> newWP(new afrl::cmasi::Waypoint);

            double lat,lon,alt;
            
            lat = this->m_ptr_CurrentAirVehicleState->getLocation()->getLatitude(); 
            lon = this->m_ptr_CurrentAirVehicleState->getLocation()->getLongitude();
            alt =  this->m_ptr_CurrentAirVehicleState->getLocation()->getAltitude();

            //for the takeoff position...
            newWP->setLatitude(lat);
            newWP->setLongitude(lon);
            newWP->setAltitude(alt);
            newWP->setNumber(0);
            m_newWaypointList.insert(m_newWaypointList.begin(),newWP);
            //for the takeoff position...
            COUT_INFO("GOT Global POS, Starting WP send");
            m_missionStateLastSendCount=-1;
            this->MissionUpdate_ClearAutopilotWaypoints();//MissionUpdate_SendNewWayPointCount();
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
    
    uint8_t system_id=m_MAVLinkID_UxAS;
    uint8_t component_id=MAV_COMP_ID_MISSIONPLANNER;
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    uint8_t target_system=this->m_VehicleIDtoWatch;
    uint8_t target_component=0;
    uint8_t mission_type=0;

    //uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
    //                           uint8_t target_system, uint8_t target_component, uint8_t mission_type
    mavlink_msg_mission_clear_all_pack(system_id,component_id,&msg,
                                        target_system,target_component,mission_type);
    /*uint16_t slen =*/ mavlink_msg_to_send_buffer(buf, &msg);
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
        m_missionStateLastSendTime = uxas::common::Time::getInstance().getUtcTimeSinceEpoch_ms();
        if(this->m_missionStateLastSendCount == -1)
            m_missionStateLastSendCount = 0;
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
        COUT_INFO("Sending count of " << m_newWaypointCount << " waypoints");
    uint8_t system_id=m_MAVLinkID_UxAS;
    uint8_t component_id=MAV_COMP_ID_MISSIONPLANNER;
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    uint8_t target_system=this->m_VehicleIDtoWatch;
    uint8_t target_component=0;
    uint8_t mission_type=0;
    //uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
    //                           uint8_t target_system, uint8_t target_component, uint16_t count, uint8_t mission_type
    mavlink_msg_mission_count_pack(system_id, component_id, &msg,
                           target_system, target_component, this->m_newWaypointCount,mission_type);
    /*uint16_t slen =*/ mavlink_msg_to_send_buffer(buf, &msg);
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
        m_missionStateLastSendTime = uxas::common::Time::getInstance().getUtcTimeSinceEpoch_ms();
        if(this->m_missionStateLastSendCount == -1)
            m_missionStateLastSendCount = 0;
    }
}
void PixhawkService::MissionUpdate_SendWayPoint(void)
{
    auto wp = m_newWaypointList[m_wpIterator];
    //afrl::cmasi::Waypoint* wp = m_newWaypointList[m_wpIterator];
    uint8_t     system_id=m_MAVLinkID_UxAS;
    uint8_t     component_id=MAV_COMP_ID_MISSIONPLANNER;
    mavlink_message_t msg;
    uint8_t     target_system=this->m_VehicleIDtoWatch;
    uint8_t     target_component=0; 
    uint16_t    seq=m_wpIterator;
    uint8_t     frame=MAV_FRAME_GLOBAL_RELATIVE_ALT;//MAV_FRAME_GLOBAL
    uint16_t    command=MAV_CMD_NAV_WAYPOINT;
    uint8_t     current=0; 
    uint8_t     autocontinue=1;
    float       param1=0.0f;
    float       param2=0.0f;
    float       param3=0.0f; 
    float       param4=0.0f; 
    float       x=wp->getLatitude();
    float       y=wp->getLongitude();
    //float   def_alt = this->m_SavedHomePositionMsg.altitude;
    //def_alt /= 1000.0;
     #warning altitude is set at 50 above ground

    float       z = 50;//wp->getAltitude()+10;
    uint8_t mission_type=0;
    //179	MAV_CMD_DO_SET_HOME
    //22	MAV_CMD_NAV_TAKEOFF
    m_missionSendState = SENT_WAYPOINT;

    if(m_wpIterator == 0)
    {
        command = MAV_CMD_NAV_TAKEOFF;
        current = 0;
        param1 = 15.0;
        COUT_INFO("About to send Takeoff WP");
    }   
    else if(m_wpIterator == 1)
    {
        current = 1;
        COUT_INFO("About to send Active WP");
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
    /*uint16_t slen =*/ mavlink_msg_to_send_buffer(buf, &msg);
    ssize_t send_len = sendto(m_netSocketFD, buf, sizeof(buf), 0, (struct sockaddr *) &m_remoteSocket, sizeof(m_remoteSocket));
    if (send_len == -1)
    {
        COUT_INFO("bad WP sent " << send_len);
    }
    else
    {
        COUT_INFO("New WP sent #" << m_wpIterator << " (x,y,z) " << x << ", " << y << ", " << z);
    }
}
//deal with QGC changing the PX4's mission mode
void PixhawkService::MissionUpdate_SendWayPointInt(void)
{
    auto wp = m_newWaypointList[m_wpIterator];
    //afrl::cmasi::Waypoint* wp = m_newWaypointList[m_wpIterator];
    uint8_t     system_id=m_MAVLinkID_UxAS;
    uint8_t     component_id=MAV_COMP_ID_MISSIONPLANNER;
    mavlink_message_t msg;
    uint8_t     target_system=this->m_VehicleIDtoWatch;
    uint8_t     target_component=0; 
    uint16_t    seq=m_wpIterator;
    uint8_t     frame=MAV_FRAME_GLOBAL_RELATIVE_ALT;//MAV_FRAME_GLOBAL
    uint16_t    command=MAV_CMD_NAV_WAYPOINT;
    uint8_t     current=0; 
    uint8_t     autocontinue=1;
    float       param1=0.0f;
    float       param2=0.0f;
    float       param3=0.0f; 
    float       param4=0.0f; 
    float       tempFloat = wp->getLatitude()*10e7;
    int32_t     x=tempFloat;
    tempFloat = int32_t(wp->getLongitude()*10e7);
    int32_t     y=tempFloat;
    //float   def_alt = this->m_SavedHomePositionMsg.altitude;
    //def_alt /= 1000.0;
    #warning altitude is set at 50 above ground
    float       z = 50;//wp->getAltitude()+10;
    uint8_t mission_type=0;
    //179	MAV_CMD_DO_SET_HOME
    //22	MAV_CMD_NAV_TAKEOFF
    m_missionSendState = SENT_WAYPOINT;

    if(m_wpIterator == 0)
    {
        command = MAV_CMD_NAV_TAKEOFF;
        current = 0;
        param1 = 15.0;
        COUT_INFO("About to send Takeoff WP");
    }   
    else if(m_wpIterator == 1)
    {
        current = 1;
        COUT_INFO("About to send Active WP");
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
    mavlink_msg_mission_item_int_pack(system_id,component_id,&msg,target_system,
            target_component,seq,frame,command,current,autocontinue,
            param1,param2,param3,param4,x,y,z,mission_type);
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    mavlink_msg_to_send_buffer(buf, &msg);
    ssize_t send_len = sendto(m_netSocketFD, buf, sizeof(buf), 0, (struct sockaddr *) &m_remoteSocket, sizeof(m_remoteSocket));
    if (send_len == -1)
    {
        COUT_INFO("bad WP sent " << send_len);
    }
    else
    {
        COUT_INFO("New WP sent #" << m_wpIterator << " (x,y,z) " << x << ", " << y << ", " << z);
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
    
    uint8_t system_id=m_MAVLinkID_UxAS; //this is UxAS system id
    uint8_t component_id=MAV_COMP_ID_MISSIONPLANNER;
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    uint8_t target_system=this->m_VehicleIDtoWatch;
    uint8_t target_component=0;

    //uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
    //                           uint8_t target_system, uint8_t target_component, uint16_t seq
    mavlink_msg_mission_set_current_pack(system_id,component_id,&msg,
                                        target_system,target_component,newWP_px);
    /*uint16_t slen =*/ mavlink_msg_to_send_buffer(buf, &msg);
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
