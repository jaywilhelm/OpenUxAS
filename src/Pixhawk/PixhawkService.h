//#define MAVLINK_USE_CONVENIENCE_FUNCTIONS
#include "mavlinkc/common/mavlink.h"

// include header for this service

// include to add a periodic timer to send out messages periodically
#include "UxAS_TimerManager.h"

//include for KeyValuePair LMCP Message
#include "afrl/cmasi/KeyValuePair.h"

// print outs
#include <iostream>     // std::cout, cerr, etc
//#include "serial/serial.h"
//#include "SerialHelper.h"

// convenience definitions for the option strings
#ifndef PIXHAWKSERVICE_H
#define PIXHAWKSERVICE_H
#include "afrl/cmasi/AirVehicleState.h"
#include "afrl/cmasi/MissionCommand.h"
#include "afrl/cmasi/AirVehicleConfiguration.h"
#include "afrl/cmasi/CameraState.h"
#include "afrl/cmasi/GimbalState.h"
#include "afrl/cmasi/VideoStreamState.h"
#include "uxas/messages/uxnative/AutopilotKeepAlive.h"
#include "uxas/messages/uxnative/StartupComplete.h"
#include "afrl/cmasi/MissionCommand.h"
#include "afrl/cmasi/GoToWaypointAction.h"
#include "afrl/cmasi/NavigationMode.h"

#include "ServiceBase.h"
#include "CallbackTimer.h"
#include "TypeDefs/UxAS_TypeDefs_Timer.h"
extern "C"{
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/ip.h>
};
namespace uxas
{
namespace service
{

/*! \class PixhawkService
 *\brief This is a basic example of a UxAS service that sends/receives KeyValuePair
 * messages and prints out the results. 
 * 
 * Configuration String:
 *  <Service Type="PixhawkService" StringToSend="Hello World!" SendPeriod_ms="1000" />
 * 
 * Options:
 *  - StringToSend - the text of the message to send out
 *  - SendPeriod_ms - how often, in milliseconds, to send the message
 * 
 * Subscribed Messages:
 *  - afrl::cmasi::KeyValuePair
 * 
 * Sent Messages:
 *  - afrl::cmasi::KeyValuePair
 * 
 */

class PixhawkService : public ServiceBase
{
public:

    /** \brief This string is used to identify this service in XML configuration
     * files, i.e. Service Type="PihawkService". It is also entered into
     * service registry and used to create new instances of this service. */
    static const std::string&
    s_typeName()
    {
        static std::string s_string("PixhawkService");
        return (s_string);
    };

    static const std::vector<std::string>
    s_registryServiceTypeNames()
    {
        std::vector<std::string> registryServiceTypeNames = {s_typeName()};
        return (registryServiceTypeNames);
    };

    /** \brief If this string is not empty, it is used to create a data 
     * directory to be used by the service. The path to this directory is
     * accessed through the ServiceBase variable m_workDirectoryPath. */
    static const std::string&
    s_directoryName() { static std::string s_string(""); return (s_string); };

    static ServiceBase*
    create()
    {
        return new PixhawkService;
    };


    // service constructor
    PixhawkService();
    virtual ~PixhawkService();

private:

    static
    ServiceBase::CreationRegistrar<PixhawkService> s_registrar;

    /** brief Copy construction not permitted */
    PixhawkService(PixhawkService const&) = delete;

    /** brief Copy assignment operation not permitted */
    void operator=(PixhawkService const&) = delete;

    bool
    configure(const pugi::xml_node& serviceXmlNode) override;

    bool
    initialize() override;

    bool
    start() override;

    bool
    terminate() override;

    bool
    processReceivedLmcpMessage(std::unique_ptr<uxas::communications::data::LmcpMessage> receivedLmcpMessage) override;
    
    
private:
protected:

    /** brief The timer calls this function periodically to send out messages */
    void    executePixhawkAutopilotCommProcessing();
    void    executeSecondaryMAVLINKProcessing();
    std::unique_ptr<std::thread> m_receiveFromPixhawkProcessingThread;
    std::unique_ptr<std::thread> m_receiveFromSecondaryThread;

    int     MavlinkConnect(void);
    int     MavlinkDisconnect();

    void    SafetyTimer();
    void    CheckMaxPX4WPDist(void);

    std::string m_stringToSend = std::string("PixhawkService String");
    int64_t     m_dtSafetyTimer{1000};
    uint64_t    m_SafetyTimerId{0};
    
    bool        m_useNetConnection{true};
    int         m_netSocketFD;
    sockaddr_in m_listenAddr;
    sockaddr_in m_remoteAddr;
    #define     netPortDefault 14550;
    uint16_t    m_listenPortMavlink{0};
    uint16_t    m_IncomingMAVLinkSecondary{14541};
    int         m_netSocketFDSecondary;
    sockaddr_in m_listenAddrSecondary;
    sockaddr_in m_remoteAddrSecondary;
    int RecvAndProcessFromMainMAVLINKConnection(void);
    int RecvAndProcessFromSecondaryMAVLINKConnection(void);

    int RecvAndProcessMAVLINKConnection(int sockfd, uint8_t chan);

    int ProcessMAVLINKMessage(mavlink_message_t &msg, mavlink_status_t &status, uint8_t chan);

    bool        m_bServer{true};
    bool        m_isTerminate{false};//read thread terminate
    int64_t     m_CurrentWaypoint{-1};
    mavlink_attitude_t m_Attitude;
    double      m_Airspeed{0.0};
    std::shared_ptr<afrl::cmasi::AirVehicleState> m_ptr_CurrentAirVehicleState;
    std::mutex  m_AirvehicleStateMutex;
    bool        bAVSReady=false;
    //Serial
    //std::shared_ptr<serial::Serial> m_serialConnectionPixhawk;
    std::string m_strTTyDevice{"/dev/tty.usbmodem"};
    uint32_t m_ui32Baudrate{57600};
    uint32_t m_serialTimeout_ms{5000};
    uint64_t m_PX4EpocTimeDiff=0;
    bool m_bStartupComplete{false};
        enum Mission_States{
        NULL_STATE,
        WAIT_GLOBAL_POSITION,
        SENT_CLEAR,
        SENT_COUNT,
        SENT_WAYPOINT,
        SENT_LAST_WAYPOINT,
        SENT_ACTIVE_WAYPOINT
    };
    int32_t m_missionSendState{NULL_STATE};

    int32_t m_wpIterator{0};
    int32_t m_newWaypointCount{0};
    uint16_t m_VehicleIDtoWatch{1};
    uint16_t m_MAVLinkID_UxAS{199};
    uint64_t m_WaypointOffset{0};
    //std::vector<afrl::cmasi::Waypoint*> m_newWaypointList;
    std::vector<std::shared_ptr<afrl::cmasi::Waypoint>> m_newWaypointList;
    //std::shared_ptr<afrl::cmasi::MissionCommand> m_newMissionCommand;
    void MAVLINK_ProcessNewPosition(uint64_t vID, float nAlt, float nCOG, double nLat, double nLon, uint32_t ntimems);

    void Process_isMissionCommand(std::shared_ptr<afrl::cmasi::MissionCommand> missionCmd);

    uint64_t m_missionStateLastSendTime = 0;
    int32_t m_missionStateLastSendCount = 0;
    void MissionUpdate_ClearAutopilotWaypoints(void);
    void MissionUpdate_SendNewWayPointCount(void);
    void MissionUpdate_SendWayPoint(void);
    void MissionUpdate_SendWayPointInt(void);
    void MissionUpdate_SetActiveWaypoint(uint32_t newWP_px);
    bool mWaypointDistCheck=false;
    mavlink_home_position_t m_SavedHomePositionMsg;
};

}; //namespace service
}; //namespace uxas

#endif /* PIXHAWKSERVICE_H */
