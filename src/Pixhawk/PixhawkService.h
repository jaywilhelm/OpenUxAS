#include "mavlinkc/common/mavlink.h"

// include header for this service

// include to add a periodic timer to send out messages periodically
#include "UxAS_TimerManager.h"

//include for KeyValuePair LMCP Message
#include "afrl/cmasi/KeyValuePair.h"

// print outs
#include <iostream>     // std::cout, cerr, etc
#include "serial/serial.h"
// convenience definitions for the option strings
#define STRING_XML_STRING_TO_SEND "StringToSend"
#define STRING_XML_SEND_PERIOD_MS "SendPeriod_ms"
#ifndef PIXHAWKSERVICE_H
#define PIXHAWKSERVICE_H
#include "afrl/cmasi/AirVehicleState.h"
#include "afrl/cmasi/MissionCommand.h"
#include "afrl/cmasi/AirVehicleConfiguration.h"
#include "afrl/cmasi/CameraState.h"
#include "afrl/cmasi/GimbalState.h"
#include "afrl/cmasi/VideoStreamState.h"
#include "ServiceBase.h"
#include "CallbackTimer.h"
#include "TypeDefs/UxAS_TypeDefs_Timer.h"
extern "C"{
#include<arpa/inet.h>
#include<sys/socket.h>
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
    /** brief The timer calls this function periodically to send out messages */
    //void OnSendMessage();
    void
    executePixhawkAutopilotCommProcessing();
    
protected:
    std::string m_stringToSend = std::string("PixhawkService String");
    int64_t m_sendPeriod_ms{1000};
    uint64_t m_sendMessageTimerId{0};
    
    bool m_useNetConnection{true};
    ///// TCP/IP
    /*! \brief this is the zmq context used to connect to the external device */
    ///std::shared_ptr<zmq::context_t> m_contextLocal;
    /*! \brief this is the stream socket used to connect to the external device */
    //std::shared_ptr<zmq::socket_t> m_tcpConnectionSocket;
    int m_netSocketFD;
    sockaddr_in m_listenSocket;
    sockaddr_in m_remoteSocket;
    //std::string m_tcpAddress{"udp://localhost:14501"};
    uint16_t m_netPort{14552};
    bool m_bServer{true};
    std::unique_ptr<std::thread> m_receiveFromPixhawkProcessingThread;
    bool m_isTerminate{false};//read thread terminate
    int64_t m_CurrentWaypoint{-1};
    mavlink_attitude_t m_Attitude;
    double m_Airspeed{0.0};
    std::shared_ptr<afrl::cmasi::AirVehicleState> m_ptr_CurrentAirVehicleState;
    std::mutex m_AirvehicleStateMutex;

    
    //Serial
    std::shared_ptr<serial::Serial> m_serialConnectionPixhawk;
    std::string m_strTTyDevice{"/dev/tty.usbmodem"};
    uint32_t m_ui32Baudrate{57600};
    uint32_t m_serialTimeout_ms{5000};
};

}; //namespace service
}; //namespace uxas

#endif /* PIXHAWKSERVICE_H */