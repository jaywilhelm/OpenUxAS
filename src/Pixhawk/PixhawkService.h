#include "mavlinkc/common/mavlink.h"

// include header for this service

// include to add a periodic timer to send out messages periodically
#include "UxAS_TimerManager.h"

//include for KeyValuePair LMCP Message
#include "afrl/cmasi/KeyValuePair.h"

// print outs
#include <iostream>     // std::cout, cerr, etc

// convenience definitions for the option strings
#define STRING_XML_STRING_TO_SEND "StringToSend"
#define STRING_XML_SEND_PERIOD_MS "SendPeriod_ms"
#ifndef PIXHAWKSERVICE_H
#define PIXHAWKSERVICE_H

#include "ServiceBase.h"
#include "CallbackTimer.h"
#include "TypeDefs/UxAS_TypeDefs_Timer.h"

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
    
private:
    std::string m_stringToSend = std::string("PixhawkService String");
    int64_t m_sendPeriod_ms{1000};
    uint64_t m_sendMessageTimerId{0};

};

}; //namespace service
}; //namespace uxas

#endif /* PIXHAWKSERVICE_H */