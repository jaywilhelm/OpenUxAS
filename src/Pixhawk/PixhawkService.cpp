
// include header for this service
#include "PixhawkService.h"


#warning "Building with Pixhawk"
#include "foo.hpp"

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
    mavlink_message_t r_message;
    mavlink_status_t r_mavlink_status;
    char newc = 0;
    uint8_t res = mavlink_parse_char(0,newc, &r_message, &r_mavlink_status);
    foobar fb;
    std::cout << "PixhawkService called"<<" "<<fb.giveme() << std::endl;
}

PixhawkService::~PixhawkService() { }
bool PixhawkService::configure(const pugi::xml_node& ndComponent)
{
    bool isSuccess(true);

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

    return (isSuccess);
}

bool PixhawkService::initialize()
{
    // create send timer
    //m_sendMessageTimerId = uxas::common::TimerManager::getInstance().createTimer(
    //    std::bind(&HelloWorld::OnSendMessage, this), "HelloWorld::OnSendMessage");
    
    return (true);
}

bool PixhawkService::start()
{
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
    
    return (true);
}

bool PixhawkService::processReceivedLmcpMessage(std::unique_ptr<uxas::communications::data::LmcpMessage> receivedLmcpMessage)
{
    if (afrl::cmasi::isKeyValuePair(receivedLmcpMessage->m_object))
    {
        //receive message
        //auto keyValuePairIn = std::static_pointer_cast<afrl::cmasi::KeyValuePair> (receivedLmcpMessage->m_object);
        //std::cout << "*** RECEIVED:: Received Id[" << m_serviceId << "] Sent Id[" << keyValuePairIn->getKey() << "] Message[" << keyValuePairIn->getValue() << "] *** " << std::endl;
    }
    return false;
}

};};