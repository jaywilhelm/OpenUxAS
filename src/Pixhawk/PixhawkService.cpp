
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

    return (isSuccess);
}

bool PixhawkService::initialize()
{
    // create send timer
    //m_sendMessageTimerId = uxas::common::TimerManager::getInstance().createTimer(
    //    std::bind(&HelloWorld::OnSendMessage, this), "HelloWorld::OnSendMessage");
    bool bSuccess(true);

    std::cout <<  "PX init"<<std::endl;

    if (m_useTcpIpConnection)
    {
        // open tcp/ip socket for sending/receiving messages
        assert(!m_tcpAddress.empty());
        std::cout << "PX Connecting to " << m_tcpAddress << std::endl;
        m_contextLocal.reset(new zmq::context_t(1));
        m_tcpConnectionSocket.reset(new zmq::socket_t(*m_contextLocal, ZMQ_STREAM));
        if (m_bServer)
        {
            m_tcpConnectionSocket->bind(m_tcpAddress.c_str());
        }
        else
        {
            m_tcpConnectionSocket->connect(m_tcpAddress.c_str());
        }
    }
    else
    {
        // 0) initialize the serial connection
        //m_serialConnectionPiccolo.reset(new serial::Serial(m_strTTyDevice, m_ui32Baudrate, serial::Timeout::simpleTimeout(m_serialTimeout_ms)));
        //if (!m_serialConnectionPiccolo->isOpen())
        {
            //UXAS_LOG_ERROR(s_typeName(), ":: Initialize - serial connection failed:: m_strTTyDevice[", m_strTTyDevice, "m_ui32Baudrate[", m_ui32Baudrate);
            bSuccess = false;
        }
    }
    return (bSuccess);
}

bool PixhawkService::start()
{
    m_receiveFromPixhawkProcessingThread = uxas::stduxas::make_unique<std::thread>(&PixhawkService::executePixhawkAutopilotCommProcessing, this);
    std::cout <<  "PX start"<<std::endl;
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

    if (m_tcpConnectionSocket)
    {
        uint32_t ui32LingerTime(0);
        m_tcpConnectionSocket->setsockopt(ZMQ_LINGER, &ui32LingerTime, sizeof (ui32LingerTime));
        m_tcpConnectionSocket->close();
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
    std::cout <<  "PX executePixhawkAutopilotCommProcessing"<<std::endl;

    std::string strInputFromPiccolo;
    while (!m_isTerminate)
    {
        std::string strInputFromPiccolo;
        strInputFromPiccolo.clear();
        if (m_useTcpIpConnection)
        {
            //  Process messages from receiver and controller
            //zmq::pollitem_t items [] = {
            //    { *m_tcpConnectionSocket, 0, ZMQ_POLLIN, 0}
            //};

            //TODO:: should I use 0 time out???
            //size_t szPollTimeOut_ms = 1; //larger numbers limit the speed that messages can be sent

            //zmq::poll(&items [0], 1, szPollTimeOut_ms);

            //if (items [0].revents & ZMQ_POLLIN) //m_ptr_ZsckTcpConnection
            {
                char buf;
                int len = 1;
                uint32_t flags = 0;
                int ret = m_tcpConnectionSocket->recv(&buf,len,flags);
                std::cout << "PX data " << ret << std::endl;
            } 
        }
        else
        {
            //assert(m_serialConnectionPiccolo);
            //strInputFromPiccolo = m_serialConnectionPiccolo->read(m_serialReadSize);
            //UXAS_LOG_DEBUG_VERBOSE("PiccoloAutopilotAdapterService::executePiccoloAutopilotSerialProcessing", " bytes on serial port: ", strInputFromPiccolo.length());
            //UXAS_LOG_DEBUG_VERBOSE("PiccoloAutopilotAdapterService::executePiccoloAutopilotSerialProcessing", " all bytes read from serial port: ", strInputFromPiccolo);
        }
    }
};



};};