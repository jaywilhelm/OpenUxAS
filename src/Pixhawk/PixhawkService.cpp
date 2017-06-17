
// include header for this service
#include "PixhawkService.h"


#warning "Building with Pixhawk"
#include "foo.hpp"
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

        //bind socket to port
        if( bind(m_netSocketFD , (struct sockaddr*)&m_listenSocket, sizeof(m_listenSocket) ) == -1)
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
    while (!m_isTerminate)
    {
        std::string strInputFromPiccolo;
        strInputFromPiccolo.clear();
        if (m_useNetConnection)
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
                char buf[1024];
                int len = 1;
                memset((char *) &m_remoteSocket, 0, sizeof(m_remoteSocket));

                socklen_t slen = sizeof(m_remoteSocket);
                int recv_len=0;
                    
                //int ret = recv(m_netSocketFD,buf,sizeof(buf), 0);//non-blocking, also will drop bytes if packet is smaller than buffer
                //(ret == 0)
                
                recv_len = recvfrom(m_netSocketFD, buf, sizeof(buf), 0, (struct sockaddr *) &m_remoteSocket, &slen);
                if (recv_len == -1)
                {
                    COUT_INFO("bad recv " << recv_len);
                }
                else
                {
                    buf[recv_len]=0;
                    //COUT_INFO("recv " << buf << " " << recv_len);
                    int chan = 0;
                    mavlink_message_t msg;
                    mavlink_status_t status;
                    std::cout << std::hex << (uint16_t) 0xFD << std::endl;
                    for(uint32_t i=0;i<recv_len;i++)
                    {
                        uint8_t mvp_ret = mavlink_parse_char(chan,buf[i],&msg,&status);
                        uint16_t data = 0x00FF & buf[i];
                        //std::cout << "p_ret " << std::hex << data << std::endl;
                        if(mvp_ret != 0)
                        {
                            COUT_INFO("Msg " << msg.msgid);
                            switch (msg.msgid)
                            {
                                case MAVLINK_MSG_ID_HEARTBEAT:
                                {
                                    mavlink_heartbeat_t heartbeat;
                                    mavlink_msg_heartbeat_decode(&msg, &heartbeat);
                                    std::cout << "HB " << (uint16_t) heartbeat.autopilot << " - " << (uint16_t) heartbeat.mavlink_version << std::endl;
                                }
                            }
                        }         
                    }
                        
                }               
                uint32_t flags = 0;
                //int ret = m_tcpConnectionSocket->recv(&buf,len,flags);
                //std::cout << "PX data " << ret << std::endl;
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