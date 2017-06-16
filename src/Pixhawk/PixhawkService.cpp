
// include header for this service
#include "PixhawkService.h"


#warning "Building with Pixhawk"
#include "foo.hpp"

// namespace definitions
namespace uxas  // uxas::
{
namespace service   // uxas::service::
{

// this entry registers the service in the service creation registry
PixhawkService::ServiceBase::CreationRegistrar<PixhawkService>
PixhawkService::s_registrar(PixhawkService::s_registryServiceTypeNames());
// service constructor
PixhawkService::PixhawkService()
: ServiceBase(PixhawkService::s_typeName(), PixhawkService::s_directoryName()) 
{ 
        mavlink_message_t r_message;
        mavlink_status_t r_mavlink_status;
        char newc = 0;
        uint8_t res = mavlink_parse_char(0,newc, &r_message, &r_mavlink_status);
        
        foobar fb;
        std::cout << "PixhawkService called"<<" "<<fb.giveme() << std::endl;
};

// service destructor
PixhawkService::~PixhawkService() {};
};
};
