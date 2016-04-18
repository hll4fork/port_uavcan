/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <unistd.h>
#include <ch.hpp>
#include <hal.h>
#include <uavcan_stm32/uavcan_stm32.hpp>
#include <uavcan/equipment/actuator/Status.hpp>

#include "debug.h"
#include <errno.h>

#define UAVCAN_NODE_ID 1

namespace app
{
namespace
{

uavcan_stm32::CanInitHelper<128> can;

typedef uavcan::Node<16384> Node;

uavcan::LazyConstructor<Node> node_;

Node& getNode()
{
    if (!node_.isConstructed())
    {
        node_.construct<uavcan::ICanDriver&, uavcan::ISystemClock&>(can.driver, uavcan_stm32::SystemClock::instance());
    }
    return *node_;
}

void ledSet(bool state)
{
    palWritePad(GPIOG, GPIOG_LED3_GREEN, state);
}

#if __GNUC__
__attribute__((noreturn))
#endif
void die(int status)
{
    lowsyslog("Now I am dead x_x %i\n", status);
    while (1)
    {
    	ledSet(false);
        chThdSleepMilliseconds(500);
        ledSet(true);
        chThdSleepMilliseconds(500);
    }
}

class : public chibios_rt::BaseStaticThread<8192>
{
public:
    void main() override
    {
        int res;
        std::uint32_t bitrate = 1000000;
        res = can.init([]() { ::usleep(can.getRecommendedListeningDelay().toUSec()); }, bitrate);
        if (res < 0)
        {
            die(res);
        }

        /*
         * Setting up the node parameters
         */
        Node& node = app::getNode();

        node.setNodeID(UAVCAN_NODE_ID);
        node.setName("org.uavcan.stm32f429");

        /*
         * Initializing the UAVCAN node - this may take a while
         */
        while (true)
        {
            // Calling start() multiple times is OK - only the first successfull call will be effective
            int res = node.start();

            if (res < 0)
            {
                lowsyslog("Node initialization failure: %i, will try agin soon\n", res);
            }
            else
            {
                break;
            }
            chThdSleepMilliseconds(1000);
        }

        // publisher uavcan::equipment::actuator::Status msg
        uavcan::Publisher<uavcan::equipment::actuator::Status> s_pub(node);
        const int s_pub_start_res = s_pub.init();
        if (s_pub_start_res < 0) {
            lowsyslog("error Status publisher init");
            while (1);
        }
        // publisher uavcan::equipment::actuator::Status msg

        /*
         * Main loop
         */
        lowsyslog("UAVCAN node started\n");
        node.setModeOperational();
        while (true)
        {
            const int spin_res = node.spin(uavcan::MonotonicDuration::fromMSec(1000));
            if (spin_res < 0)
            {
                lowsyslog("Spin failure: %i\n", spin_res);
            }

            // publisher uavcan::equipment::actuator::Command msg
            uavcan::equipment::actuator::Status s_msg;
            s_msg.actuator_id = 1;
            s_msg.position = 1.0f;
            s_msg.force = 1.0f;
            s_msg.speed = 1.0f;
            s_msg.power_rating_pct = 0;

            const int s_pub_res = s_pub.broadcast(s_msg);
            if(s_pub_res <0){
            	lowsyslog("Status Broadcast failure:\n");
            }
            // publisher uavcan::equipment::actuator::Command msg

            palTogglePad(GPIOG, GPIOG_LED3_GREEN);
        }
    }
} uavcan_node_thread;

}
}

extern "C" {

int uavcan_node_start(void *arg)
{
    (void) arg;
    lowsyslog("Starting the UAVCAN thread\n");
    app::uavcan_node_thread.start(LOWPRIO);
    return 0;
}

} // extern "C"
