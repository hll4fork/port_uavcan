/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <unistd.h>
#include <ch.hpp>
#include <hal.h>
#include <uavcan_stm32/uavcan_stm32.hpp>
#include <uavcan/protocol/NodeStatus.hpp>

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

#if __GNUC__
__attribute__((noreturn))
#endif
void die(int status)
{
    lowsyslog("Now I am dead x_x %i\n", status);
    while (1)
    {
        chThdSleepMilliseconds(500);
    }
}

void node_status_cb(const uavcan::ReceivedDataStructure<uavcan::protocol::NodeStatus>& msg)
{
    const uint8_t st = msg.health;
    const char *st_name;
    switch (st) {
        case 0:
            st_name = "HEALTH_OK";
            break;
        case 1:
            st_name = "HEALTH_WARNING";
            break;
        case 2:
            st_name = "HEALTH_ERROR";
            break;
        case 3:
            st_name = "HEALTH_CRITICAL";
            break;
        default:
            st_name = "UNKNOWN_STATUS";
            break;
    }
    //palTogglePad(GPIOC, GPIOC_LED);
    lowsyslog("NodeStatus from %d: %u (%s)\n", msg.getSrcNodeID().get(), st, st_name);
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
        node.setName("org.uavcan.cvra_test_stm32f407");

        // TODO: fill software version info (version number, VCS commit hash, ...)
        // TODO: fill hardware version info (version number, unique ID)

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

        /*
         * NodeStatus subscriber
         */
        uavcan::Subscriber<uavcan::protocol::NodeStatus> ns_sub(node);
        const int ns_sub_start_res = ns_sub.start(node_status_cb);
        if (ns_sub_start_res < 0) {
            lowsyslog("error NodeStatus subscriber init");
            while (1);
        }

        /*
         * Main loop
         */
        lowsyslog("UAVCAN node started\n");
        node.setHealthOk();
        while (true)
        {
            const int spin_res = node.spin(uavcan::MonotonicDuration::fromMSec(1000));
            if (spin_res < 0)
            {
                lowsyslog("Spin failure: %i\n", spin_res);
            }

            lowsyslog("Memory usage: used=%u free=%u\n",
                      node.getAllocator().getNumUsedBlocks(), node.getAllocator().getNumFreeBlocks());

            lowsyslog("CAN errors: %lu\n",
                      static_cast<unsigned long>(can.driver.getIface(0)->getErrorCount()));
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
