/*
BSD 3-Clause License

Copyright (c) 2023, teamprof.net@gmail.com. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1 Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.
2 Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.
3 Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#pragma once
#include <Arduino.h>
#include "./LibLog.h"

#if defined(TARGET_RASPBERRY_PI_PICO)
#include <mbed.h>
#define delayMilliSecond(x) rtos::ThisThread::sleep_for(std::chrono::milliseconds(x))
#elif defined(ESP8266) || defined(ESP32)
#define delayMilliSecond(x) vTaskDelay(pdMS_TO_TICKS(x))
#else
#define delayMilliSecond(x) delay(x)
#endif

#if defined(TARGET_RASPBERRY_PI_PICO)
#include <mbed.h>
// using namespace mbed;
using namespace rtos;
using namespace events;

#define PIN_CSn 26u  // W5500: nCS pin = GPIO26 (WIZ830MJ-RP2040 connector)
#define Pin_Intn 27u // W5500: INTn pin = GPIO27 (WIZ830MJ-RP2040 connector)
#define Pin_A0 0u    // W5500: A0 pin = GPIO0 (WIZ830MJ-RP2040 connector)
#define Pin_A1 1u    // W5500: A1 pin = GPIO1 (WIZ830MJ-RP2040 connector)

// note: A2-A9 is overlap with D8-D15
#define Pin_A2 2u // W5500: A2 pin = GPIO2 (WIZ830MJ-RP2040 connector)
#define Pin_A3 3u // W5500: A3 pin = GPIO3 (WIZ830MJ-RP2040 connector)
#define Pin_A4 4u // W5500: A4 pin = GPIO4 (WIZ830MJ-RP2040 connector)
#define Pin_A5 5u // W5500: A5 pin = GPIO5 (WIZ830MJ-RP2040 connector)
#define Pin_A6 6u // W5500: A6 pin = GPIO6 (WIZ830MJ-RP2040 connector)
#define Pin_A7 7u // W5500: A7 pin = GPIO7 (WIZ830MJ-RP2040 connector)
#define Pin_A8 8u // W5500: A8 pin = GPIO8 (WIZ830MJ-RP2040 connector)
#define Pin_A9 9u // W5500: A9 pin = GPIO9 (WIZ830MJ-RP2040 connector)

#define Pin_D0 10u // W5500: D0 pin = GPIO10 (WIZ830MJ-RP2040 connector)
#define Pin_D1 11u // W5500: D1 pin = GPIO11 (WIZ830MJ-RP2040 connector)
#define Pin_D2 12u // W5500: D2 pin = GPIO12 (WIZ830MJ-RP2040 connector)
#define Pin_D3 13u // W5500: D3 pin = GPIO13 (WIZ830MJ-RP2040 connector)
#define Pin_D4 14u // W5500: D4 pin = GPIO14 (WIZ830MJ-RP2040 connector)
#define Pin_D5 15u // W5500: D5 pin = GPIO15 (WIZ830MJ-RP2040 connector)
#define Pin_D6 16u // W5500: D6 pin = GPIO16 (WIZ830MJ-RP2040 connector)
#define Pin_D7 17u // W5500: D7 pin = GPIO17 (WIZ830MJ-RP2040 connector)

#define Pin_Wr 18u // W5500: /WR pin = GPIO18 (WIZ830MJ-RP2040 connector)
#define Pin_Rd 19u // W5500: /RD pin = GPIO19 (WIZ830MJ-RP2040 connector)

#elif defined(ESP32)
#error "Not support"
#else
#error "No Wiznet chip module defined"
#endif

#include "./Ethernet.h"

#define Sn_INT (IR_SnINT(7) | IR_SnINT(6) | IR_SnINT(5) | IR_SnINT(4) | \
                IR_SnINT(3) | IR_SnINT(2) | IR_SnINT(1) | IR_SnINT(0))

class SocketEventApi
{
public:
    SocketEventApi();
    ~SocketEventApi();

    typedef void (*SocketEventCallback)(uint16_t sn_ir);
    virtual bool subscribeSocketEvent(uint16_t sn_ir, SocketEventCallback onSocketEvent, uint8_t sockindex);
    virtual void unsubscribeSocketEvent(uint16_t sn_ir, uint8_t sockindex);

protected:
    // uint8_t sockIndex; // MAX_SOCK_NUM means client not in use
    uint16_t sn_ir;
    SocketEventCallback eventCallback;
};

///////////////////////////////////////////////////////////////////////////////
// EventEthernetUDP
///////////////////////////////////////////////////////////////////////////////
#define SnIR_MASK (Sn_IR_PRECV | Sn_IR_PFAIL | Sn_IR_PNEXT | Sn_IR_SENDOK | Sn_IR_TIMEOUT | Sn_IR_RECV | Sn_IR_DISCON | Sn_IR_CON)
class EventEthernetUDP : public EthernetUDP, SocketEventApi
{
public:
    virtual void begin(uint16_t port, uint16_t sn_ir = 0, SocketEventCallback onSocketEvent = nullptr);
    virtual uint8_t beginMulticast(IPAddress ip, uint16_t port, uint16_t sn_ir = 0, SocketEventCallback onSocketEvent = nullptr); // initialize, start listening on specified port. Returns 1 if successful, 0 if there are no sockets available to use
    virtual void stop();
};

///////////////////////////////////////////////////////////////////////////////
// EventEthernetClient
///////////////////////////////////////////////////////////////////////////////
class EventEthernetClient : public EthernetClient, SocketEventApi
{
public:
    EventEthernetClient() : EthernetClient() {}
    EventEthernetClient(uint8_t s) : EthernetClient(s) {}
    virtual int connect(IPAddress ip, uint16_t port, uint16_t sn_ir = 0, SocketEventCallback onSocketEvent = nullptr);
    virtual int connect(const char *host, uint16_t port, uint16_t sn_ir = 0, SocketEventCallback onSocketEvent = nullptr);
    virtual void stop();
};

///////////////////////////////////////////////////////////////////////////////
// EventEthernetServer
///////////////////////////////////////////////////////////////////////////////
class EventEthernetServer : public EthernetServer, SocketEventApi
{
public:
    EventEthernetServer(uint16_t port);
    ~EventEthernetServer();
    EventEthernetClient available();
    EventEthernetClient accept();
    virtual void begin(uint16_t sn_ir = 0, SocketEventCallback onSocketEvent = nullptr);

    virtual bool subscribeSocketEvent(uint16_t sn_ir, SocketEventCallback onSocketEvent);
    virtual void unsubscribeSocketEvent(uint16_t sn_ir);

private:
    uint16_t _serverPort;
};

///////////////////////////////////////////////////////////////////////////////
// EventEthernetClass
///////////////////////////////////////////////////////////////////////////////
class EventEthernetClass : public EthernetClass
{
public:
    typedef void (*EthernetEventCallback)(uint16_t ir);

    EventEthernetClass();
    ~EventEthernetClass();
    static int begin(uint8_t *mac, uint16_t ir, EthernetEventCallback onEthernetEvent);
    static int begin(uint8_t *mac, unsigned long timeout = 60000, unsigned long responseTimeout = 4000, uint8_t ir = 0, EthernetEventCallback onEthernetEvent = nullptr);
    static void begin(uint8_t *mac, IPAddress ip, uint8_t ir = 0, EthernetEventCallback onEthernetEvent = nullptr);
    static void begin(uint8_t *mac, IPAddress ip, IPAddress dns, uint8_t ir = 0, EthernetEventCallback onEthernetEvent = nullptr);
    static void begin(uint8_t *mac, IPAddress ip, IPAddress dns, IPAddress gateway, uint8_t ir = 0, EthernetEventCallback onEthernetEvent = nullptr);
    static void begin(uint8_t *mac, IPAddress ip, IPAddress dns, IPAddress gateway, IPAddress subnet, uint8_t ir = 0, EthernetEventCallback onEthernetEvent = nullptr);

    void subscribeEthernetEvent(uint16_t ir, EthernetEventCallback onEthernetEvent);
    void unsubscribeEthernetEvent(uint16_t ir);

    void init(uint8_t sspin = PIN_CSn, int8_t pinIntn = -1, int8_t pinRst = -1);
    void hardReset(void);

    bool registerSocketCallback(uint8_t sockindex, SocketEventApi::SocketEventCallback onSocketEvent);
    void unregisterSocketCallback(uint8_t sockindex);

    friend class EthernetClient;
    friend class EthernetServer;
    friend class EthernetUDP;
    friend class EthernetClass;
    friend class W5300Class;

private:
    uint16_t ir;
    EthernetEventCallback ethernetEventCallback;
    SocketEventApi::SocketEventCallback socketEventCallback[MAX_SOCK_NUM];
    // void (*ethernetEventCallback)(uint8_t, uint8_t, uint8_t);
    // void (*socketEventCallback[MAX_SOCK_NUM])(uint8_t);

    int8_t pinIntn;
    int8_t pinRst;
    static EventEthernetClass *instance;
    static void isr(EventEthernetClass *);
    void queueIsr(void);

#if defined(TARGET_RASPBERRY_PI_PICO)
    Thread thread;
    EventQueue queue;
#elif defined(ESP32)
    static TaskHandle_t taskHandle;
    static EventGroupHandle_t eventGroupHandle;
    static StaticEventGroup_t createdEventGroup;
#endif
};

// refer to W5100.cpp for chip ID
#define CHIP_W5100S 50
#define CHIP_W5100 51
#define CHIP_W5200 52
#define CHIP_W5500 55
#define CHIP_W6100 61
#define CHIP_W5300 0x53

#define UDP_PACKET_INFO_SIZE 8
#define TCP_MACRAW_PACKET_INFO_SIZE 2

extern EventEthernetClass Ethernet;
