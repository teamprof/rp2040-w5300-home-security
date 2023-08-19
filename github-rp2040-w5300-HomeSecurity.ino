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
#include <mbed.h>
#include "AppLog.h"
#include <UrlEncode.h>
#include "./src/lib/EventEthernet/src/EventEthernet.h"

#include "secret.h"

////////////////////////////////////////////////////////////////////////////////////////////
// Pi Pico pin connection
// pin#         GPIO            connect to
// 1-2          GPIO0-1         W5300's A0-A1
// 4-7,9-12     GPIO2-9         W5300's A2-A9
// 14-17,19-22  GPIO10-17       W5300's D0-D7
// 24           GPIO18          W5300's /WR
// 25           GPIO19          W5300's /RD
// 29           GPIO22          W5300's /RST
// 31           GPIO26          W5300's /CS
// 32           GPIO27          W5300's /INT
// 34           GPIO28          PIR's signal
// 14-15        GPIO20-21       (available)

#define PIN_ETH_CS 26u   // W5300's /CS pin is connected to Pico's GPIO26
#define PIN_ETH_INTN 27u // W5300's /INT pin is connected to Pico's GPIO27
#define PIN_ETH_RST 22u  // W5300's /RST pin is connected to Pico's GPIO22

#define PIN_PIR_SIG 28u

////////////////////////////////////////////////////////////////////////////////////////////
// Enter a MAC address for your controller below.
// Newer Ethernet shields have a MAC address printed on a sticker on the shield
static byte mac[] = {
    0x00, 0xAA, 0xBB, 0xCC, 0xDE, 0x02};

static EventEthernetClient tcpClient; // demo of TCP client

static const char apiHost[] = CALLMEBOT_HOST;
static const char apiPath[] = CALLMEBOT_PATH;
static const int apiPort = 80;

// string to be sent
static char sendingText[256] = "";

static bool pirEventOccurred = false;

typedef enum _AppState
{
    StateReady,
    StateConnecting,
    StateConnected
} AppState;

static AppState appState = StateReady;

static const int DELAY_TIME_UNIT = 100;  // 100ms
static const int CONNECT_TIMEOUT = 3000; // 3000ms
static int timeoutCount = 0;

////////////////////////////////////////////////////////////////////////////////////////////
static void initEthernet(void)
{
    LOG_DEBUG("Ethernet.init(", PIN_ETH_CS, ", ", PIN_ETH_INTN, ") // W5100: /CS pin = ", PIN_ETH_CS,
              ", /INT pin = ", PIN_ETH_INTN, ", /RST pin = ", PIN_ETH_RST);
    Ethernet.init(PIN_ETH_CS, PIN_ETH_INTN, PIN_ETH_RST);

    while (Ethernet.begin(mac) == 0)
    {
        LOG_TRACE("Failed to configure Ethernet using DHCP");

        // Check for Ethernet hardware present
        if (Ethernet.hardwareStatus() == EthernetNoHardware)
        {
            LOG_ERROR("Ethernet was not found. Sorry, can't run without hardware. :(");
        }
        else if (Ethernet.linkStatus() == LinkOFF)
        {
            LOG_TRACE("Ethernet cable is not connected");
        }

        delay(1000);
    }

    LOG_TRACE("localIP(): ", Ethernet.localIP());
    LOG_TRACE("dnsServerIP(): ", Ethernet.dnsServerIP());
}
static void initPirSignal(void)
{
    const int pin = PIN_PIR_SIG;
    attachInterrupt(
        digitalPinToInterrupt(pin), []()
        {
        if (digitalRead(pin) == HIGH)
        {
            pirEventOccurred = true;
        } },
        RISING);
}

static bool startConnect(void)
{
    static int pirEventCount = 0;

    // make text data to be sent
    static char text[128];
    snprintf(text, sizeof(text), "W5300 home security demo: PIR event count %d", ++pirEventCount);
    String encoded = urlEncode(text);
    snprintf(sendingText, sizeof(sendingText), "%s%s", apiPath, encoded.c_str());
    LOG_TRACE("sendingText=", sendingText);

    // connect to host/server
    if (tcpClient.connect(apiHost, apiPort))
    {
        LOG_DEBUG("Connecting to ", tcpClient.remoteIP(), ":", tcpClient.remotePort());
        return true;
    }
    else
    {
        // if you didn't get a connection to the server:
        LOG_DEBUG("Connecting to Internet ", apiHost, ":", apiPort, " failed!");
        return false;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////
static bool readTcpPacket(void)
{
    static const int TCP_RX_BUFFER_SIZE = 1024;
    static uint8_t bufferTcpRx[TCP_RX_BUFFER_SIZE];
    bool ret = false;

    int tcpSize = tcpClient.available();
    while (tcpSize > 0)
    {
        // LOG_DEBUG("tcpSize=", tcpSize);
        if (tcpSize >= sizeof(bufferTcpRx))
        {
            tcpSize = sizeof(bufferTcpRx) - 1; // truncate data if oversize
        }
        tcpClient.read(bufferTcpRx, tcpSize);
        bufferTcpRx[tcpSize] = '\0';
        LOG_DEBUG("Received ", tcpSize, " bytes from ", tcpClient.remoteIP(), ":", tcpClient.remotePort());
        LOG_DEBUG("Content: ", (const char *)bufferTcpRx);

        tcpSize = tcpClient.available();
        ret = true;
    }
    // LOG_DEBUG("tcpSize=", tcpSize);
    return ret;
}

static void sendWhatsappMessage(void)
{
    LOG_DEBUG("sendingText=", sendingText);

    // Make a HTTP request:
    tcpClient.print("GET ");
    tcpClient.print(sendingText);
    tcpClient.println(" HTTP/1.1");
    tcpClient.print("Host: ");
    tcpClient.println(apiHost);
    tcpClient.println("Connection: close");
    tcpClient.println();
}

static void resetAppState(void)
{
    tcpClient.stop();
    timeoutCount = 0;
    pirEventOccurred = false; // signal complete of handling PIR event
    appState = StateReady;
}

// detect USB serial with timeout
// return true if USB serial is found
//        false if timeout
#define DetectUsbSerialTime 3000  // in ms
#define PollUsbSerialInterval 100 // in ms
static bool initUsbSerial(void)
{
    Serial.begin(115200);
    for (int i = 0; i < DetectUsbSerialTime; i += PollUsbSerialInterval)
    {
        if (Serial)
        {
            return true;
        }
        delay(PollUsbSerialInterval);
    }
    return false;
}

void setup(void)
{
    if (initUsbSerial())
    {
        LOG_ATTACH_SERIAL(Serial);
        LOG_SET_DELIMITER("");

        // The default log_leval is DebugLogLevel::LVL_INFO
        // 0: NONE, 1: ERROR, 2: WARN, 3: INFO, 4: DEBUG, 5: TRACE
        LOG_SET_LEVEL(DebugLogLevel::LVL_TRACE);

        PRINTLN("==========================================================================");
        PRINTLN("W5300 home security demo on Raspherry Pi Pico board");
    }
    else
    {
        LOG_SET_LEVEL(DebugLogLevel::LVL_NONE);
    }

    // init pins
    // pinMode(PIN_ETH_CS, OUTPUT);
    pinMode(PIN_ETH_INTN, INPUT);
    pinMode(PIN_ETH_RST, OUTPUT);

    initEthernet();
    initPirSignal();
}

void loop(void)
{
    switch (appState)
    {
    case StateReady:
        if (pirEventOccurred && startConnect())
        {
            appState = StateConnecting;
            timeoutCount = 0;
        }
        break;
    case StateConnecting:
        if (tcpClient.connected())
        {
            sendWhatsappMessage();
            appState = StateConnected;
        }
        else if (timeoutCount >= CONNECT_TIMEOUT)
        {
            // timeout => reset app state
            resetAppState();
        }
        break;
    case StateConnected:
        if (readTcpPacket())
        {
            // received response from callmebot => reset app state
            resetAppState();
        }
        else if (timeoutCount >= CONNECT_TIMEOUT)
        {
            // timeout => reset app state
            resetAppState();
        }
        break;
    default:
        LOG_TRACE("Unsupported appState=", appState);
        break;
    }
    delay(DELAY_TIME_UNIT);
    timeoutCount += DELAY_TIME_UNIT;
}
