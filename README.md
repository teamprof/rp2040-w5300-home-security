## Personal use home security system
Per HomeGuide (https://homeguide.com/costs/home-security-system-cost),
the average cost of a wireless home security system ranges between $200 and $2,000.
The cost includes equipment, installation and monthly fee.

This project intends to make a low cost personal use intruder alarm system.
A passive infra-red (PIR) detector acts as a motion detector to monitor intruder entry.
The detector sends a signal to a microcontroller which sends a message to cloud via Wiznet's W5300 chip.
Finally, user receives a whatsapp message about the detector signal.

---
[![open-source 3-Clause BSD License](https://img.shields.io/badge/License-BSD_3--Clause-blue.svg)](https://opensource.org/license/BSD-3-clause/)

<a href="https://www.buymeacoffee.com/teamprofnet" target="_blank"><img src="https://cdn.buymeacoffee.com/buttons/v2/default-yellow.png" alt="Buy Me A Coffee" style="height: 38px !important;width: 168px !important;" ></a>

---
## Important Notes
This project is target to build a personal-use home security proof-of-concept (PoC).
The following hardware is required:
1. A WIZ830MJ module mounted with Wiznet's W5300 TCP/IP Core Offload Engine (TOE) chip (https://www.wiznet.io/product-item/wiz830mj/)
2. A Raspberry Pi Pico (https://www.raspberrypi.com/documentation/microcontrollers/rp2040.html)
3. A LED lamp with build-in Passive Infra-Red (PIR) detector (https://item.taobao.com/item.htm?spm=a1z09.2.0.0.342b2e8dz8H8RM&id=638048049460&_u=b3no53d890f9)
4. [optional] A WIZ830MJ-to-Pico DIY connection board. Notice that user may connect WIZ830MJ to Pi Pico board by wire wrapping, as connection schematic is available online. (please contact teamprof.net@gmail.com in case you prefer to buy the connector)
5. A Internet accessible router with Ethernet port
6. A mobile phone with Whatsapp APP (https://www.whatsapp.com/) installed and configured.

The following software is required to build and run firmware on Raspberry Pi Pico (RP2040)
1. [Arduino IDE 1.8.19+ for Arduino](https://www.arduino.cc/en/Main/Software)
2. [Arduino mbed_rp2040 core 4.0.2+](https://github.com/arduino/ArduinoCore-mbed) for [Raspberry Pi Pico]
3. [Arduino DebugLog lib](https://www.arduino.cc/reference/en/libraries/debuglog/)
4. [Arduino UrlEncode lib](https://github.com/plageoj/urlencode)
---

### Proof of Concept explanation
A LED lamp build-in a passive infra-red (PIR) detector, the PIR detector acts as a motion detector to monitor intruder entry.
Once the detector detected an intruder entry, it sends a signal to microcontroller (MCU). 
MCU invokes callmebot API to send a whatsapp message via Wiznet's W5300 Ethernet chip.
Finally, user receives a whatsapp message about the detector signal.

```
+------+        +-----+    +-------+             +-----------+     +----------+
| PIR  | -----> | MCU | -> | W5300 | ----------> | Callmebot | ... | mobile's |
| lamp |  PIR   |     |    |       |  ethernet   |           |     | whatsapp |
+------+ signal +-----+    +-------+             +-----------+     +----------+
                
```

### System diagram 
[![system diagram](/doc/images/system-diagram.jpg)](https://github.com/teamprof/rp2040-w5300-HomeSecurity/blob/main/doc/system-diagram.jpg)

---
### Hardware setup (WIZ830-RP2040 connection)
If you have a WIZ830MJ-RP2040 connector board, simply plug Raspberry Pi Pico and WIZ830MJ module on the connector board's front and back side respectively.

Front view  
[![front side of connector)](/doc/images/wiz830mj-rp2040-front.jpg)](https://github.com/teamprof/rp2040-w5300-HomeSecurity/blob/main/doc/wiz830mj-rp2040-front.jpg)

Back view  
[![back side of connector)](/doc/images/wiz830mj-rp2040-back.jpg)](https://github.com/teamprof/rp2040-w5300-HomeSecurity/blob/main/doc/wiz830mj-rp2040-back.jpg)

Side view  
[![side view of connector)](/doc/images/wiz830mj-rp2040-side.jpg)](https://github.com/teamprof/rp2040-w5300-HomeSecurity/blob/main/doc/wiz830mj-rp2040-side.jpg)


Otherwise, study the connector board schematic and wire wrapping the connection between Pi Pico and WIZ830MJ.
(pins: VCC, GND, pirSig, /RESET, /CS, /WR, /RD, /INT, ADDR[0-9], DATA[0:9])  
[![connection diagram](/doc/images/connection-diagram.jpg)](https://github.com/teamprof/rp2040-w5300-HomeSecurity/blob/main/doc/connection-diagram.jpg)

### Photo of wire wrap board (top view)  
[![photo of wire wrap board (top)](/doc/images/wiz830mj-rp2040-breadboard-front.jpg)](https://github.com/teamprof/rp2040-w5300-HomeSecurity/blob/main/doc/wiz830mj-rp2040-breadboard-front.jpg)

### Photo of wire wrap board (bottom view)  
[![photo of wire wrap board (bottom)](/doc/images/wiz830mj-rp2040-breadboard-back.jpg)](https://github.com/teamprof/rp2040-w5300-HomeSecurity/blob/main/doc/wiz830mj-rp2040-breadboard-back.jpg)


### Hardware setup (RP2040-PIR connection)
Connect the PIR detector output by wire wrapping NSA3182FT180's pin5 to Pi Pico GP28.  
[![PIR lamp photo 1](/doc/images/pir-lamp-01.jpg)](https://github.com/teamprof/rp2040-w5300-HomeSecurity/blob/main/doc/images/pir-lamp-01.jpg)

Since NSA3182FT180's pin5 is connected to R9, it is easier to soldering a wire on R9.  
[![PIR lamp photo 2](/doc/images/pir-lamp-02.jpg)](https://github.com/teamprof/rp2040-w5300-HomeSecurity/blob/main/doc/images/pir-lamp-02.jpg)

### Complete setup (connector)
[![complete-setup-connector](/doc/images/w5300-rp2040-demo-setup-connector.jpg)](https://github.com/teamprof/arduino-eventethernet/blob/main/doc/images/w5300-rp2040-demo-setup-connector.jpg)

### Complete setup (wire wrap)
[![complete-setup-wire-wrap](/doc/images/w5300-rp2040-demo-setup-wire-wrap.jpg)](https://github.com/teamprof/arduino-eventethernet/blob/main/doc/images/w5300-rp2040-demo-setup-wire-wrap.jpg)

---
## Software setup
1. Install [Arduino IDE 1.8.19+ for Arduino](https://www.arduino.cc/en/Main/Software)
2. Install [Arduino mbed_rp2040 core 4.0.2+](https://github.com/arduino/ArduinoCore-mbed) for [Raspberry Pi Pico]
3. Install [Arduino DebugLog lib](https://www.arduino.cc/reference/en/libraries/debuglog/)
4. Install [Arduino UrlEncode lib](https://github.com/plageoj/urlencode)
5. Download and extract the rp2040-w5300-HomeSecurity source code from github 

1. Follow the steps on callmebot (https://www.callmebot.com/blog/free-api-whatsapp-messages/) to get credentials for sending whatsapp message
2. Edit the file "secret.h" and replace "MobileNumber" and "ApiKey" by your mobile number and callmebot's API key. 
3. Launch Arduino IDE, click menu "Sketch" -> "Verify/Compile"
4. Connect WIZ530MJ to router(Internet) with a Ethernet cable. 
5. Connect PC and Raspberry Pi Pico with a USB cable. 
6. click menu "Tools" -> "Port" to select the serial port of Raspberry Pi Pico
7. click menu "Sketch" -> "Upload" 
8. If everything goes smooth, you should see the followings on Serial Terminal (e.g. GTKTerm)
[![bootup log](/doc/images/bootup-log.png)](https://github.com/teamprof/rp2040-w5300-HomeSecurity/blob/main/doc/bootup-log.png)
---

## Code Example

### pin assignment
pin assignment is defined in "rp2040-w5300-HomeSecurity.ino" and "./src/lib/EventEthernet/src/EventEthernet.h"
```
// rp2040-w5300-HomeSecurity.ino
#define PIN_ETH_CS 26u   // W5300's /CS pin is connected to Pico's GPIO26
#define PIN_ETH_INTN 27u // W5300's /INT pin is connected to Pico's GPIO27
#define PIN_ETH_RST 22u  // W5300's /INT pin is connected to Pico's GPIO22

// ./src/lib/EventEthernet/src/EventEthernet.h
#define PIN_CSn 26u	 // W5500: nCS pin = GPIO26 (WIZ830MJ-RP2040 connector)
#define Pin_Intn 27u // W5500: INTn pin = GPIO27 (WIZ830MJ-RP2040 connector)
#define Pin_A0 0u	 // W5500: A0 pin = GPIO0 (WIZ830MJ-RP2040 connector)
#define Pin_A1 1u	 // W5500: A1 pin = GPIO1 (WIZ830MJ-RP2040 connector)

// note: A2-A9 is overlap with D8-D15
#define Pin_A2 2u	 // W5500: A2 pin = GPIO2 (WIZ830MJ-RP2040 connector)
#define Pin_A3 3u	 // W5500: A3 pin = GPIO3 (WIZ830MJ-RP2040 connector)
#define Pin_A4 4u	 // W5500: A4 pin = GPIO4 (WIZ830MJ-RP2040 connector)
#define Pin_A5 5u	 // W5500: A5 pin = GPIO5 (WIZ830MJ-RP2040 connector)
#define Pin_A6 6u	 // W5500: A6 pin = GPIO6 (WIZ830MJ-RP2040 connector)
#define Pin_A7 7u	 // W5500: A7 pin = GPIO7 (WIZ830MJ-RP2040 connector)
#define Pin_A8 8u	 // W5500: A8 pin = GPIO8 (WIZ830MJ-RP2040 connector)
#define Pin_A9 9u	 // W5500: A9 pin = GPIO9 (WIZ830MJ-RP2040 connector)

#define Pin_D0 10u // W5500: D0 pin = GPIO10 (WIZ830MJ-RP2040 connector)
#define Pin_D1 11u // W5500: D1 pin = GPIO11 (WIZ830MJ-RP2040 connector)
#define Pin_D2 12u // W5500: D2 pin = GPIO12 (WIZ830MJ-RP2040 connector)
#define Pin_D3 13u // W5500: D3 pin = GPIO13 (WIZ830MJ-RP2040 connector)
#define Pin_D4 14u // W5500: D4 pin = GPIO14 (WIZ830MJ-RP2040 connector)
#define Pin_D5 15u // W5500: D5 pin = GPIO15 (WIZ830MJ-RP2040 connector)
#define Pin_D6 20u // W5500: D6 pin = GPIO20 (WIZ830MJ-RP2040 connector)
#define Pin_D7 21u // W5500: D7 pin = GPIO21 (WIZ830MJ-RP2040 connector)

#define Pin_Wr 18u // W5500: /WR pin = GPIO18 (WIZ830MJ-RP2040 connector)
#define Pin_Rd 19u // W5500: /RD pin = GPIO19 (WIZ830MJ-RP2040 connector)
```
### init WIZ830MJ (W5300) and PIR signal pin
```
void setup()
{
    // ...

    initEthernet();
    initPirSignal();

    // ...
}

static void initEthernet(void)
{
    // ...

    Ethernet.init(PIN_ETH_CS, PIN_ETH_INTN, PIN_ETH_RST);

    // ...

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

    // ...
}
```

Once PIR detected an intruder entry, it raised up the NSA3182FT180's pin5 to HIGH, the attached "Interrupt Service Routine" in "initPirSignal()" set a global variable "pirEventOccurred" to "true".
```
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
```
### Arduino's main loop
Arduino's main loop checks the value of of "pirEventOccurred", and start connection to callmebot once the value is HIGH.
```
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

    ...
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

Once connection between W5300 and callmebot is established, RP2040 sends HTTP REQUEST to callmebot via W5300, a whatsapp message should be received by the user if success.
If connection is NOT established within the "CONNECT_TIMEOUT" period, RP2040 reset the Application State.
void loop(void)
{
    switch (appState)
    {
    ...

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

    ...
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
```
### Please refer to "rp2040-w5300-HomeSecurity.ino" for complete code
## *** source code will be available by End/Jun ***
---

## Demo
Video demo is available on  
1. [WIZ830MJ-RP2040 connector](https://www.youtube.com/watch?v=f5R64rBVWaA)  

2. [wire-wrap board](https://www.youtube.com/watch?v=p3mZKC2UZdk)  

---
### Debug
Enable or disable log be modifying macro on "AppLog.h"

Debug is disabled by "#define DEBUGLOG_DISABLE_LOG"
Enable trace debug by "#define DEBUGLOG_DEFAULT_LOG_LEVEL_TRACE"

Example of AppLog.h
```
// Disable Logging Macro (Release Mode)
// #define DEBUGLOG_DISABLE_LOG
// You can also set default log level by defining macro (default: INFO)
// #define DEBUGLOG_DEFAULT_LOG_LEVEL_WARN // for release version
#define DEBUGLOG_DEFAULT_LOG_LEVEL_TRACE // for debug version
#include <DebugLog.h>                    // https://github.com/hideakitai/DebugLog
```
---
### Troubleshooting
If you get compilation errors, more often than not, you may need to install a newer version of the core for Arduino boards.

Sometimes, the project will only work if you update the board core to the latest version because I am using newly added functions.

---
### Issues
Submit issues to: [rp2040-w5300-homesecurity issues](https://github.com/teamprof/rp2040-w5300-homesecurity/issues) 

---
### TO DO
1. Search for bug and improvement.
---

### Contributions and Thanks

Many thanks for everyone for bug reporting, new feature suggesting, testing and contributing to the development of this project.
---

### Contributing
If you want to contribute to this project:

- Report bugs and errors
- Ask for enhancements
- Create issues and pull requests
- Tell other people about this library
---

### License
- The library is licensed under open-source 3-Clause BSD License (https://opensource.org/license/BSD-3-clause/)
---

### Copyright
- Copyright 2023 teamprof.net@gmail.com. All righs reserved.

