/*
 Repeating Web client

 This sketch connects to a web server and makes a request
 using a WIZnet Ethernet shield. You can use the Arduino Ethernet Shield, or
 the Adafruit Ethernet shield, either one will work, as long as it's got
 a WIZnet Ethernet module on board.

 This example uses DNS, by assigning the Ethernet client with a MAC address,
 IP address, and DNS address.

 Circuit:
 * Ethernet shield attached to pins 10, 11, 12, 13

 created 19 Apr 2012
 by Tom Igoe
 modified 21 Jan 2014
 by Federico Vanzati

 https://www.arduino.cc/en/Tutorial/WebClientRepeating
 This code is in the public domain.

 */

////////////////////////////////////////////////////////////////////////////////////////////
// select the Wiznet's Ethernet chip by modifying the W5x00 macro in EventEthernet.h
// note: default is W5100S_MODULE
// #include <EventEthernet.h>
#include "./src/lib/EventEthernet/src/EventEthernet.h"
////////////////////////////////////////////////////////////////////////////////////////////

#define PIN_ETH_CS 26u   // W5300's /CS pin is connected to Pico's GPIO26
#define PIN_ETH_INTN 27u // W5300's /INT pin is connected to Pico's GPIO27
#define PIN_ETH_RST 22u  // W5300's /RST pin is connected to Pico's GPIO22

// Enter a MAC address for your controller below.
// Newer Ethernet shields have a MAC address printed on a sticker on the shield
static byte mac[] = {
    0x00, 0xAA, 0xBB, 0xCC, 0xDE, 0x02};

// Set the static IP address to use if the DHCP fails to assign
static IPAddress ip(192, 168, 0, 177);
static IPAddress myDns(192, 168, 0, 1);

static EventEthernetClient client; // demo of TCP client

static const char server[] = "www.arduino.cc"; // also change the Host line in httpRequest()
// IPAddress server(64,131,82,241);

static unsigned long lastConnectionTime = 0;            // last time you connected to the server, in milliseconds
static const unsigned long postingInterval = 10 * 1000; // delay between updates, in milliseconds

///////////////////////////////////////////////////////////////////////////////
void setup()
{
    // You can use Ethernet.init(pin) to configure the CS pin
    // Ethernet.init(10);  // Most Arduino shields
    // Ethernet.init(5);   // MKR ETH Shield
    // Ethernet.init(0);   // Teensy 2.0
    // Ethernet.init(20);  // Teensy++ 2.0
    // Ethernet.init(15);  // ESP8266 with Adafruit FeatherWing Ethernet
    // Ethernet.init(33);  // ESP32 with Adafruit FeatherWing Ethernet
    Ethernet.init(PIN_ETH_CS, PIN_ETH_INTN, PIN_ETH_RST);

    Serial.begin(115200);
    while (!Serial)
    {
        delay(100);
    }
    Serial.print("Ethernet.init(");
    Serial.print(PIN_ETH_CS);
    Serial.print(", ");
    Serial.print(PIN_ETH_INTN);
    Serial.print("), /CS pin = ");
    Serial.print(PIN_ETH_CS);
    Serial.print(", /INT pin = ");
    Serial.print(PIN_ETH_INTN);
    Serial.print(", /RST pin = ");
    Serial.println(PIN_ETH_RST);

    // start the Ethernet connection:
    Serial.println("Initialize Ethernet with DHCP:");
    if (Ethernet.begin(mac) == 0)
    {
        Serial.println("Failed to configure Ethernet using DHCP");
        // Check for Ethernet hardware present
        if (Ethernet.hardwareStatus() == EthernetNoHardware)
        {
            Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
            while (true)
            {
                delay(1); // do nothing, no point running without Ethernet hardware
            }
        }
        if (Ethernet.linkStatus() == LinkOFF)
        {
            Serial.println("Ethernet cable is not connected.");
        }
        // try to configure using IP address instead of DHCP:
        Ethernet.begin(mac, ip, myDns);
        Serial.print("My IP address: ");
        Serial.println(Ethernet.localIP());
    }
    else
    {
        Serial.print("  DHCP assigned IP ");
        Serial.println(Ethernet.localIP());
    }
    // give the Ethernet shield a second to initialize:
    delay(1000);
}

// this method makes a HTTP connection to the server:
static void httpRequest()
{
    // close any connection before send a new request.
    // This will free the socket on the Ethernet shield
    client.stop();

    // if there's a successful connection:
    if (client.connect(server, 80))
    {
        Serial.println("connecting...");
        // send the HTTP GET request:
        client.println("GET /latest.txt HTTP/1.1");
        client.println("Host: www.arduino.cc");
        client.println("User-Agent: arduino-ethernet");
        client.println("Connection: close");
        client.println();

        // note the time that the connection was made:
        lastConnectionTime = millis();
    }
    else
    {
        // if you couldn't make a connection:
        Serial.println("connection failed");
    }
}

void loop()
{
    // if there's incoming data from the net connection.
    // send it out the serial port.  This is for debugging
    // purposes only:
    if (client.available())
    {
        char c = client.read();
        Serial.write(c);
    }

    // if ten seconds have passed since your last connection,
    // then connect again and send data:
    if (millis() - lastConnectionTime > postingInterval)
    {
        httpRequest();
    }
}
