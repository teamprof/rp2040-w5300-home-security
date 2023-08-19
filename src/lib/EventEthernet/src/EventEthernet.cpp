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
#include <Arduino.h>
#include <mbed.h>
#include "./EventEthernet.h"
// #include "Ethernet.h"
#include "./utility/w5300.h"

///////////////////////////////////////////////////////////////////////////////
// SocketEventApi
///////////////////////////////////////////////////////////////////////////////
static const uint8_t socketIndexToImrBitTable[] = {
	0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80 // for W5300
};

SocketEventApi::SocketEventApi()
{
	// sockIndex = MAX_SOCK_NUM;
	sn_ir = 0;
	eventCallback = nullptr;
}
SocketEventApi::~SocketEventApi()
{
	// unsubscribeSocketEvent(sn_ir, sockIndex);
	// sockIndex = MAX_SOCK_NUM;
	sn_ir = 0;
	eventCallback = nullptr;
}

bool SocketEventApi::subscribeSocketEvent(uint16_t sn_ir, SocketEventCallback onSocketEvent, uint8_t sockindex)
{
	// LOG_TRACE("sn_ir = ", DebugLogBase::HEX, sn_ir, "(hex), onSocketEvent = ", DebugLogBase::HEX, (uint32_t)(onSocketEvent), "(hex), sockindex = ", DebugLogBase::DEC, sockindex);

	bool ret = false;
	sn_ir = sn_ir & SnIR_MASK;
	if (sn_ir && (onSocketEvent != nullptr) && sockindex < MAX_SOCK_NUM)
	{
		// this->sockIndex = sockindex;
		this->eventCallback = onSocketEvent;
		ret = Ethernet.registerSocketCallback(sockindex, onSocketEvent);

		uint16_t sn_imr = W5300.getSnMR(sockindex);
		if (sn_ir & Sn_IR_PRECV)
		{
			sn_imr |= Sn_IR_PRECV;
		}
		if (sn_ir & Sn_IR_PFAIL)
		{
			sn_imr |= Sn_IR_PFAIL;
		}
		if (sn_ir & Sn_IR_PNEXT)
		{
			sn_imr |= Sn_IR_PNEXT;
		}
		if (sn_ir & Sn_IR_SENDOK)
		{
			sn_imr |= Sn_IR_SENDOK;
		}
		if (sn_ir & Sn_IR_TIMEOUT)
		{
			sn_imr |= Sn_IR_TIMEOUT;
		}
		if (sn_ir & Sn_IR_RECV)
		{
			sn_imr |= Sn_IR_RECV;
		}
		if (sn_ir & Sn_IR_DISCON)
		{
			sn_imr |= Sn_IR_DISCON;
		}
		if (sn_ir & Sn_IR_CON)
		{
			sn_imr |= Sn_IR_CON;
		}
		W5300.setSnMR(sockindex, sn_imr);

		uint16_t imr = W5300.readIMR();
		imr |= socketIndexToImrBitTable[sockindex];
		W5300.writeIMR(imr);

		this->sn_ir |= sn_ir;
		// LOG_TRACE("this->sn_ir = ", DebugLogBase::HEX, this->sn_ir, "(hex), this->eventCallback = ", DebugLogBase::HEX, (uint32_t)(this->eventCallback), "(hex)");
	}
	return ret;
}

void SocketEventApi::unsubscribeSocketEvent(uint16_t sn_ir, uint8_t sockindex)
{
	// LOG_TRACE("sn_ir = ", DebugLogBase::HEX, sn_ir, "(hex), sockindex = ", DebugLogBase::DEC, sockindex);

	sn_ir = sn_ir & SnIR_MASK;
	if (sn_ir && sockindex < MAX_SOCK_NUM)
	{
		uint16_t sn_imr = W5300.getSnMR(sockindex);
		if (sn_ir & Sn_IR_PRECV)
		{
			sn_imr &= ~Sn_IR_PRECV;
		}
		if (sn_ir & Sn_IR_PFAIL)
		{
			sn_imr &= ~Sn_IR_PFAIL;
		}
		if (sn_ir & Sn_IR_PNEXT)
		{
			sn_imr &= ~Sn_IR_PNEXT;
		}
		if (sn_ir & Sn_IR_SENDOK)
		{
			sn_imr &= ~Sn_IR_SENDOK;
		}
		if (sn_ir & Sn_IR_TIMEOUT)
		{
			sn_imr &= ~Sn_IR_TIMEOUT;
		}
		if (sn_ir & Sn_IR_RECV)
		{
			sn_imr &= ~Sn_IR_RECV;
		}
		if (sn_ir & Sn_IR_DISCON)
		{
			sn_imr &= ~Sn_IR_DISCON;
		}
		if (sn_ir & Sn_IR_CON)
		{
			sn_imr &= ~Sn_IR_CON;
		}
		W5300.setSnMR(sockindex, sn_imr);

		if (!sn_imr)
		{
			uint16_t imr = W5300.readIMR();
			imr &= ~(socketIndexToImrBitTable[sockindex]);
			W5300.writeIMR(imr);
		}

		this->sn_ir &= ~sn_ir;
		// this->sockIndex = MAX_SOCK_NUM;
		if (!this->sn_ir)
		{
			Ethernet.unregisterSocketCallback(sockindex);
			this->eventCallback = nullptr;
		}
		// LOG_TRACE("this->sn_ir = ", DebugLogBase::HEX, this->sn_ir, "(hex), this->eventCallback = ", DebugLogBase::HEX, (uint32_t)(this->eventCallback), "(hex)");
	}
}

///////////////////////////////////////////////////////////////////////////////
// EventEthernetServer
///////////////////////////////////////////////////////////////////////////////
EventEthernetServer::EventEthernetServer(uint16_t port) : EthernetServer(port)
{
	memset(server_port, 0, sizeof(server_port));
	this->_serverPort = port;
}
EventEthernetServer::~EventEthernetServer()
{
	unsubscribeSocketEvent(this->sn_ir);
	this->_serverPort = 0;
}

EventEthernetClient EventEthernetServer::available()
{
	bool listening = false;
	uint8_t sockindex = MAX_SOCK_NUM;

	// W5100 chip does not support more than 4 sockets
	uint8_t chip = W5300.getChip();
	int maxindex = (chip == 51 || chip == 50) ? 4 : MAX_SOCK_NUM;
	for (uint8_t i = 0; i < maxindex; i++)
	{
		if (server_port[i] == _port)
		{
			uint8_t stat = Ethernet.socketStatus(i);
			if (stat == SnSR::ESTABLISHED || stat == SnSR::CLOSE_WAIT)
			{
				if (Ethernet.socketRecvAvailable(i) > 0)
				{
					sockindex = i;
				}
				else
				{
					// remote host closed connection, our end still open
					if (stat == SnSR::CLOSE_WAIT)
					{
						Ethernet.socketDisconnect(i);
						// status becomes LAST_ACK for short time
					}
				}
			}
			else if (stat == SnSR::LISTEN)
			{
				listening = true;
			}
			else if (stat == SnSR::CLOSED)
			{
				server_port[i] = 0;
			}
		}
	}

	if (!listening)
	{
		LOG_TRACE("socket is NOT listening! sockindex = ", sockindex);
		LOG_TRACE("_port = ", _port);
		LOG_TRACE("server_port: ", LOG_AS_ARR(server_port, MAX_SOCK_NUM));
		begin(this->sn_ir, this->eventCallback);
	}
	LOG_TRACE("return EventEthernetClient(", sockindex, ")");

	return EventEthernetClient(sockindex);
}

EventEthernetClient EventEthernetServer::accept()
{
	bool listening = false;
	uint8_t sockindex = MAX_SOCK_NUM;

	// W5100 chip does not support more than 4 sockets
	uint8_t chip = W5300.getChip();
	int maxindex = (chip == 51 || chip == 50) ? 4 : MAX_SOCK_NUM;
	for (uint8_t i = 0; i < maxindex; i++)
	{
		if (server_port[i] == _serverPort)
		{
			uint8_t stat = Ethernet.socketStatus(i);
			if (sockindex == MAX_SOCK_NUM &&
				(stat == SnSR::ESTABLISHED || stat == SnSR::CLOSE_WAIT))
			{
				// Return the connected client even if no data received.
				// Some protocols like FTP expect the server to send the
				// first data.
				sockindex = i;
				server_port[i] = 0; // only return the client once
			}
			else if (stat == SnSR::LISTEN)
			{
				listening = true;
			}
			else if (stat == SnSR::CLOSED)
			{
				server_port[i] = 0;
			}
		}
	}

	if (!listening)
	{
		LOG_TRACE("socket is NOT listening! sockindex = ", sockindex);
		LOG_TRACE("_port = ", _port);
		LOG_TRACE("server_port: ", LOG_AS_ARR(server_port, MAX_SOCK_NUM));
		begin(this->sn_ir, this->eventCallback);
	}
	if (sockindex >= MAX_SOCK_NUM)
	{
		LOG_TRACE("_port = ", _port);
		LOG_TRACE("server_port: ", LOG_AS_ARR(server_port, MAX_SOCK_NUM));
	}
	LOG_TRACE("return EventEthernetClient(", sockindex, ")");

	return EventEthernetClient(sockindex);
}
void EventEthernetServer::begin(uint16_t sn_ir, SocketEventCallback onSocketEvent)
{
	EthernetServer::begin();
	subscribeSocketEvent(sn_ir, onSocketEvent);
}

bool EventEthernetServer::subscribeSocketEvent(uint16_t sn_ir, SocketEventCallback onSocketEvent)
{
	bool ret = true;
	if (sn_ir && (onSocketEvent != nullptr))
	{
		for (uint8_t index = 0; index < MAX_SOCK_NUM; index++)
		{
			if (server_port[index] == _serverPort)
			{
				ret = SocketEventApi::subscribeSocketEvent(sn_ir, onSocketEvent, index);
			}
		}
	}
	return ret;
}
void EventEthernetServer::unsubscribeSocketEvent(uint16_t sn_ir)
{
	// W5100 chip does not support more than 4 sockets
	uint8_t chip = W5300.getChip();
	int maxindex = (chip == 51 || chip == 50) ? 4 : MAX_SOCK_NUM;
	for (uint8_t index = 0; index < maxindex; index++)
	{
		if (server_port[index] == _serverPort)
		{
			SocketEventApi::unsubscribeSocketEvent(sn_ir, index);
		}
	}
}

///////////////////////////////////////////////////////////////////////////////
// EventEthernetClient
///////////////////////////////////////////////////////////////////////////////
int EventEthernetClient::connect(IPAddress ip, uint16_t port, uint16_t sn_ir, SocketEventCallback onSocketEvent)
{
	int ret = EthernetClient::connect(ip, port);
	if (ret)
	{
		SocketEventApi::subscribeSocketEvent(sn_ir, onSocketEvent, getSocketNumber());
	}
	return ret;
}
int EventEthernetClient::connect(const char *host, uint16_t port, uint16_t sn_ir, SocketEventCallback onSocketEvent)
{
	int ret = EthernetClient::connect(host, port);
	// LOG_TRACE("host=", host, ", port=", port, ", ret=", ret);
	if (ret)
	{
		SocketEventApi::subscribeSocketEvent(sn_ir, onSocketEvent, getSocketNumber());
	}
	return ret;
}
void EventEthernetClient::stop()
{
	SocketEventApi::unsubscribeSocketEvent(this->sn_ir, getSocketNumber());
	EthernetClient::stop();
}

///////////////////////////////////////////////////////////////////////////////
// EventEthernetUDP
///////////////////////////////////////////////////////////////////////////////
void EventEthernetUDP::begin(uint16_t port, uint16_t sn_ir, SocketEventCallback onSocketEvent)
{
	uint8_t ret = EthernetUDP::begin(port);
	// LOG_TRACE("EthernetUDP::begin(", port, ") returns ", ret, ", sn_ir=(hex)", DebugLogBase::HEX, sn_ir);
	if (ret)
	{
		SocketEventApi::subscribeSocketEvent(sn_ir, onSocketEvent, this->sockindex);
	}
}

uint8_t EventEthernetUDP::beginMulticast(IPAddress ip, uint16_t port, uint16_t sn_ir, SocketEventCallback onSocketEvent)
{
	uint8_t ret = EthernetUDP::beginMulticast(ip, port);
	if (ret)
	{
		SocketEventApi::subscribeSocketEvent(sn_ir, onSocketEvent, this->sockindex);
	}
	return ret;
}
void EventEthernetUDP::stop(void)
{
	SocketEventApi::unsubscribeSocketEvent(this->sn_ir, this->sockindex);
	EthernetUDP::stop();
}

///////////////////////////////////////////////////////////////////////////////
// EventEthernetClass
///////////////////////////////////////////////////////////////////////////////
EventEthernetClass *EventEthernetClass::instance = nullptr;
// extern EventEthernetClass *instance __attribute__((weak));

#if defined(TARGET_RASPBERRY_PI_PICO)
#define EventEthernetThreadQueueSize (256 * EVENTS_EVENT_SIZE)
EventEthernetClass::EventEthernetClass() : queue(EventEthernetThreadQueueSize)
// EventEthernetClass::EventEthernetClass() : queue(EventEthernetThreadQueueSize), ethSemaphore(1)
{
	instance = this;
	pinIntn = -1;
	pinRst = -1;
	ir = 0;

	// attachInterrupt(pinIntn, isr, FALLING, this);
	thread.start(callback(&queue, &EventQueue::dispatch_forever));
}
#elif defined(ESP32)
#error "Not supported"
#endif
EventEthernetClass::~EventEthernetClass()
{
	if (pinIntn >= 0)
	{
		detachInterrupt(pinIntn);
	}
	instance = nullptr;
	unsubscribeEthernetEvent(ir);
}

int EventEthernetClass::begin(uint8_t *mac, uint16_t ir, EthernetEventCallback onEthernetEvent)
{
	int ret = EthernetClass::begin(mac);
	if (ret)
	{
		Ethernet.subscribeEthernetEvent(ir, onEthernetEvent);
	}
	return ret;
}

int EventEthernetClass::begin(uint8_t *mac, unsigned long timeout, unsigned long responseTimeout, uint8_t ir, EthernetEventCallback onEthernetEvent)
{
	int ret = EthernetClass::begin(mac, timeout, responseTimeout);
	if (ret)
	{
		Ethernet.subscribeEthernetEvent(ir, onEthernetEvent);
	}
	return ret;
}

void EventEthernetClass::begin(uint8_t *mac, IPAddress ip, uint8_t ir, EthernetEventCallback onEthernetEvent)
{
	EthernetClass::begin(mac, ip);
	Ethernet.subscribeEthernetEvent(ir, onEthernetEvent);
}

void EventEthernetClass::begin(uint8_t *mac, IPAddress ip, IPAddress dns, uint8_t ir, EthernetEventCallback onEthernetEvent)
{
	EthernetClass::begin(mac, ip, dns);
	Ethernet.subscribeEthernetEvent(ir, onEthernetEvent);
}

void EventEthernetClass::begin(uint8_t *mac, IPAddress ip, IPAddress dns, IPAddress gateway, uint8_t ir, EthernetEventCallback onEthernetEvent)
{
	EthernetClass::begin(mac, ip, dns, gateway);
	Ethernet.subscribeEthernetEvent(ir, onEthernetEvent);
}

void EventEthernetClass::begin(uint8_t *mac, IPAddress ip, IPAddress dns, IPAddress gateway, IPAddress subnet, uint8_t ir, EthernetEventCallback onEthernetEvent)
{
	EthernetClass::begin(mac, ip, dns, gateway, subnet);
	Ethernet.subscribeEthernetEvent(ir, onEthernetEvent);
}

void EventEthernetClass::init(uint8_t sspin, int8_t pinIntn, int8_t pinRst)
{
	EthernetClass::init(sspin);
	this->pinIntn = pinIntn;
	if (pinIntn >= 0)
	{
		// Wiznet W5300 INTn is a level interrupt (active LOW),
		// attach as "FALLING" instead of "LOW" is a workaround
		// to prevent infinite interrupts during the time between isr and queueIsr
#if defined(TARGET_RASPBERRY_PI_PICO)
		attachInterrupt(digitalPinToInterrupt(pinIntn), isr, FALLING, this);
		// attachInterrupt(pinIntn, isr, FALLING, instance);
#elif defined(ESP32)
		attachInterrupt(pinIntn, std::bind(&EventEthernetClass::isr, this), FALLING);
#endif
	}

	this->pinRst = pinRst;
	if (pinRst >= 0)
	{
		pinMode(pinRst, OUTPUT);
	}

	hardReset();
}

void EventEthernetClass::hardReset(void)
{
	if (pinRst >= 0)
	{
		// assert low for 2us+
		digitalWrite(pinRst, LOW);
		delayMicroseconds(2);

		// de-assert for 10ms
		digitalWrite(pinRst, HIGH);

		delayMilliSecond(10);

		uint16_t mr = W5300.getMR();
		LOG_TRACE("data bus width=", (mr & MR_DBW) ? "16-bit" : "8-bit",
				  ", data bus swap=", (mr & MR_DBS) ? "enable" : "disable",
				  ", bus interface mode=", (mr & MR_DBS) ? "Indirect address Mode" : "Direct address Mode",
				  ", mr=(hex)", DebugLogBase::HEX, mr);
	}
}

#define dim(x) (sizeof(x) / sizeof(x[0]))
void EventEthernetClass::subscribeEthernetEvent(uint16_t ir, EthernetEventCallback onEthernetEvent)
{
	// mask Sn_INT which is enable/disable in registerSocketCallback()/unregisterSocketCallback()
	ir &= ~Sn_INT;

	this->ir |= ir;

	if (ir)
	{
		ethernetEventCallback = onEthernetEvent;
	}

	uint16_t imr = W5300.readIMR();
	if (ir & IR_IPCF)
	{
		imr |= IR_IPCF;
	}
	if (ir & IR_DPUR)
	{
		imr |= IR_DPUR;
	}
	if (ir & IR_PPPT)
	{
		imr |= IR_PPPT;
	}
	if (ir & IR_FMTU)
	{
		imr |= IR_FMTU;
	}
	W5300.writeIMR(imr);
	// LOG_TRACE("this->ir=(hex)", DebugLogBase::HEX, this->ir);
}

void EventEthernetClass::unsubscribeEthernetEvent(uint16_t ir)
{
	uint8_t imr = W5300.readIMR();
	if (ir & IR_IPCF)
	{
		imr &= ~IR_IPCF;
	}
	if (ir & IR_DPUR)
	{
		imr &= ~IR_DPUR;
	}
	if (ir & IR_PPPT)
	{
		imr &= ~IR_PPPT;
	}
	if (ir & IR_FMTU)
	{
		imr &= ~IR_FMTU;
	}
	W5300.writeIMR(imr);

	// ignore Sn_INT which is handled by registerSocketCallback()/unregisterSocketCallback()
	ir &= ~Sn_INT;
	imr &= ~Sn_INT;
	if (!imr)
	{
		ethernetEventCallback = nullptr;
	}

	this->ir &= ~ir;
	// LOG_TRACE("this->ir=(hex)", DebugLogBase::HEX, this->ir);
}

bool EventEthernetClass::registerSocketCallback(uint8_t sockindex, SocketEventApi::SocketEventCallback onSocketEvent)
{
	if (sockindex < MAX_SOCK_NUM)
	{
		socketEventCallback[sockindex] = onSocketEvent;
		return true;
	}
	return false;
}
void EventEthernetClass::unregisterSocketCallback(uint8_t sockindex)
{
	if (sockindex < MAX_SOCK_NUM)
	{
		socketEventCallback[sockindex] = nullptr;
	}
}

void EventEthernetClass::isr(EventEthernetClass *instance)
{
	if (instance != nullptr)
	{
#if defined(TARGET_RASPBERRY_PI_PICO)
		auto ev = instance->queue.event(instance, &EventEthernetClass::queueIsr);
		ev.post();
#elif defined(ESP32)
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		BaseType_t xResult = xEventGroupSetBitsFromISR(
			instance->eventGroupHandle,
			EVENT_BIT_ETHERNET, /* The bits being set. */
			&xHigherPriorityTaskWoken);

		/* Was the message posted successfully? */
		if (xResult != pdFAIL)
		{
			/* If xHigherPriorityTaskWoken is now set to pdTRUE then a context
			switch should be requested.  The macro used is port specific and will
			be either portYIELD_FROM_ISR() or portEND_SWITCHING_ISR() - refer to
			the documentation page for the port being used. */
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		}
#endif
	}
}
// void EventEthernetClass::isr(EventEthernetClass *inst)
// {
// 	if (instance != nullptr)
// 	{
// 		auto ev = instance->queue.event(instance, &EventEthernetClass::queueIsr);
// 		ev.post();
// 	}
// }

void EventEthernetClass::queueIsr(void)
{
	while (digitalRead(pinIntn) == LOW)
	{
		uint8_t chip = W5300.getChip();
		assert(chip == CHIP_W5300);

		// read registers and clear interrupt registers
		uint16_t ir = W5300.readIR();
		// LOG_TRACE("chip=0x", DebugLogBase::HEX, chip, ", ir=(hex)", DebugLogBase::HEX, ir);

		// // clear interrupt registers
		// W5300.writeIR(ir);

		uint16_t ir_eth = ir & ~Sn_INT;
		// LOG_TRACE("ir_eth=(hex)", DebugLogBase::HEX, ir_eth);
		if ((ir_eth) && (ethernetEventCallback != nullptr))
		{
#if defined(TARGET_RASPBERRY_PI_PICO)
			auto ethEvent = queue.event(ethernetEventCallback);
			ethEvent.post(ir);
#elif defined(ESP32)
			(*ethernetEventCallback)(ir);
#endif
		}

		uint16_t ir_sock = ir & Sn_INT;
		// LOG_TRACE("ir_sock=(hex)", DebugLogBase::HEX, ir_sock);
		if (ir_sock && chip)
		{
			// W5100 chip never supports more than 4 sockets
			int maxindex = (chip == 51 || chip == 50) ? 4 : MAX_SOCK_NUM;
			for (int i = 0; i < maxindex; i++)
			{
				if (ir_sock & 0x01)
				{
					uint16_t sn_ir = W5300.getSnIR(i);
					uint8_t sn_sr = W5300.getSnSR(i);
					W5300.setSnIR(i, sn_ir);
					// LOG_TRACE("sn_ir=(hex)", DebugLogBase::HEX, sn_ir, ", sn_sr=(hex)", DebugLogBase::HEX, sn_sr);

					auto eventCallback = socketEventCallback[i];
					if (eventCallback != nullptr)
					{
#if defined(TARGET_RASPBERRY_PI_PICO)
						auto sockEvent = queue.event(eventCallback);
						sockEvent.post(sn_ir);
#elif defined(ESP32)
						(*eventCallback)(sn_ir);
#endif
					}
				}
				ir_sock >>= 1;
			}
		}

		// clear interrupt registers
		W5300.writeIR(ir);

		// delayMilliSecond(1);

		// if (digitalRead(pinIntn) == LOW)
		// {
		// 	// developer has to fix the "INTn LOW even clearing interrupt registers" error
		// 	LOG_ERROR("W5100S INTn pin keeps LOW even clearing interrupt registers!");
		// }
	}
}

EventEthernetClass Ethernet;
