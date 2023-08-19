/*
 *  Udp.cpp: Library to send/receive UDP packets with the Arduino ethernet shield.
 *  This version only offers minimal wrapping of socket.cpp
 *  Drop Udp.h/.cpp into the Ethernet library directory at hardware/libraries/Ethernet/
 *
 * MIT License:
 * Copyright (c) 2008 Bjoern Hartmann
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * bjoern@cs.stanford.edu 12/30/2008
 */
/*
 * modified by teamprof.net@gmail.com
 */

#include <Arduino.h>
#include "./EventEthernet.h"
#include "./Dns.h"
#include "./utility/w5300.h"

/* Start EthernetUDP socket, listening at local port PORT */
uint8_t EthernetUDP::begin(uint16_t port)
{
	// LOG_TRACE("sockindex=", sockindex, ", port=", port);
	if (sockindex < MAX_SOCK_NUM)
		Ethernet.socketClose(sockindex);
	sockindex = Ethernet.socketBegin(SnMR::UDP, port);
	if (sockindex >= MAX_SOCK_NUM)
		return 0;
	_port = port;
	_remaining = 0;
	return 1;
}

/* return number of bytes available in the current packet,
   will return zero if parsePacket hasn't been called yet */
int EthernetUDP::available()
{
	return _remaining;
}

/* Release any resources being used by this EthernetUDP instance */
void EthernetUDP::stop()
{
	if (sockindex < MAX_SOCK_NUM)
	{
		Ethernet.socketClose(sockindex);
		sockindex = MAX_SOCK_NUM;
	}
}

int EthernetUDP::beginPacket(const char *host, uint16_t port)
{
	// Look up the host first
	int ret = 0;
	DNSClient dns;
	IPAddress remote_addr;

	dns.begin(Ethernet.dnsServerIP());
	ret = dns.getHostByName(host, remote_addr);
	if (ret != 1)
		return ret;
	return beginPacket(remote_addr, port);
}

int EthernetUDP::beginPacket(IPAddress ip, uint16_t port)
{
	_offset = 0;
	return Ethernet.socketStartUDP(sockindex, rawIPAddress(ip), port);
}

int EthernetUDP::endPacket()
{
	// LOG_TRACE("sockindex=", sockindex);
	return Ethernet.socketSendUDP(sockindex);
}

size_t EthernetUDP::write(uint8_t byte)
{
	return write(&byte, 1);
}

size_t EthernetUDP::write(const uint8_t *buffer, size_t size)
{
	// LOG_TRACE("sockindex=", sockindex, ", _offset=", _offset,
	// 		  ", W5300.udp_send_packet_len=", W5300.udp_send_packet_len,
	// 		  ", size=", size, ", buffer=(hex)", DebugLogBase::HEX, LOG_AS_ARR(buffer, size));

	W5300.udp_send_packet_len += size; // Austin :)
	uint16_t bytes_written = Ethernet.socketBufferData(sockindex, _offset, buffer, size);
	_offset += bytes_written;
	return bytes_written;
}

int EthernetUDP::parsePacket()
{
	uint16_t rsr = 0;
	// discard any remaining bytes in the last packet
	// LOG_TRACE("_remaining=", _remaining);
	while (_remaining)
	{
		// LOG_TRACE("discard remaining bytes of the last packet: _remaining=", _remaining);

		// could this fail (loop endlessly) if _remaining > 0 and recv in read fails?
		// should only occur if recv fails after telling us the data is there, lets
		// hope the w5100 always behaves :)
		read((uint8_t *)NULL, _remaining);
	}
	rsr = Ethernet.socketRecvAvailable(sockindex);
	if (rsr > 0)
	{
		// LOG_TRACE("Ethernet.socketRecvAvailable(", sockindex, ") returns ", rsr);
		// ASSERT(rsr >= UDP_PACKET_INFO_SIZE);

		// HACK - hand-parse the UDP packet using TCP recv method
		uint8_t tmpBuf[UDP_PACKET_INFO_SIZE];
		int ret = 0;

		// read 8 header bytes and get IP and port from it
		ret = Ethernet.socketRecvUDP(sockindex, tmpBuf, UDP_PACKET_INFO_SIZE); // Get IP, Port, and Remaining size
		if (ret > 0)
		{
			_remoteIP = tmpBuf;
			_remotePort = tmpBuf[4];
			_remotePort = (_remotePort << 8) + tmpBuf[5];
			_remaining = tmpBuf[6];
			_remaining = (_remaining << 8) + tmpBuf[7];

			// When we get here, any remaining bytes are the data
			ret = _remaining;

			// LOG_TRACE("tmpBuf=", DebugLogBase::HEX, LOG_AS_ARR((uint8_t *)tmpBuf, UDP_PACKET_INFO_SIZE));
			// LOG_TRACE("_remoteIP=", LOG_AS_ARR((uint8_t *)&_remoteIP[0], 4), ", _remotePort=", _remotePort, ", _remaining=", _remaining);
		}

		return ret;
	}
	// There aren't any packets available
	return 0;
}

int EthernetUDP::read()
{
	uint8_t byte;
	// LOG_TRACE("_remaining=", _remaining);
	if ((_remaining > 0) && (Ethernet.socketRecv(sockindex, &byte, 1) > 0))
	{
		// We read things without any problems
		_remaining--;
		return byte;
	}

	// If we get here, there's no data available
	return -1;
}

int EthernetUDP::read(unsigned char *buffer, size_t len)
{
	// LOG_TRACE("_remaining=", _remaining, ", len=", len);
	if (_remaining <= 0)
	{
		// there's no data available or recv failed
		return -1;
	}
	if (len == 0)
	{
		return 0;
	}

	int got = Ethernet.socketRecv(sockindex, buffer, _remaining <= len ? _remaining : len);
	// LOG_TRACE("_remaining=", _remaining, ", len=", len, ", sockindex=", sockindex, ", got=", got,
	// 		  ", buffer=(hex)=", DebugLogBase::HEX, LOG_AS_ARR(buffer, got));
	if (got > 0)
	{
		ASSERT(got <= _remaining);
		_remaining -= got;
		return got;
	}
	else
	{
		return -1;
	}
}

int EthernetUDP::peek()
{
	// Unlike recv, peek doesn't check to see if there's any data available, so we must.
	// If the user hasn't called parsePacket yet then return nothing otherwise they
	// may get the UDP header
	if (sockindex >= MAX_SOCK_NUM || _remaining == 0)
		return -1;
	return Ethernet.socketPeek(sockindex);
}

void EthernetUDP::flush()
{
	// TODO: we should wait for TX buffer to be emptied
}

/* Start EthernetUDP socket, listening at local port PORT */
uint8_t EthernetUDP::beginMulticast(IPAddress ip, uint16_t port)
{
	// LOG_TRACE("sockindex=", sockindex, ", ip=", LOG_AS_ARR((const uint8_t *)(void *)&ip, sizeof(ip)), ", port=", port);
	if (sockindex < MAX_SOCK_NUM)
		Ethernet.socketClose(sockindex);
	sockindex = Ethernet.socketBeginMulticast(SnMR::UDP | SnMR::MULTI, ip, port);
	if (sockindex >= MAX_SOCK_NUM)
		return 0;
	_port = port;
	_remaining = 0;
	return 1;
}
