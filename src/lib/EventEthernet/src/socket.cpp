/* Copyright 2018 Paul Stoffregen
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this
 * software and associated documentation files (the "Software"), to deal in the Software
 * without restriction, including without limitation the rights to use, copy, modify,
 * merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 * OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
/*
 * modified by teamprof.net@gmail.com
 */

#include <Arduino.h>
#include "./EventEthernet.h"
#include "./utility/w5300.h"

#if ARDUINO >= 156 && !defined(ARDUINO_ARCH_PIC32)
extern void yield(void);
#else
#define yield()
#endif

// socket.h START
#define SOCKET uint8_t ///< SOCKET type define for legacy driver

#define SOCK_OK 1		 ///< Result is OK about socket process.
#define SOCK_BUSY 0		 ///< Socket is busy on processing the operation. Valid only Non-block IO Mode.
#define SOCK_FATAL -1000 ///< Result is fatal error about socket process.

#define SOCK_ERROR 0
#define SOCKERR_SOCKNUM (SOCK_ERROR - 1)	///< Invalid socket number
#define SOCKERR_SOCKOPT (SOCK_ERROR - 2)	///< Invalid socket option
#define SOCKERR_SOCKINIT (SOCK_ERROR - 3)	///< Socket is not initialized or SIPR is Zero IP address when Sn_MR_TCP
#define SOCKERR_SOCKCLOSED (SOCK_ERROR - 4) ///< Socket unexpectedly closed.
#define SOCKERR_SOCKMODE (SOCK_ERROR - 5)	///< Invalid socket mode for socket operation.
#define SOCKERR_SOCKFLAG (SOCK_ERROR - 6)	///< Invalid socket flag
#define SOCKERR_SOCKSTATUS (SOCK_ERROR - 7) ///< Invalid socket status for socket operation.
#define SOCKERR_ARG (SOCK_ERROR - 10)		///< Invalid argument.
#define SOCKERR_PORTZERO (SOCK_ERROR - 11)	///< Port number is zero
#define SOCKERR_IPINVALID (SOCK_ERROR - 12) ///< Invalid IP address
#define SOCKERR_TIMEOUT (SOCK_ERROR - 13)	///< Timeout occurred
#define SOCKERR_DATALEN (SOCK_ERROR - 14)	///< Data length is zero or greater than buffer max size.
#define SOCKERR_BUFFER (SOCK_ERROR - 15)	///< Socket buffer is not enough for data communication.

#define SOCKFATAL_PACKLEN (SOCK_FATAL - 1) ///< Invalid packet length. Fatal Error.

#define SF_TCP_ALIGN 0x02	///< Valid only \ref Sn_MR_TCP and W5300, refer to \ref Sn_MR_ALIGN
#define SF_IO_NONBLOCK 0x01 ///< Socket nonblock io mode. It used parameter in \ref socket().

/*
 * UDP & MACRAW Packet Infomation
 */
#define PACK_FIRST 0x80		///< In Non-TCP packet, It indicates to start receiving a packet. (When W5300, This flag can be applied)
#define PACK_REMAINED 0x01	///< In Non-TCP packet, It indicates to remain a packet to be received. (When W5300, This flag can be applied)
#define PACK_COMPLETED 0x00 ///< In Non-TCP packet, It indicates to complete to receive a packet. (When W5300, This flag can be applied)
// A20150601 : For Integrating with W5300
#define PACK_FIFOBYTE 0x02 ///< Valid only W5300, It indicate to have read already the Sn_RX_FIFOR.

// socket.h  END

// TODO: randomize this when not using DHCP, but how?
static uint16_t local_port = 49152; // 49152 to 65535

// W5300 TODO: Erase
// USed SocketPeek() --> UDP
typedef struct
{
	uint16_t RX_RSR; // Number of bytes received
	uint16_t RX_RD;	 // Address to read
	uint16_t TX_FSR; // Free space ready for transmit
	uint8_t RX_inc;	 // how much have we advanced RX_RD
} socketstate_t;
// W5300 TODO: add sock_remained_size, sock_is_sending, sock_any_port

static socketstate_t state[MAX_SOCK_NUM];

static uint32_t getSockTX_FSR(uint8_t s);
// static uint32_t getSockRX_RSR(uint8_t s);
static void write_data(uint8_t s, uint16_t offset, const uint8_t *data, uint16_t len);
static void read_data(uint8_t s, uint8_t *dst, uint16_t len); // W5300
// static void read_data(uint8_t s, uint16_t src, uint8_t *dst, uint16_t len);

/*****************************************/
/*          Socket management            */
/*****************************************/

void EthernetClass::socketPortRand(uint16_t n)
{
	n &= 0x3FFF;
	local_port ^= n;
}

// W5300, ioLibrary's Socket Func
uint8_t EthernetClass::socketBegin(uint8_t protocol, uint16_t port)
{
	// LOG_TRACE("protocol=", protocol, ", port=", port);
	uint8_t s, stat, status[MAX_SOCK_NUM], chip, maxindex = MAX_SOCK_NUM;
#if 0 // W5300 ioLibrary socket() START
	switch(protocol)
	{
      case Sn_MR_TCP :
         {

            uint32_t taddr;
            W5300.getSIPR((uint8_t*)&taddr);
            if(taddr == 0) return SOCKERR_SOCKINIT;
	    break;
         }
      case Sn_MR_UDP :
      case Sn_MR_MACRAW :
	  case Sn_MR_IPRAW :
         break;
#if (_WIZCHIP_ < 5200)
      case Sn_MR_PPPoE :
         break;
#endif
      default :
         return SOCKERR_SOCKMODE;
	}
#endif // W5300 ioLibrary socket() END
	// ARDUINO codes START to read socket START
	//  first check hardware compatibility
	chip = W5300.getChip();
	// LOG_TRACE("chip=", DebugLogBase::HEX, chip);
	if (!chip)
		return MAX_SOCK_NUM; // immediate error if no hardware detected

	for (s = 0; s < maxindex; s++)
	{
		status[s] = W5300.getSnSR(s);
		// LOG_TRACE("s=", s, ", W5300.getSnSR(s)=status[s]=(hex)", DebugLogBase::HEX, status[s]);
		// W5300, Debug

		if (status[s] == SnSR::CLOSED)
			goto makesocket; // W5300 Debug status[s] is 0x01 Wrong Values
	}
	for (s = 0; s < maxindex; s++)
	{
		stat = status[s];
		if (stat == SnSR::LAST_ACK)
			goto closemakesocket;
		if (stat == SnSR::TIME_WAIT)
			goto closemakesocket;
		if (stat == SnSR::FIN_WAIT)
			goto closemakesocket;
		if (stat == SnSR::CLOSING)
			goto closemakesocket;
	}
// ARDUINO codes START to read socket END
closemakesocket:
	// LOG_TRACE("W5300 socketBegin::socket close: stat=(hex)", DebugLogBase::HEX, stat);
	W5300.execCmdSn(s, Sn_CR_CLOSE); // ARDUINO: Sock_CLOSE

makesocket:
	EthernetServer::server_port[s] = 0;
	uint8_t flag = 0x00; // Temp for ioLibrary,  SF_IO_NONBLOCK or Sn_MR_MULTI
	W5300.setSnMR(s, ((uint16_t)(protocol | (flag & 0xF0))) | (((uint16_t)(flag & 0x02)) << 7));
	W5300.setSnIR(s, 0xFF);

	// W5300 ioLibrary START
	if (!port)
	{
		port = W5300.sock_any_port++;
		if (W5300.sock_any_port == 0xFFF0)
			W5300.sock_any_port = SOCK_ANY_PORT_NUM;
	}
	W5300.setSnPORT(s, port);
	W5300.execCmdSn(s, Sn_CR_OPEN);

	// A20150401 : For release the previous sock_io_mode
	W5300.sock_io_mode &= ~(1 << s);
	W5300.sock_io_mode |= ((flag & SF_IO_NONBLOCK) << s);
	W5300.sock_is_sending &= ~(1 << s);
	W5300.sock_remained_size[s] = 0;
	W5300.sock_pack_info[s] = PACK_COMPLETED;

	while (W5300.getSnSR(s) == SOCK_CLOSED)
	{
		; // TODO: Temp. blocked, check if Blocking code
		delayMilliSecond(10);
	}
	// W5300 ioLibrary END

	return (int8_t)s;
}

// TODO: W5300 Impl
//  multicast version to set fields before open  thd
uint8_t EthernetClass::socketBeginMulticast(uint8_t protocol, IPAddress ip, uint16_t port)
{
	uint8_t s, status[MAX_SOCK_NUM], chip, maxindex = MAX_SOCK_NUM;

	// first check hardware compatibility
	chip = W5300.getChip();
	if (!chip)
		return MAX_SOCK_NUM; // immediate error if no hardware detected
	// Serial.printf("W5000socket begin, protocol=%d, port=%d\n", protocol, port);

	// look at all the hardware sockets, use any that are closed (unused)
	for (s = 0; s < maxindex; s++)
	{
		status[s] = W5300.getSnSR(s);
		if (status[s] == SnSR::CLOSED)
			goto makesocket;
	}
	// Serial.printf("W5000socket step2\n");
	//  as a last resort, forcibly close any already closing
	for (s = 0; s < maxindex; s++)
	{
		uint8_t stat = status[s];
		if (stat == SnSR::LAST_ACK)
			goto closemakesocket;
		if (stat == SnSR::TIME_WAIT)
			goto closemakesocket;
		if (stat == SnSR::FIN_WAIT)
			goto closemakesocket;
		if (stat == SnSR::CLOSING)
			goto closemakesocket;
	}
#if 0 // check CLOSE WAIT to socket close
	Serial.printf("W5000socket step3\n");
	// next, use any that are effectively closed
	for (s=0; s < MAX_SOCK_NUM; s++) {
		uint8_t stat = status[s];
		// TODO: this also needs to check if no more data
		if (stat == SnSR::CLOSE_WAIT) goto closemakesocket;
	}
#endif
	return MAX_SOCK_NUM; // all sockets are in use
closemakesocket:
	// Serial.printf("W5000socket close\n");
	W5300.execCmdSn(s, Sn_CR_CLOSE); // ARDUINO: Sock_CLOSE
makesocket:
	// Serial.printf("W5000socket %d\n", s);
	EthernetServer::server_port[s] = 0;
	delayMicroseconds(250); // TODO: is this needed??
	W5300.setSnMR(s, protocol);
	W5300.setSnIR(s, 0xFF);
	if (port > 0)
	{
		W5300.setSnPORT(s, port);
	}
	else
	{
		// if don't set the source port, set local_port number.
		if (++local_port < 49152)
			local_port = 49152;
		W5300.setSnPORT(s, local_port);
	}
	// Calculate MAC address from Multicast IP Address
	byte mac[] = {0x01, 0x00, 0x5E, 0x00, 0x00, 0x00};
	mac[3] = ip[1] & 0x7F;
	mac[4] = ip[2];
	mac[5] = ip[3];
	W5300.setSnDIPR(s, ip.raw_address()); // 239.255.0.1
	W5300.setSnDPORT(s, port);
	W5300.setSnDHAR(s, mac);
	W5300.execCmdSn(s, Sn_CR_OPEN); // ARDUINO Sock_OPEN
#if 0								// ARDUINO code
	state[s].RX_RSR = 0;
	state[s].RX_RD  = W5300.getSnRX_RD(s); //TODO: W5300 Not Used
	state[s].RX_inc = 0;
	state[s].TX_FSR = 0;
#endif
	// Serial.printf("W5000 socket prot=%d, RX_RD=%d\n", W5100.readSnMR(s), state[s].RX_RD);
	return s;
}

// Return the socket's status
//
uint8_t EthernetClass::socketStatus(uint8_t s)
{
	uint8_t status = W5300.getSnSR(s);
	// LOG_TRACE("W5300.getSnSR(", s, ")=(hex)", DebugLogBase::HEX, status);
	return status;
}

// Immediately close. If a TCP connection is established, the remote host is left unaware we closed.
//
void EthernetClass::socketClose(uint8_t s)
{
	if (((W5300.getSnMR(s) & 0x0F) == Sn_MR_TCP) &&
		(W5300.getSnTX_FSR(s) != W5300.getSnTxMAX(s)))
	{
		uint8_t destip[4] = {0, 0, 0, 1};

		W5300.setSnMR(s, Sn_MR_UDP);
		W5300.setSnPORTR(s, 0x3000);
		W5300.setSnCR(s, Sn_CR_OPEN);
		while (W5300.getSnCR(s) != 0)
		{
			delayMilliSecond(10);
		}
		while (W5300.getSnSR(s) != SOCK_UDP)
		{
			delayMilliSecond(10);
		}

		// TODO: check sendto is needed
		// sendto(s,destip,1,destip,0x3000); // send the dummy data to an unknown destination(0.0.0.1).
	}

	W5300.execCmdSn(s, Sn_CR_CLOSE); // ARDUINO value: Sock_CLOSE

	/* clear all interrupt of the socket. */
	W5300.setSnIR(s, 0xFF);

	// A20150401 : Release the sock_io_mode of socket n.
	W5300.sock_io_mode &= ~(1 << s);

	W5300.sock_is_sending &= ~(1 << s);
	W5300.sock_remained_size[s] = 0;
	W5300.sock_pack_info[s] = 0;
	while (W5300.getSnSR(s) != SOCK_CLOSED)
	{
		// TODO: W5300, check Sn_SR and Sn_CR
		delayMilliSecond(10);
	}
}

// Place the socket in listening (server) mode
//
uint8_t EthernetClass::socketListen(uint8_t s)
{
	if (W5300.getSnSR(s) != SnSR::INIT)
	{
		// Serial.printf("EthernetClass::socketListen(), readSnSR is INIT(13)=0x%02X\r\n", W5300.getSnSR(s));
		return 0;
	}

	W5300.execCmdSn(s, Sock_LISTEN);
	while (W5300.getSnSR(s) != SOCK_LISTEN)
	{
		socketClose(s);
		return SOCKERR_SOCKCLOSED;
	}
	return SOCK_OK;
}

// establish a TCP connection in Active (client) mode.
uint8_t EthernetClass::socketConnect(uint8_t s, uint8_t *addr, uint16_t port)
{
	// LOG_TRACE("s=", s, ", addr=", LOG_AS_ARR(addr, 4), ", port=", port);

	// set destination IP
	W5300.setSnDIPR(s, addr);
	W5300.setSnDPORT(s, port);
	W5300.execCmdSn(s, Sn_CR_CONNECT);

	if (W5300.sock_io_mode & (1 << s))
	{
		LOG_TRACE("return SOCK_BUSY");
		return SOCK_BUSY;
	}

	if (Ethernet.socketEventCallback[s] == nullptr)
	{
		while (W5300.getSnSR(s) != SOCK_ESTABLISHED)
		{
			if (W5300.getSnIR(s) & Sn_IR_TIMEOUT)
			{
				W5300.setSnIR(s, Sn_IR_TIMEOUT);
				return SOCKERR_TIMEOUT;
			}
			if (W5300.getSnSR(s) == SOCK_CLOSED)
			{
				return SOCKERR_SOCKCLOSED;
			}
		}
	}

	return SOCK_OK;
}

// Gracefully disconnect a TCP connection.
//
uint8_t EthernetClass::socketDisconnect(uint8_t s)
{
	W5300.execCmdSn(s, Sock_DISCON); // Sn_CR_DISCON
	W5300.sock_is_sending &= ~(1 << s);
	if (W5300.sock_io_mode & (1 << s))
		return SOCK_BUSY;

	if (Ethernet.socketEventCallback[s] == nullptr)
	{
		while (W5300.getSnSR(s) != SOCK_CLOSED)
		{
			if (W5300.getSnIR(s) & Sn_IR_TIMEOUT)
			{
				socketClose(s);
				return SOCKERR_TIMEOUT;
			}
		}
	}

	return SOCK_OK;
}

/*****************************************/
/*    Socket Data Receive Functions      */
/*****************************************/

// // TODO: Debug, W5300 Modified
// static uint32_t getSockRX_RSR(uint8_t s)
// {
// 	uint32_t rx_size = W5300.getSnRX_RSR(s);
// 	// LOG_TRACE("getSockRX_RSR(", s, ")=(hex)", rx_size);
// 	return rx_size;
// }

// static void read_data(uint8_t s, uint16_t src, uint8_t *dst, uint16_t len)
static void read_data(uint8_t s, uint8_t *buf, uint16_t len)
{
	if (len == 0)
		return;

	uint16_t wordData = 0;
	uint8_t byteData;

	// dummy read Sn_MR: workaround on the limitation of accessing Sn_RX_FIFOR right after accessing Sn_TX_FIFOR
	uint16_t snmr = W5300.getSnMR(s);

	for (uint16_t i = 0; i < len; i++)
	{
		if ((i & 0x01) == 0)
		{
			wordData = W5300.getSnRX_FIFOR(s); // TODO: Check Data recv, Addr: 0x230 + socketOffset(N * 40)
			// LOG_TRACE("getSnRX_FIFOR(", s, ")=(hex)", DebugLogBase::HEX, wordData);
			byteData = (uint8_t)(wordData >> 8);
		}
		else
		{
			byteData = (uint8_t)wordData; // For checking the memory access violation
		}

		if (buf != nullptr)
		{
			buf[i] = byteData;
		}
	}
	W5300.sock_remained_byte[s] = (uint8_t)wordData; // back up the remaind fifo byte.
}

int EthernetClass::socketRecv(uint8_t sn, uint8_t *buf, int16_t len)
{
	ASSERT(len > 0);
	// ASSERT(len >= 0);

	uint16_t snmr = W5300.getSnMR(sn);
	// LOG_TRACE("snmr=(hex)", DebugLogBase::HEX, snmr,
	// 		  ", Sn_MR_TCP=(hex)", Sn_MR_TCP, ", Sn_MR_UDP=(hex)", Sn_MR_UDP, ", Sn_MR_IPRAW=(hex)", Sn_MR_IPRAW);

	uint32_t recvsize = W5300.getSnRxMAX(sn);
	// LOG_TRACE("sn=", sn, ", sock_remained_size[sn]=", W5300.sock_remained_size[sn], ", recvsize=", recvsize, ", len=", len);
	ASSERT(recvsize <= 65536);
	if (len > recvsize)
	{
		len = recvsize;
	}

	if (W5300.sock_remained_size[sn] == 0)
	{
		do
		{
			recvsize = W5300.getSnRX_RSR(sn);
			uint8_t snsr = W5300.getSnSR(sn); // == 0x22 SOCK_UDP
			// LOG_TRACE("getSockRX_RSR(sn)=", recvsize, ", W5300.getSnSR(sn)=(hex)", DebugLogBase::HEX, snsr);

			if (snsr != SOCK_ESTABLISHED)
			{
				if (snsr == SOCK_CLOSE_WAIT)
				{
					// FSR= Free space ready for transmit
					if ((recvsize == 0) && (W5300.getSnTX_FSR(sn) == W5300.getSnTxMAX(sn)))
					{
						socketClose(sn);
						// LOG_WARN("socketRecv(): Free space is not ready for transmit: getSockTX_FSR()=", getSockTX_FSR(sn));
						return SOCKERR_SOCKSTATUS;
					}
				}
				else
				{
					LOG_TRACE("socketRecv() --> invalid status");
					socketClose(sn);
					return SOCKERR_SOCKSTATUS;
				}
			}
			if ((W5300.sock_io_mode & (1 << sn)) && (recvsize == 0))
			{
				LOG_TRACE("socketRecv() --> sock busy");
				return SOCK_BUSY;
			}
		} while (recvsize == 0);
	}

	snmr = W5300.getSnMR(sn);
	// uint16_t snmr = W5300.getSnMR(sn);
	// LOG_TRACE("sock_remained_size[sn]=", W5300.sock_remained_size[sn], ", getSnMR(sn)=(hex)", DebugLogBase::HEX, snmr);
	if ((W5300.sock_remained_size[sn] == 0) || (snmr & Sn_MR_ALIGN))
	{
		uint16_t rxSize = (snmr & Sn_MR_ALIGN) ? (uint16_t)recvsize : readTcpMacRawPacketInfo(sn);
		W5300.sock_remained_size[sn] = rxSize;
	}

	int size = min(W5300.sock_remained_size[sn], len);
	if (size <= 0)
	{
		return 0;
	}
	recvsize = size;

	// // LOG_TRACE("W5300.sock_remained_size[sn]=", W5300.sock_remained_size[sn], ", recvsize=", recvsize, ", len=", len, ", W5300.sock_pack_info[sn]=(hex)", DebugLogBase::HEX, W5300.sock_pack_info[sn]);
	// if (len > W5300.sock_remained_size[sn])
	// 	len = W5300.sock_remained_size[sn];

	// recvsize = len;
	// // LOG_TRACE("(received size)=", len);
	// // Serial.printf("EthernetClass::socketRecv(), (received size) = %d \n", len);

	// LOG_TRACE("sock_remained_size[sn]=", W5300.sock_remained_size[sn], ", size=", size,
	// 		  ", sock_pack_info[sn]=(hex)", DebugLogBase::HEX, W5300.sock_pack_info[sn]);
	if (W5300.sock_pack_info[sn] & PACK_FIFOBYTE) // PACK_FIFOBYTE indicates to have read already the Sn_RX_FOFOR
	{
		if (buf != nullptr)
		{
			*buf++ = W5300.sock_remained_byte[sn];
		}
		W5300.sock_pack_info[sn] &= ~(PACK_FIFOBYTE);
		recvsize--;
		W5300.sock_remained_size[sn]--;
		// LOG_TRACE("PACK_FIFOBYTE(received size)=", recvsize);
	}

	// LOG_TRACE("recvsize=", recvsize);
	if (recvsize > 0)
	// if (recvsize != 0)
	{
		read_data(sn, buf, recvsize);
		W5300.sock_remained_size[sn] -= recvsize;
	}
	if (W5300.sock_remained_size[sn] > 0)
	{
		W5300.sock_pack_info[sn] |= PACK_REMAINED;
		if (recvsize & 0x1)
			W5300.sock_pack_info[sn] |= PACK_FIFOBYTE;
	}
	else
	{
		W5300.sock_pack_info[sn] = PACK_COMPLETED;
		// LOG_TRACE("recvsize=", recvsize, ", W5300.sock_remained_size[sn]=", W5300.sock_remained_size[sn]);
		// LOG_TRACE("recvsize=", recvsize, ", buf=", DebugLogBase::HEX, LOG_AS_ARR(buf, recvsize));

		W5300.execCmdSn(sn, Sn_CR_RECV);
		// W5300.setSnCR(sn, Sn_CR_RECV);
		// if (Ethernet.socketEventCallback[sn] == nullptr)
		// {
		// 	while (W5300.getSnCR(sn))
		// 	{
		// 		delayMilliSecond(10);
		// 	}
		// }
	}

	if (W5300.getSnMR(sn) & Sn_MR_ALIGN)
	{
		LOG_TRACE("Sn_MR_ALIGN");
		W5300.sock_remained_size[sn] = 0;
	}
	// LOG_TRACE("socketRecv, len=", len);
	return size;
}

uint16_t EthernetClass::readTcpMacRawPacketInfo(uint8_t sn)
{
	uint8_t head[TCP_MACRAW_PACKET_INFO_SIZE];
	read_data(sn, head, TCP_MACRAW_PACKET_INFO_SIZE);

	uint16_t mr = W5300.getMR();
	uint16_t dataSize =
		(mr & MR_FS) ? (((uint16_t)head[1]) << 8) | ((uint16_t)head[0])
					 : (((uint16_t)head[0]) << 8) | ((uint16_t)head[1]);
	W5300.sock_pack_info[sn] = PACK_FIRST;
	// LOG_TRACE("mr=(hex)", DebugLogBase::HEX, mr, ", head=(hex)", LOG_AS_ARR(head, TCP_MACRAW_PACKET_INFO_SIZE),
	// 		  ", dataSize=", DebugLogBase::DEC, dataSize);
	return dataSize;
}

#define Sn_MR_P0_P3 0x0F
uint16_t EthernetClass::readUdpPacketInfo(uint8_t sn, uint8_t *buf, uint16_t len)
{
	uint16_t snmr = W5300.getSnMR(sn);
	// LOG_TRACE("snmr=(hex)", DebugLogBase::HEX, snmr);
	ASSERT((snmr & Sn_MR_P0_P3) == (Sn_MR_UDP));
	ASSERT(len == UDP_PACKET_INFO_SIZE);

	read_data(sn, buf, UDP_PACKET_INFO_SIZE);
	/**< FIFO swap bit of \ref MR. Swap MSB & LSB of \ref Sn_TX_FIFOR & Sn_RX_FIFOR (0 : No swap, 1 : Swap) */
	W5300.sock_remained_size[sn] = (W5300.getMR() & MR_FS) ? (((uint16_t)buf[7]) << 8) | (uint16_t)(buf[6])
														   : (((uint16_t)buf[6]) << 8) | (uint16_t)(buf[7]);
	W5300.sock_pack_info[sn] = PACK_FIRST;
	// LOG_TRACE("sn=", sn, ", sock_remained_size[sn]=", W5300.sock_remained_size[sn],
	// 		  ", buf=(hex)", DebugLogBase::HEX, LOG_AS_ARR(buf, UDP_PACKET_INFO_SIZE));
	return W5300.sock_remained_size[sn];
}

int EthernetClass::socketRecvUDP(uint8_t sn, uint8_t *buf, int16_t len)
{
	// LOG_TRACE("sn=", sn, ", len=", len, ", sock_remained_size[sn]=", W5300.sock_remained_size[sn]);
	// ASSERT(buf != nullptr);
	ASSERT(len >= 0);

	if (W5300.sock_remained_size[sn] == 0)
	{
		uint32_t rxRSR = 0;
		do
		{
			rxRSR = W5300.getSnRX_RSR(sn);
			if (W5300.getSnSR(sn) == SOCK_CLOSED)
				return SOCKERR_SOCKCLOSED;
			if ((W5300.sock_io_mode & (1 << sn)) && (rxRSR == 0))
				return SOCK_BUSY;
			delayMilliSecond(100);
		} while (rxRSR < UDP_PACKET_INFO_SIZE);

		// parse ip address  and port number.
		return readUdpPacketInfo(sn, buf, len);
	}

	int pack_len = min(W5300.sock_remained_size[sn], len);

	if (W5300.sock_pack_info[sn] & PACK_FIFOBYTE) // PACK_FIFOBYTE indicates to have read already the Sn_RX_FOFOR
	{
		if (buf != nullptr)
		{
			*buf++ = W5300.sock_remained_byte[sn];
		}
		W5300.sock_pack_info[sn] &= ~(PACK_FIFOBYTE);
		pack_len--;
		W5300.sock_remained_size[sn]--;
		// LOG_TRACE("PACK_FIFOBYTE(received size)=", recvsize);
	}

	// READ
	if (pack_len > 0)
	{
		read_data(sn, buf, pack_len); // data copy.
		W5300.sock_remained_size[sn] -= pack_len;
	}

	if (W5300.sock_remained_size[sn] > 0)
	{
		W5300.sock_pack_info[sn] |= PACK_REMAINED;
		if (pack_len & 0x01)
			W5300.sock_pack_info[sn] |= PACK_FIFOBYTE;
	}
	else
	{
		W5300.sock_pack_info[sn] = PACK_COMPLETED;

		W5300.execCmdSn(sn, Sn_CR_RECV);
		// W5300.setSnCR(sn, Sn_CR_RECV);
		// if (Ethernet.socketEventCallback[sn] == nullptr)
		// {
		// 	/* wait to process the command... */
		// 	while (W5300.getSnCR(sn))
		// 	{
		// 		delayMilliSecond(10);
		// 	}
		// }
	}
	if (W5300.getSnMR(sn) & Sn_MR_ALIGN)
	{
		LOG_TRACE("Sn_MR_ALIGN");
		W5300.sock_remained_size[sn] = 0;
	}
	return pack_len;
}

uint32_t EthernetClass::socketRecvAvailable(uint8_t s)
{
	uint32_t rsr = W5300.getSnRX_RSR(s);
	// LOG_TRACE("W5300.getSnRX_RSR(", s, ")=", rsr);
	return rsr;
}

// get the first byte in the receive queue (no checking)
//
uint8_t EthernetClass::socketPeek(uint8_t s)
{
	ASSERT(false); // NOT implemented

	uint16_t b;
	uint16_t ptr = state[s].RX_RD;
	W5300.read((ptr & W5300.SMASK) + W5300.RBASE(s), &b, 1);
	return b;
}

/*****************************************/
/*    Socket Data Transmit Functions     */
/*****************************************/

static uint32_t getSockTX_FSR(uint8_t s)
{
	return W5300.getSnTX_FSR(s);
}

static void setsockTX_WRSR(uint8_t s, uint16_t txwrs)
{
	W5300.setSnTX_WRSR(s, txwrs);
}

static void write_data(uint8_t s, uint16_t data_offset, const uint8_t *data, uint16_t len)
{
	if (len == 0)
		return;

	// LOG_TRACE("len=", len);
	// ASSERT((len & 0x01) == 0); // allow even length only

	//  no checking of odd length, is it a potential bug?
	for (uint32_t i = 0; i < len; i += 2)
		W5300.setSnTX_FIFOR(s, (((uint16_t)data[i]) << 8) | (((uint16_t)data[i + 1]) & 0x00FF));

	// dummy read Sn_MR: workaround on the limitation of accessing Sn_RX_FIFOR right after accessing Sn_TX_FIFOR
	uint16_t snmr = W5300.getSnMR(s);
}

/**
 * @brief	This function used to send the data in TCP mode
 * @return	1 for success else 0.
 */
int16_t EthernetClass::socketSend(uint8_t sn, const uint8_t *buf, uint16_t len)
{
	// LOG_TRACE("sn=", sn, ", len=", len, ", buf=(hex)", DebugLogBase::HEX, LOG_AS_ARR(buf, len));

	uint8_t snsr = W5300.getSnSR(sn);
	if (snsr != SOCK_ESTABLISHED && snsr != SOCK_CLOSE_WAIT)
	{
		LOG_TRACE("SOCKERR_SOCKSTATUS");
		return SOCKERR_SOCKSTATUS;
	}

	while (W5300.sock_is_sending & (1 << sn))
	{
		uint16_t snir = W5300.getSnIR(sn);
		if (snir & Sn_IR_SENDOK)
		{
			W5300.setSnIR(sn, Sn_IR_SENDOK);
			W5300.sock_is_sending &= ~(1 << sn);
		}
		else if (snir & Sn_IR_TIMEOUT)
		{
			socketClose(sn);
			LOG_TRACE("SOCKERR_TIMEOUT");
			return SOCKERR_TIMEOUT;
		}
		else
		{
			LOG_TRACE("delayMilliSecond()");
			delayMilliSecond(1);
			// LOG_TRACE("SOCK_BUSY");
			// return SOCK_BUSY;
		}
	}

	len = min(W5300.getSnTxMAX(sn), len); // check size not to exceed MAX size.
	uint16_t freesize;
	do
	{
		freesize = W5300.getSnTX_FSR(sn);
		snsr = W5300.getSnSR(sn);
		if ((snsr != SOCK_ESTABLISHED) && (snsr != SOCK_CLOSE_WAIT))
		{
			socketClose(sn);
			LOG_TRACE("SOCKERR_SOCKSTATUS");
			return SOCKERR_SOCKSTATUS;
		}
		if ((W5300.sock_io_mode & (1 << sn)) && (len > freesize))
		{
			LOG_TRACE("delayMilliSecond()");
			delayMilliSecond(10);
			// LOG_TRACE("SOCK_BUSY");
			// return SOCK_BUSY;
		}
	} while (len > freesize);
	write_data(sn, 0, (uint8_t *)buf, len);

	W5300.setSnTX_WRSR(sn, len);

	W5300.execCmdSn(sn, Sn_CR_SEND);
	// setSn_CR(sn,Sn_CR_SEND);
	/* wait to process the command... */
	// while(getSn_CR(sn));

	W5300.sock_is_sending |= (1 << sn);
	// M20150409 : Explicit Type Casting
	// return len;
	return (int16_t)len;
}

uint16_t EthernetClass::socketSendAvailable(uint8_t s)
{
	uint16_t freesize = getSockTX_FSR(s);
	uint8_t status = W5300.getSnSR(s);
	if ((status == SnSR::ESTABLISHED) || (status == SnSR::CLOSE_WAIT))
	{
		return freesize;
	}
	return 0;
}

// W5300, Called by UDP
uint16_t EthernetClass::socketBufferData(uint8_t s, uint16_t offset, const uint8_t *buf, uint16_t len)
{
	// LOG_TRACE("s=", s, ", offset", offset, ", len=", len, ", buf=(hex)", DebugLogBase::HEX, LOG_AS_ARR(buf, len));
	uint16_t ret = 0;
	uint32_t txfree = W5300.getSnTX_FSR(s);
	if (len > txfree)
	{
		ret = txfree; // check size not to exceed MAX size.
	}
	else
	{
		ret = len;
	}
	write_data(s, offset, buf, ret);
	return ret;
}

bool EthernetClass::socketStartUDP(uint8_t s, uint8_t *addr, uint16_t port)
{
	if (((addr[0] == 0x00) && (addr[1] == 0x00) && (addr[2] == 0x00) && (addr[3] == 0x00)) ||
		((port == 0x00)))
	{
		return false;
	}
	W5300.setSnDIPR(s, addr);
	W5300.setSnDPORT(s, port);
	return true;
}

bool EthernetClass::socketSendUDP(uint8_t s)
{
	// LOG_TRACE("s=", s, ", W5300.udp_send_packet_len=", W5300.udp_send_packet_len);

	// W5300 only START
	W5300.setSnTX_WRSR(s, W5300.udp_send_packet_len); // Opt udp_send_packet_len for Class type
	W5300.execCmdSn(s, Sock_SEND);					  // Sn_CR_SEND

	if (Ethernet.socketEventCallback[s])
	{
		// Sn_IR_SENDOK event will be sent once completed, no need to poll
		W5300.udp_send_packet_len = 0;
	}
	else
	{
		/* +2008.01 bj */
		volatile uint16_t snir = W5300.getSnIR(s);
		// LOG_TRACE("snir=(hex)", DebugLogBase::HEX, snir);
		while ((snir & SnIR::SEND_OK) != SnIR::SEND_OK)
		{
			snir = W5300.getSnIR(s);
			// LOG_TRACE("snir=(hex)", DebugLogBase::HEX, snir);
			if (snir & SnIR::TIMEOUT)
			{
				// LOG_TRACE("SnIR::TIMEOUT");
				/* +2008.01 [bj]: clear interrupt */
				W5300.setSnIR(s, (SnIR::SEND_OK | SnIR::TIMEOUT));
				return false;
			}
			yield();
			delayMilliSecond(100);
		}

		/* +2008.01 bj */
		W5300.setSnIR(s, SnIR::SEND_OK);
		W5300.udp_send_packet_len = 0;
	}

	/* Sent ok */
	return true;
}
