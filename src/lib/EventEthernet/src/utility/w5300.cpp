/*
 * Copyright 2018 Paul Stoffregen
 * Copyright (c) 2010 by Cristian Maglie <c.maglie@bug.st>
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 */
/*
 * modified by teamprof.net@gmail.com
 */

#include <Arduino.h>
#include "../EventEthernet.h"
#include "./w5300.h"

/***************************************************/
/**            Default SS pin setting             **/
/***************************************************/

// W5300 controller instance
uint32_t W5300Class::pinAddrMask;
uint32_t W5300Class::pinDataMask;

// If variant.h or other headers specifically define the
// default SS pin for Ethernet, use it.

#define SS_PIN_DEFAULT PD7

#define WIZCHIP_RST PC8
#define WIZCHIP_INT PC9
#define WIZCHIP_CS1 PD7

// W5300 controller instance
uint8_t W5300Class::ss_pin = PIN_CSn;
// uint16_t W5300Class::SSIZE = 2048;
// uint16_t W5100Class::SMASK = 0x07FF;

W5300Class W5300;

uint8_t W5300Class::chip = 53;
uint16_t W5300Class::sock_io_mode = 0;
uint16_t W5300Class::sock_any_port = SOCK_ANY_PORT_NUM;
uint16_t W5300Class::sock_is_sending = 0;
// uint16_t W5300Class::SSIZE = 2048;
// uint16_t W5300Class::SMASK = 0x07FF;

uint8_t W5300Class::init(void)
{
  static bool initialized = false;
  uint8_t i;

  if (initialized)
    return 1;

  // Many Ethernet shields have a CAT811 or similar reset chip
  // connected to W5100 or W5200 chips.  The W5200 will not work at
  // all, and may even drive its MISO pin, until given an active low
  // reset pulse!  The CAT811 has a 240 ms typical pulse length, and
  // a 400 ms worst case maximum pulse length.  MAX811 has a worst
  // case maximum 560 ms pulse length.  This delay is meant to wait
  // until the reset pulse is ended.  If your hardware has a shorter
  // reset time, this can be edited or removed.

  // Serial.println("w5100 init");

  pinAddrMask = (0x01 << Pin_A0) | (0x01 << Pin_A1) |
                (0x01 << Pin_A2) | (0x01 << Pin_A3) |
                (0x01 << Pin_A4) | (0x01 << Pin_A5) |
                (0x01 << Pin_A6) | (0x01 << Pin_A7) |
                (0x01 << Pin_A8) | (0x01 << Pin_A9);

  pinDataMask = (0x01 << Pin_D0) | (0x01 << Pin_D1) |
                (0x01 << Pin_D2) | (0x01 << Pin_D3) |
                (0x01 << Pin_D4) | (0x01 << Pin_D5) |
                (0x01 << Pin_D6) | (0x01 << Pin_D7);

  // LOG_TRACE("pinAddrMask=(hex)", DebugLogBase::HEX, pinAddrMask);
  // LOG_TRACE("pinDataMask=(hex)", DebugLogBase::HEX, pinDataMask);

  // config A0-A9, /Rd, /Wr, /CS as output
  // set A0-A9=0, /Rd=1, /Wr=1
  uint32_t outMask = pinAddrMask | (0x01 << Pin_Wr) | (0x01 << Pin_Rd) | (0x01 << ss_pin);
  gpio_init_mask(outMask);
  gpio_set_dir_out_masked(outMask);
  uint32_t value = (0x01 << Pin_Wr) | (0x01 << Pin_Rd);
  gpio_put_masked(outMask, value);

  // config D0-D7 as input
  gpio_init_mask(pinDataMask);
  gpio_set_dir_in_masked(pinDataMask);

  // initSS();
  resetSS();

  delayMilliSecond(560);

  // SSIZE = 2048;
  uint8_t memsize[2][MAX_SOCK_NUM] = {{8, 8, 8, 8, 8, 8, 8, 8}, {8, 8, 8, 8, 8, 8, 8, 8}};
  // uint8_t memsize[2][MAX_SOCK_NUM] = {{8, 8, 8, 8, 0, 0, 0, 0}, {8, 8, 8, 8, 0, 0, 0, 0}};
  for (int8_t i = 0; i < MAX_SOCK_NUM; i++)
  {
    W5300.write_SnTX_SIZE(i, memsize[0][i]);
    W5300.write_SnRX_SIZE(i, memsize[1][i]);
  }

  W5300.reset_soft();

  initialized = true;
  return 1; // successful init
}

W5300Linkstatus W5300Class::getLinkStatus()
{
  if (init())
    return LINK_ON;
  else
    return LINK_OFF;
#if 0
	if (!init()) return UNKNOWN;
	switch (chip) {
		case 53:
		//phystatus = readPSTATUS_W5300();              //TODO: W5300, Check Register Addr of PHYCFGR_W5300
		//phystatus = readPHYCFGR_W5300();
		Serial.println("W5300Class::getLinkStatus() \n");
		phystatus = 1;                                  //TODO: W5300, Temp Code:  Not implemented PHYCFGR
		if (phystatus & 0x01) return LINK_ON;
		return LINK_OFF;
	  default:
		return UNKNOWN;
	}
#endif
}

void W5300Class::write_SnRX_SIZE(uint8_t sn, uint8_t size)
{
  // W5300.setRMSR(sn, size);
  setRMSR(sn, size);
}
void W5300Class::write_SnTX_SIZE(uint8_t sn, uint8_t size)
{
  // W5300.setTMSR(sn, size);
  setTMSR(sn, size);
}

void W5300Class::reset_soft(void)
{
  // Serial.println("W5300Class::reset_soft() \n");
  uint8_t gw[4], sn[4], sip[4];
  uint8_t mac[6];

  getSHAR(mac);
  getGAR(gw);
  getSUBR(sn);
  getSIPR(sip);
  setMR(MR_RST);
  getMR(); // for delay

  setSHAR(mac);
  setGAR(gw);
  setSUBR(sn);
  setSIPR(sip);

  chip = getIDR();
}

void W5300Class::setup_Ethernet(void)
{

  uint8_t mac[6] = {0x00, 0x08, 0xDC, 0x12, 0x34, 0x56};
  uint8_t ip[4] = {192, 168, 11, 111};
  uint8_t sn[4] = {255, 255, 255, 0};
  uint8_t gw[4] = {192, 168, 11, 1};
  W5300.setMACAddress(mac);
  W5300.setIPAddress(ip);
  W5300.setGatewayIp(gw);
  W5300.setSubnetMask(sn);
}

uint32_t W5300Class::getSnRX_RSR(uint8_t sn)
{
  uint32_t received_rx_size = (((uint32_t)WIZCHIP_READ(Sn_RX_RSR(sn))) << 16) |
                              (((uint32_t)WIZCHIP_READ(WIZCHIP_OFFSET_INC(Sn_RX_RSR(sn), 2))));
  return received_rx_size + (uint32_t)((sock_pack_info[sn] & 0x02) ? 1 : 0);

  // uint32_t received_rx_size = 0;
  // uint32_t received_rx_size1 = 1;
  // while (1)
  // {
  //   received_rx_size = (((uint32_t)WIZCHIP_READ(Sn_RX_RSR(sn))) << 16) |
  //                      (((uint32_t)WIZCHIP_READ(WIZCHIP_OFFSET_INC(Sn_RX_RSR(sn), 2))) & 0x0000FFFF);
  //   LOG_TRACE("received_rx_size=", received_rx_size, ", received_rx_size1=", received_rx_size1);
  //   if (received_rx_size == received_rx_size1)
  //     break;
  //   received_rx_size1 = received_rx_size; // if first == sencond, Sn_RX_RSR value is valid.
  // }                                       // save second value into first
  // return received_rx_size + (uint32_t)((sock_pack_info[sn] & 0x02) ? 1 : 0);
}

uint32_t W5300Class::getSnTX_FSR(uint8_t sn)
{
  uint32_t free_tx_size = (((uint32_t)WIZCHIP_READ(Sn_TX_FSR(sn))) << 16) |
                          (((uint32_t)WIZCHIP_READ(WIZCHIP_OFFSET_INC(Sn_TX_FSR(sn), 2))));
  return free_tx_size;

  // uint32_t free_tx_size = 0;
  // uint32_t free_tx_size1 = 1;
  // while (1)
  // {
  //   free_tx_size = (((uint32_t)WIZCHIP_READ(Sn_TX_FSR(sn))) << 16) |
  //                  (((uint32_t)WIZCHIP_READ(WIZCHIP_OFFSET_INC(Sn_TX_FSR(sn), 2))) & 0x0000FFFF); // read
  //   if (free_tx_size == free_tx_size1)
  //     break;                      // if first == sencond, Sn_TX_FSR value is valid.
  //   free_tx_size1 = free_tx_size; // save second value into first
  // }
  // return free_tx_size;
}

void W5300Class::execCmdSn(uint8_t s, uint8_t cmd)
{
  // Send command to socket
  W5300.setSnCR(s, cmd);

  // Wait for command to complete, if not in event driver mode
  if (Ethernet.socketEventCallback[s] == nullptr)
  {
    while (W5300.getSnCR(s))
    {
      delayMilliSecond(10);
    }
  }
}

uint16_t W5300Class::write(uint16_t addr, const uint16_t *buf, uint16_t len)
{
  for (uint16_t i = 0; i < len; i++)
  {
    write(addr, (uint16_t)buf[i]);
    // _W5300_DATA(addr) = (uint16_t)buf[i];
  }
  return len;
}

uint16_t W5300Class::read(uint16_t addr, uint16_t *buf, uint16_t len)
{
  for (uint16_t i = 0; i < len; i++)
  {
    buf[i] = read(addr);
    // buf[i] = _W5300_DATA(addr);
  }
  return len;
}

void W5300Class::setRMSR(uint8_t sn, uint8_t rmsr)
{
  uint16_t rmem;
  // rmem = read(WIZCHIP_OFFSET_INC(RMS01R, (sn & 0xFE)));
  rmem = WIZCHIP_READ(WIZCHIP_OFFSET_INC(RMS01R, (sn & 0xFE)));
  if (sn & 0x01)
    rmem = (rmem & 0xFF00) | (((uint16_t)rmsr) & 0x00FF);
  else
    rmem = (rmem & 0x00FF) | (((uint16_t)rmsr) << 8);
  // write(WIZCHIP_OFFSET_INC(RMS01R, (sn & 0xFE)),rmem);
  WIZCHIP_WRITE(WIZCHIP_OFFSET_INC(RMS01R, (sn & 0xFE)), rmem);
  // Serial3.printf("setRMSR: %x, mem: %x \r\n", rmem, rmem);
}

uint8_t W5300Class::getRMSR(uint8_t sn)
{
  if (sn & 0x01)
    return (uint8_t)(WIZCHIP_READ(WIZCHIP_OFFSET_INC(RMS01R, (sn & 0xFE))) & 0x00FF);
  return (uint8_t)(WIZCHIP_READ(WIZCHIP_OFFSET_INC(RMS01R, (sn & 0xFE))) >> 8);
}

void W5300Class::setTMSR(uint8_t sn, uint8_t tmsr)
{
  uint16_t tmem;
  tmem = WIZCHIP_READ(WIZCHIP_OFFSET_INC(TMS01R, (sn & 0xFE)));
  if (sn & 0x01)
    tmem = (tmem & 0xFF00) | (((uint16_t)tmsr) & 0x00FF);
  else
    tmem = (tmem & 0x00FF) | (((uint16_t)tmsr) << 8);
  WIZCHIP_WRITE(WIZCHIP_OFFSET_INC(TMS01R, (sn & 0xFE)), tmem);
}

uint8_t W5300Class::getTMSR(uint8_t sn)
{
  if (sn & 0x01)
    return (uint8_t)(WIZCHIP_READ(WIZCHIP_OFFSET_INC(TMS01R, (sn & 0xFE))) & 0x00FF);
  return (uint8_t)(WIZCHIP_READ(WIZCHIP_OFFSET_INC(TMS01R, (sn & 0xFE))) >> 8);
}