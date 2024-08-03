/*
  mcp_can.h
  2012 Copyright (c) Seeed Technology Inc.  All right reserved.
  2017 Copyright (c) Cory J. Fowler  All Rights Reserved.

  Author: Loovee
  Contributor: Cory J. Fowler
  2017-09-25
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-
  1301  USA
*/

#ifndef _mcp_can_25625T_H_
#define _mcp_can_25625T_H_

#include "mcp_can_dfs_25625T.h"

class MCP_CAN
{
private:
    INT8U m_nExtFlg;      // Identifier Type
                          // Extended (29 bit) or Standard (11 bit)
    INT32U m_nID;         // CAN ID
    INT8U m_nDlc;         // Data Length Code
    INT8U m_nDta[MAX_CHAR_IN_MESSAGE]; // Data array
    INT8U m_nRtr;         // Remote request flag
    INT8U m_nfilhit;      // The number of the filter that matched the message
    SPIClass *mcpSPI;     // The SPI-Device used
    INT8U MCPCS;          // Chip Select pin number
    INT8U mcpMode;        // Mode to return to after configurations are performed.

    void mcp25625_reset(void);       // Soft Reset MCP25625T
    INT8U mcp25625_readRegister(const INT8U address); // Read MCP25625T register
    void mcp25625_readRegisterS(const INT8U address, INT8U values[], const INT8U n); // Read MCP25625T successive registers
    void mcp25625_setRegister(const INT8U address, const INT8U value); // Set MCP25625T register
    void mcp25625_setRegisterS(const INT8U address, const INT8U values[], const INT8U n); // Set MCP25625T successive registers
    void mcp25625_initCANBuffers(void);
    void mcp25625_modifyRegister(const INT8U address, const INT8U mask, const INT8U data); // Set specific bit(s) of a register
    INT8U mcp25625_readStatus(void); // Read MCP25625T Status
    INT8U mcp25625_setCANCTRL_Mode(const INT8U newmode); // Set mode
    INT8U mcp25625_requestNewMode(const INT8U newmode); // Set mode
    INT8U mcp25625_configRate(const INT8U canSpeed, const INT8U canClock); // Set baudrate
    INT8U mcp25625_init(const INT8U canIDMode, const INT8U canSpeed, const INT8U canClock); // Initialize Controller
    void mcp25625_write_mf(const INT8U mcp_addr, const INT8U ext, const INT32U id); // Write CAN Mask or Filter
    void mcp25625_write_id(const INT8U mcp_addr, const INT8U ext, const INT32U id); // Write CAN ID
    void mcp25625_read_id(const INT8U mcp_addr, INT8U* ext, INT32U* id); // Read CAN ID
    void mcp25625_write_canMsg(const INT8U buffer_sidh_addr); // Write CAN message
    void mcp25625_read_canMsg(const INT8U buffer_sidh_addr); // Read CAN message
    INT8U mcp25625_getNextFreeTXBuf(INT8U *txbuf_n); // Find empty transmit buffer

    INT8U setMsg(INT32U id, INT8U rtr, INT8U ext, INT8U len, INT8U *pData); // Set message
    INT8U clearMsg(); // Clear all message to zero
    INT8U readMsg(); // Read message
    INT8U sendMsg(); // Send message

public:
    MCP_CAN(INT8U _CS);
    MCP_CAN(SPIClass *_SPI, INT8U _CS);
    INT8U begin(INT8U idmodeset, INT8U speedset, INT8U clockset); // Initialize controller parameters
    INT8U init_Mask(INT8U num, INT8U ext, INT32U ulData); // Initialize Mask(s)
    INT8U init_Mask(INT8U num, INT32U ulData); // Initialize Mask(s)
    INT8U init_Filt(INT8U num, INT8U ext, INT32U ulData); // Initialize Filter(s)
    INT8U init_Filt(INT8U num, INT32U ulData); // Initialize Filter(s)
    void setSleepWakeup(INT8U enable); // Enable or disable the wake up interrupt (If disabled the MCP25625T will not be woken up by CAN bus activity)
    INT8U setMode(INT8U opMode); // Set operational mode
    INT8U sendMsgBuf(INT32U id, INT8U ext, INT8U len, INT8U *buf); // Send message to transmit buffer
    INT8U sendMsgBuf(INT32U id, INT8U len, INT8U *buf); // Send message to transmit buffer
    INT8U readMsgBuf(INT32U *id, INT8U *ext, INT8U *len, INT8U buf[]); // Read message from receive buffer
    INT8U readMsgBuf(INT32U *id, INT8U *len, INT8U buf[]); // Read message from receive buffer
    INT8U checkReceive(void); // Check for received data
    INT8U checkError(void); // Check for errors
    INT8U getError(void); // Check for errors
    INT8U errorCountRX(void); // Get error count
    INT8U errorCountTX(void); // Get error count
    INT8U enOneShotTX(void); // Enable one-shot transmission
    INT8U disOneShotTX(void); // Disable one-shot transmission
    INT8U abortTX(void); // Abort queued transmission(s)
    INT8U setGPO(INT8U data); // Sets GPO
    INT8U getGPI(void); // Reads GPI
};

#endif
