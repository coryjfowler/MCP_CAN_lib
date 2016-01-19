/*
  mcp_can.h
  2012 Copyright (c) Seeed Technology Inc.  All right reserved.

  Author:Loovee
  Contributor: Cory J. Fowler
  2014-1-16
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
#ifndef _MCP2515_H_
#define _MCP2515_H_

#include "mcp_can_dfs.h"    //REGS2515.h does not confilct mcp_can_dfs.h
#include "REGS2515.h"      //Added by Gabriel

class MCP_CAN
{
    private:

    byte CSpin = 10; //defualt val
    
    byte   m_nExtFlg;                                                  /* identifier xxxID             */
    INT32U m_nID;                                                      /* can id                       */
    byte   m_nDlc;                                                     /* data length:                 */
    byte   m_nDta[8];                                                  /* data                         */
    byte   m_nRtr;                                                     /* rtr                          */
    byte   m_nfilhit;

   private:

    void mcp2515_reset(void);                                           /* reset mcp2515                */
    byte mcp2515_readRegister(const byte address);                    /* read mcp2515's register      */
    void mcp2515_readRegisterS(const byte address, byte values[], const byte n);
    void mcp2515_setRegister(const byte address, const byte value);
    void mcp2515_setRegisterS(const byte address, const byte values[], const byte n);
    void mcp2515_initCANBuffers(void);
    void mcp2515_modifyRegister(const byte address, const byte mask, const byte data);

    byte mcp2515_readStatus(void);                                     /* read mcp2515's Status        */
    byte mcp2515_setCANCTRL_Mode(const byte newmode);                 /* set mode                     */
    boolean mcp2515_configRate(const byte canSpeed);                     /* set boadrate                 */
    byte mcp2515_init(const byte canSpeed);                           /* mcp2515init                  */

    void mcp2515_write_id(const byte mcp_addr, const bool ext, const INT32U id );
    void mcp2515_read_id(const byte mcp_addr, byte* ext, INT32U* id );

    void mcp2515_write_canMsg(const byte buffer_sidh_addr );          /* write can msg                */
    void mcp2515_read_canMsg(const byte buffer_sidh_addr);            /* read can msg                 */
    void mcp2515_start_transmit(const byte mcp_addr);                  /* start transmit               */
    byte mcp2515_getNextFreeTXBuf(byte *txbuf_n);                     /* get Next free txbuf          */

/*
*  can operator function
*/    

    byte setMsg(INT32U id, byte ext, byte rtr, byte len, byte *pData);    /* set message                  */  
    byte clearMsg();                                               /* clear all message to zero    */
    byte readMsg();                                                /* read message                 */
    byte sendMsg();                                                /* send message                 */

public:
    MCP_CAN(byte _CS);
    byte begin(byte speedset);                                       /* Constructor                    */
    byte sendMsgBuf(INT32U id, byte ext, byte rtr, byte len, byte *buf);      /* send buf                     */
    byte readMsgBuf(byte *len, byte *buf);                          /* read buf                     */
    byte checkReceive(void);                                        /* if something received        */
    INT32U getCanId(void);                                          /* get can id when receive      */
    bool getCanRTR(void);

    //------Reception Configuration-----------SEG
    void enableRXBuf0Filters(byte mode);            // Set the filtering mode for the recieve buffer 0
    byte getRXBuf0FilterHit();                      // Returns a value for which filter allowed the message in buffer 0
    void enableRXBuf1Filters(byte mode);            // Set the filtering mode for the recieve buffer 1
    byte getRXBuf1FilterHit();                      // Returns a value for which filter allowed the message in buffer 1
    void setRollover(bool mode);                    // Enables or disables rollover in the recieve buffers
    byte getBuf0RTR();                              // Returns a value representing if the RTR bits are set in recieve buffer 0
    byte getBuf1RTR();                              // Returns a value representing if the RTR bits are set in recieve buffer 1

    //------Masks & Filters Configuration-----SEG
    void setRXFilt(int num, bool extended, INT32U filtBits);
    void setRXMask(int num, bool extended, INT32U maskBits);

    //------Interrupt Configuration-----------SEG
    void enableInt(byte intBits);
    byte getInt();
    void clearInt(int intBit);
    
    //------Error Handaling-------------------GRM
    byte getTxErrors(void);  //
    byte getRxErrors(void);
    bool getErrorFlag(byte bitName);
    void setErrorFlag(byte bitName, boolean val);

};

extern MCP_CAN CAN;
#endif
/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
