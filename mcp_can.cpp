/*
  mcp_can.cpp
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
#include "mcp_can.h"



#define spi_read() SPI.transfer(0x00)                           //The arduino SPI lib dose not have a read, instead 0's are sent while the data is read in at the same time

/*********************************************************************************************************
** Function name:           mcp2515_reset
** Description:             Resets the MCP2515, a common command used during "init" and other times
*********************************************************************************************************/
void MCP_CAN::mcp2515_reset(void)                                      
{
    digitalWrite(CSpin, LOW);
    SPI.transfer(MCP_RESET);
    digitalWrite(CSpin, HIGH);
    delay(10);
}

/*********************************************************************************************************
** Function name:           mcp2515_readRegister
** Description:             Read Register, See (T11-1) for Register names
*********************************************************************************************************/
byte MCP_CAN::mcp2515_readRegister(const byte address)                                                                     
{
    byte ret;

    digitalWrite(CSpin, LOW);   //Start
    SPI.transfer(MCP_READ);     //Send Read Instruction (12.3)
    SPI.transfer(address);      //Send byte address (T11-1)
    ret = spi_read();           //Send 0's on MOSI while reciving byte on MISO
    digitalWrite(CSpin, HIGH);  //End

    return ret;
}

/*********************************************************************************************************
** Function name:           mcp2515_readRegisterS
** Description:             just like readRegister but if you keep reading, you get the fallowing registers too :)
*********************************************************************************************************/
void MCP_CAN::mcp2515_readRegisterS(const byte address, byte values[], const byte num_of_regs)
{
	byte i;

	digitalWrite(CSpin, LOW);
	SPI.transfer(MCP_READ);
	SPI.transfer(address);
	for (i=0; i<num_of_regs; i++) 
    {
		values[i] = spi_read();
	}
	digitalWrite(CSpin, HIGH);
}

/*********************************************************************************************************
** Function name:           mcp2515_setRegister
** Description:             set register, , See (T11-1) for Register names
*********************************************************************************************************/
void MCP_CAN::mcp2515_setRegister(const byte address, const byte value)
{
    digitalWrite(CSpin, LOW);   //Start
    SPI.transfer(MCP_WRITE);    //Send Write Instruction (12.5)
    SPI.transfer(address);      //Send byte address (T11-1)
    SPI.transfer(value);        //Send data to be written
    digitalWrite(CSpin, HIGH);  //End
}

/*********************************************************************************************************
** Function name:           mcp2515_setRegisterS
** Description:             just like serRegister but if you keep sending in values, you get the fallowing registers writen too :)
*********************************************************************************************************/
void MCP_CAN::mcp2515_setRegisterS(const byte address, const byte values[], const byte n)
{
    byte i;
    digitalWrite(CSpin, LOW);
    SPI.transfer(MCP_WRITE);
    SPI.transfer(address);
       
    for (i=0; i<n; i++) 
    {
        SPI.transfer(values[i]);
    }
    digitalWrite(CSpin, HIGH);
}

/*********************************************************************************************************
** Function name:           mcp2515_modifyRegister
** Description:             set bits of one register, reminder: the mask slects whitch bits out of the byte to change, 
                            the data is the bits you want the changes to be, the mask and data bits line up.
*********************************************************************************************************/
void MCP_CAN::mcp2515_modifyRegister(const byte address, const byte mask, const byte data)
{
    digitalWrite(CSpin, LOW);
    SPI.transfer(MCP_BITMOD);
    SPI.transfer(address);
    SPI.transfer(mask);
    SPI.transfer(data);
    digitalWrite(CSpin, HIGH);
}

/*********************************************************************************************************
** Function name:           mcp2515_readStatus
** Description:             read mcp2515's Status, 8 very important bits are returned in a byte
*********************************************************************************************************/
byte MCP_CAN::mcp2515_readStatus(void)                             
{
	byte i;
	digitalWrite(CSpin, LOW);
	SPI.transfer(MCP_READ_STATUS);  //(12.8)
	i = spi_read();
	digitalWrite(CSpin, HIGH);
	
	return i;
}

/*********************************************************************************************************
** Function name:           mcp2515_setCANCTRL_Mode
** Description:             set control mode
*********************************************************************************************************/
byte MCP_CAN::mcp2515_setCANCTRL_Mode(const byte newmode)
{
    byte i;

    mcp2515_modifyRegister(MCP_CANCTRL, MODE_MASK, newmode);

    i = mcp2515_readRegister(MCP_CANCTRL);
    i &= MODE_MASK;

    if ( i == newmode ) 
    {
        return MCP2515_OK;
    }

    return MCP2515_FAIL;
}

/*********************************************************************************************************
** Function name:           mcp2515_configRate
** Description:             set boadrate
*********************************************************************************************************/
boolean MCP_CAN::mcp2515_configRate(const byte canSpeed)            
{
    byte cfg1, cfg2, cfg3;
    boolean set = 1;

    switch (canSpeed) 
    {
        case (CAN_5KBPS):
        cfg1 = MCP_16MHz_5kBPS_CFG1;
        cfg2 = MCP_16MHz_5kBPS_CFG2;
        cfg3 = MCP_16MHz_5kBPS_CFG3;
        break;

        case (CAN_10KBPS):
        cfg1 = MCP_16MHz_10kBPS_CFG1;
        cfg2 = MCP_16MHz_10kBPS_CFG2;
        cfg3 = MCP_16MHz_10kBPS_CFG3;
        break;

        case (CAN_20KBPS):
        cfg1 = MCP_16MHz_20kBPS_CFG1;
        cfg2 = MCP_16MHz_20kBPS_CFG2;
        cfg3 = MCP_16MHz_20kBPS_CFG3;
        break;
        
        case (CAN_31K25BPS):
        cfg1 = MCP_16MHz_31k25BPS_CFG1;
        cfg2 = MCP_16MHz_31k25BPS_CFG2;
        cfg3 = MCP_16MHz_31k25BPS_CFG3;
        break;

        case (CAN_40KBPS):
        cfg1 = MCP_16MHz_40kBPS_CFG1;
        cfg2 = MCP_16MHz_40kBPS_CFG2;
        cfg3 = MCP_16MHz_40kBPS_CFG3;
        break;

        case (CAN_50KBPS):
        cfg1 = MCP_16MHz_50kBPS_CFG1;
        cfg2 = MCP_16MHz_50kBPS_CFG2;
        cfg3 = MCP_16MHz_50kBPS_CFG3;
        break;

        case (CAN_80KBPS):
        cfg1 = MCP_16MHz_80kBPS_CFG1;
        cfg2 = MCP_16MHz_80kBPS_CFG2;
        cfg3 = MCP_16MHz_80kBPS_CFG3;
        break;

        case (CAN_100KBPS):                                             /* 100KBPS                  */
        cfg1 = MCP_16MHz_100kBPS_CFG1;
        cfg2 = MCP_16MHz_100kBPS_CFG2;
        cfg3 = MCP_16MHz_100kBPS_CFG3;
        break;

        case (CAN_125KBPS):
        cfg1 = MCP_16MHz_125kBPS_CFG1;
        cfg2 = MCP_16MHz_125kBPS_CFG2;
        cfg3 = MCP_16MHz_125kBPS_CFG3;
        break;

        case (CAN_200KBPS):
        cfg1 = MCP_16MHz_200kBPS_CFG1;
        cfg2 = MCP_16MHz_200kBPS_CFG2;
        cfg3 = MCP_16MHz_200kBPS_CFG3;
        break;

        case (CAN_250KBPS):
        cfg1 = MCP_16MHz_250kBPS_CFG1;
        cfg2 = MCP_16MHz_250kBPS_CFG2;
        cfg3 = MCP_16MHz_250kBPS_CFG3;
        break;

        case (CAN_500KBPS):
        cfg1 = MCP_16MHz_500kBPS_CFG1;
        cfg2 = MCP_16MHz_500kBPS_CFG2;
        cfg3 = MCP_16MHz_500kBPS_CFG3;
        break;
        
        case (CAN_1000KBPS):
        cfg1 = MCP_16MHz_1000kBPS_CFG1;
        cfg2 = MCP_16MHz_1000kBPS_CFG2;
        cfg3 = MCP_16MHz_1000kBPS_CFG3;
        break;  

        default:
        set = 0;
        cfg1 = MCP_16MHz_500kBPS_CFG1;          //!!! Default is 500kbs but will also fail the MCP2515 init so it dosent matter
        cfg2 = MCP_16MHz_500kBPS_CFG2;
        cfg3 = MCP_16MHz_500kBPS_CFG3;       
        break;
    }


    if (set) {
        mcp2515_setRegister(MCP_CNF1, cfg1);
        mcp2515_setRegister(MCP_CNF2, cfg2);
        mcp2515_setRegister(MCP_CNF3, cfg3);
        return MCP2515_OK;
    }
    else {
        return MCP2515_FAIL;
    }
}

/*********************************************************************************************************
** Function name:           mcp2515_initCANBuffers
** Description:             init canbuffers
*********************************************************************************************************/
void MCP_CAN::mcp2515_initCANBuffers(void)
{
    byte i, a1, a2, a3;
    
    bool std = 0;               
    bool ext = 1;
    INT32U ulMask = 0x00, ulFilt = 0x00;


    mcp2515_write_id(MCP_RXM0SIDH, ext, ulMask);			/*Set both masks to 0           */
    mcp2515_write_id(MCP_RXM1SIDH, ext, ulMask);			/*Mask register ignores ext bit */
    
                                                            /* Set all filters to 0         */
    mcp2515_write_id(MCP_RXF0SIDH, ext, ulFilt);			/* RXB0: extended               */
    mcp2515_write_id(MCP_RXF1SIDH, std, ulFilt);			/* RXB1: standard               */
    mcp2515_write_id(MCP_RXF2SIDH, ext, ulFilt);			/* RXB2: extended               */
    mcp2515_write_id(MCP_RXF3SIDH, std, ulFilt);			/* RXB3: standard               */
    mcp2515_write_id(MCP_RXF4SIDH, ext, ulFilt);
    mcp2515_write_id(MCP_RXF5SIDH, std, ulFilt);

                                                                        /* Clear, deactivate the three  */
                                                                        /* transmit buffers             */
                                                                        /* TXBnCTRL -> TXBnD7           */
    a1 = MCP_TXB0CTRL;
    a2 = MCP_TXB1CTRL;
    a3 = MCP_TXB2CTRL;
    for (i = 0; i < 14; i++) {                                          /* in-buffer loop               */
        mcp2515_setRegister(a1, 0);
        mcp2515_setRegister(a2, 0);
        mcp2515_setRegister(a3, 0);
        a1++;
        a2++;
        a3++;
    }
    mcp2515_setRegister(RXB0CTRL, 0);
    mcp2515_setRegister(RXB1CTRL, 0);
}

/*********************************************************************************************************
** Function name:           mcp2515_init
** Description:             initalise the device
*********************************************************************************************************/
byte MCP_CAN::mcp2515_init(const byte canSpeed)                      
{
  byte res;

    mcp2515_reset();

    res = mcp2515_setCANCTRL_Mode(MODE_CONFIG);
    if(res > 0)
    {
      return res;
    }

                                                                        /* set boadrate                 */
    if(mcp2515_configRate(canSpeed))
    {
      return res;
    }

    if ( res == MCP2515_OK ) {

                                                                        /* init canbuffers              */
        mcp2515_initCANBuffers();
                                                                        /* interrupt mode               */
        mcp2515_setRegister(MCP_CANINTE, MCP_RX0IF | MCP_RX1IF);
                                                                        /* enable both receive-buffers  */
                                                                        /* to receive messages          */
                                                                        /* with std. and ext. identifie */
                                                                        /* rs                           */
                                                                        /* and enable rollover          */
        mcp2515_modifyRegister(RXB0CTRL,
        MCP_RXB_RX_MASK | MCP_RXB_BUKT_MASK,
        MCP_RXB_RX_STDEXT | MCP_RXB_BUKT_MASK );
        mcp2515_modifyRegister(RXB1CTRL, MCP_RXB_RX_MASK,
        MCP_RXB_RX_STDEXT);
                                                                        /* enter normal mode            */
        res = mcp2515_setCANCTRL_Mode(MODE_NORMAL);     
                                                                   
        if(res)
        {           
          return res;
        }

    }                                                 
}

/*********************************************************************************************************
** Function name:           mcp2515_write_id
** Description:             write can id
*********************************************************************************************************/
void MCP_CAN::mcp2515_write_id(const byte mcp_addr, const bool ext, const INT32U id )
{
    uint16_t canid;
    byte tbufdata[4];

    canid = (uint16_t)(id & 0x0FFFF);

    if ( ext == 1) 
    {
        tbufdata[MCP_EID0] = (byte) (canid & 0xFF);
        tbufdata[MCP_EID8] = (byte) (canid >> 8);
        canid = (uint16_t)(id >> 16);
        tbufdata[MCP_SIDL] = (byte) (canid & 0x03);
        tbufdata[MCP_SIDL] += (byte) ((canid & 0x1C) << 3);
        tbufdata[MCP_SIDL] |= MCP_TXB_EXIDE_M;
        tbufdata[MCP_SIDH] = (byte) (canid >> 5 );
    }
    else 
    {
        tbufdata[MCP_SIDH] = (byte) (canid >> 3 );
        tbufdata[MCP_SIDL] = (byte) ((canid & 0x07 ) << 5);
        tbufdata[MCP_EID0] = 0;
        tbufdata[MCP_EID8] = 0;
    }
    mcp2515_setRegisterS( mcp_addr, tbufdata, 4 );
}

/*********************************************************************************************************
** Function name:           mcp2515_read_id
** Description:             read can id
*********************************************************************************************************/
void MCP_CAN::mcp2515_read_id(const byte mcp_addr, byte* ext, INT32U* id )
{
    byte tbufdata[4];

    *ext = 0;
    *id = 0;

    mcp2515_readRegisterS( mcp_addr, tbufdata, 4 );

    *id = (tbufdata[MCP_SIDH]<<3) + (tbufdata[MCP_SIDL]>>5);

    if ( (tbufdata[MCP_SIDL] & MCP_TXB_EXIDE_M) ==  MCP_TXB_EXIDE_M ) 
    {
                                                                        /* extended id                  */
        *id = (*id<<2) + (tbufdata[MCP_SIDL] & 0x03);
        *id = (*id<<8) + tbufdata[MCP_EID8];
        *id = (*id<<8) + tbufdata[MCP_EID0];
        *ext = 1;
    }
}

/*********************************************************************************************************
** Function name:           mcp2515_write_canMsg
** Description:             write msg
*********************************************************************************************************/
void MCP_CAN::mcp2515_write_canMsg(const byte buffer_sidh_addr)
{
    byte mcp_addr;
    mcp_addr = buffer_sidh_addr;
    mcp2515_setRegisterS(mcp_addr+5, m_nDta, m_nDlc );                  /* write data bytes, m_nDlc is
                                                                         used to tell setRegisterS how many*/

    if ( m_nRtr == 1)                                                   /* if RTR set bit in byte       */
    {
        m_nDlc |= MCP_RTR_MASK;  
    }
    mcp2515_setRegister((mcp_addr+4), m_nDlc );                        /* write the RTR and DLC (mcp_addr+4) = TXBnCTRL +1 +4 = TXBnDLC */
    mcp2515_write_id(mcp_addr, m_nExtFlg, m_nID );                     /* write CAN id                 */

}

/*********************************************************************************************************
** Function name:           mcp2515_read_canMsg
** Description:             read message
*********************************************************************************************************/
void MCP_CAN::mcp2515_read_canMsg(const byte buffer_sidh_addr)        /* read can msg                 */
{
    byte mcp_addr, ctrl;

    mcp_addr = buffer_sidh_addr;

    mcp2515_read_id( mcp_addr, &m_nExtFlg,&m_nID );

    ctrl = mcp2515_readRegister( mcp_addr-1 );
    m_nDlc = mcp2515_readRegister( mcp_addr+4 );

    if ((ctrl & 0x08)) {
        m_nRtr = 1;
    }
    else {
        m_nRtr = 0;
    }

    m_nDlc &= MCP_DLC_MASK;
    mcp2515_readRegisterS( mcp_addr+5, &(m_nDta[0]), m_nDlc );
}

/*********************************************************************************************************
** Function name:           mcp2515_start_transmit
** Description:             Tells the MCP2515 to begin transmission of the message we put in the buffer
*********************************************************************************************************/
void MCP_CAN::mcp2515_start_transmit(const byte mcp_addr)              /* start transmit               */
{
    mcp2515_modifyRegister( mcp_addr-1 , MCP_TXB_TXREQ_M, MCP_TXB_TXREQ_M );
}

/*********************************************************************************************************
** Function name:           mcp2515_getNextFreeTXBuf
** Description:             finds the next TX buffer out of the three to use i guess
*********************************************************************************************************/
byte MCP_CAN::mcp2515_getNextFreeTXBuf(byte *txbuf_n)                 /* get Next free txbuf          */
{
    byte res, i, ctrlval;
    byte ctrlregs[MCP_N_TXBUFFERS] = { MCP_TXB0CTRL, MCP_TXB1CTRL, MCP_TXB2CTRL };

    res = MCP_ALLTXBUSY;
    *txbuf_n = 0x00;

                                                                        /* check all 3 TX-Buffers       */
    for (i=0; i<MCP_N_TXBUFFERS; i++) {
        ctrlval = mcp2515_readRegister( ctrlregs[i] );
        if ( (ctrlval & MCP_TXB_TXREQ_M) == 0 ) {
            *txbuf_n = ctrlregs[i]+1;                                   /* return SIDH-address of Buffe */
                                                                        /* r                            */
            res = MCP2515_OK;
            return res;                                                 /* ! function exit              */
        }
    }
    return res;
}

/*********************************************************************************************************
** Function name:           MCP_CAN
** Description:             init CS pin and set UNSELECTED
*********************************************************************************************************/
MCP_CAN::MCP_CAN(byte _CS)
{
    CSpin = _CS;
    pinMode(CSpin, OUTPUT);
    digitalWrite(CSpin, HIGH);
}

/*********************************************************************************************************
** Function name:           init
** Description:             init can and set speed
*********************************************************************************************************/
byte MCP_CAN::begin(byte speedset)
{
    SPI.begin();
    return mcp2515_init(speedset);
}

/*********************************************************************************************************
** Function name:           send
** Description:             set can message, such as dlc, id, dta[] and so on
*********************************************************************************************************/
byte MCP_CAN::setMsg(INT32U id, byte ext, byte rtr, byte len, byte *pData)
{
    int i = 0;
    m_nExtFlg = ext;
    m_nID     = id;
    m_nRtr    = rtr;
    m_nDlc    = len;
    for(i = 0; i<8; i++)  //8 bytes in message
    m_nDta[i] = *(pData+i);
    return MCP2515_OK;
}

/*********************************************************************************************************
** Function name:           clearMsg
** Description:             set all message to zero
*********************************************************************************************************/
byte MCP_CAN::clearMsg()
{
    m_nID       = 0;
    m_nDlc      = 0;
    m_nExtFlg   = 0;
    m_nRtr      = 0;
    m_nfilhit   = 0;
    for(int i = 0; i<m_nDlc; i++ )
      m_nDta[i] = 0x00;

    return MCP2515_OK;
}

/*********************************************************************************************************
** Function name:           sendMsg
** Description:             send message
*********************************************************************************************************/
byte MCP_CAN::sendMsg()
{
    byte res, res1, txbuf_n;
    uint16_t uiTimeOut = 0;

    do {
        res = mcp2515_getNextFreeTXBuf(&txbuf_n);                       /* info = addr.                 */
        uiTimeOut++;
    } while (res == MCP_ALLTXBUSY && (uiTimeOut < TIMEOUTVALUE));

    if(uiTimeOut == TIMEOUTVALUE) 
    {   
        return CAN_GETTXBFTIMEOUT;                                      /* get tx buff time out         */
    }
    uiTimeOut = 0;
    mcp2515_write_canMsg( txbuf_n);
    mcp2515_start_transmit( txbuf_n );
    do
    {
        uiTimeOut++;        
        res1= mcp2515_readRegister(txbuf_n);  			                /* read send buff ctrl reg 	*/
        res1 = res1 & 0x08;                               		
    }while(res1 && (uiTimeOut < TIMEOUTVALUE));   
    if(uiTimeOut == TIMEOUTVALUE)                                       /* send msg timeout             */	
    {
        return CAN_SENDMSGTIMEOUT;
    }
    return CAN_OK;

}

/*********************************************************************************************************
** Function name:           sendMsgBuf
** Description:             send buf
*********************************************************************************************************/
byte MCP_CAN::sendMsgBuf(INT32U id, byte ext, byte rtr, byte len, byte *buf)
{
    setMsg(id, ext, rtr, len, buf);
    sendMsg();
}

/*********************************************************************************************************
** Function name:           readMsg
** Description:             read message
*********************************************************************************************************/
byte MCP_CAN::readMsg()
{
    byte stat, res;

    stat = mcp2515_readStatus();

    if ( stat & MCP_STAT_RX0IF )                                        /* Msg in Buffer 0              */
    {
        mcp2515_read_canMsg( MCP_RXBUF_0);
        mcp2515_modifyRegister(MCP_CANINTF, MCP_RX0IF, 0);
        res = CAN_OK;
    }
    else if ( stat & MCP_STAT_RX1IF )                                   /* Msg in Buffer 1              */
    {
        mcp2515_read_canMsg( MCP_RXBUF_1);
        mcp2515_modifyRegister(MCP_CANINTF, MCP_RX1IF, 0);
        res = CAN_OK;
    }
    else 
    {
        res = CAN_NOMSG;
    }
    return res;
}

/*********************************************************************************************************
** Function name:           readMsgBuf
** Description:             read message buf
*********************************************************************************************************/
byte MCP_CAN::readMsgBuf(byte *len, byte buf[])
{
    readMsg();
    *len = m_nDlc;
    for(int i = 0; i<m_nDlc; i++)
    {
      buf[i] = m_nDta[i];
    }
}

/*********************************************************************************************************
** Function name:           checkReceive
** Description:             check if got something
*********************************************************************************************************/
byte MCP_CAN::checkReceive(void)
{
    byte res;
    res = mcp2515_readStatus();                                         /* RXnIF in Bit 1 and 0         */
    if ( res & MCP_STAT_RXIF_MASK ) 
    {
        return CAN_MSGAVAIL;
    }
    else 
    {
        return CAN_NOMSG;
    }
}


/*********************************************************************************************************
** Function name:           getCanId
** Description:             when receive something ,u can get the can id!!
*********************************************************************************************************/
INT32U MCP_CAN::getCanId(void)
{
    return m_nID;
}

//------Reception Configuration-----------SEG

/*********************************************************************************************************
** Function name:           enableRXBuf0Filters
** Description:             This function allows the user to pick an opeation mode for the recieve buffers
**                          of the MCP2515. This rule determines which kinds of messages are placed in
**                          recieve buffer 0.
**
**                          Options for mode are as follows:
**                          RXBUFMSK_ALL - Disable Mask/Filters (All messages are allowed in RXB0).
**                          RXBUFMSK_EXT - Allow only valid messages using extended identifiers.
**                          RXBUFMSK_STD - Allow only valid messages using standard identifiers.
**                          RXBUFMSK_VLD - Allow all valid messages that match Masks/Filters.
*********************************************************************************************************/
void MCP_CAN::enableRXBuf0Filters(byte mode){
    mcp2515_setCANCTRL_Mode(MODE_CONFIG);                               // Enter Configuration Mode. *WARNING: THIS WILL FLUSH ALL STATUS BUFFERS*

    if(mode == RXBUFMSK_ALL){                                           // If mode is RXBUFMSK_ALL
        mcp2515_modifyRegister(RXB0CTRL, RXM1|RXM0, RXBUFMSK_ALL);      // Edit those bits to match
    }
    else if(mode == RXBUFMSK_EXT){                                      // If mode is RXBUFMSK_EXT
        mcp2515_modifyRegister(RXB0CTRL, RXM1|RXM0, RXBUFMSK_EXT);      // Edit those bits to match
    }                                                                   
    else if(mode == RXBUFMSK_STD){                                      // If mode is RXBUFMSK_STD
        mcp2515_modifyRegister(RXB0CTRL, RXM1|RXM0, RXBUFMSK_STD);      // Edit those bits to match
    }                                                                   
    else if(mode == RXBUFMSK_VLD){                                      // If mode is RXBUFMSK_VLD
        mcp2515_modifyRegister(RXB0CTRL, RXM1|RXM0, RXBUFMSK_VLD);      // Edit those bits to match
    }                                                                   

    mcp2515_setCANCTRL_Mode(MODE_NORMAL);                               // Return to normal operation

    return;
}

/*********************************************************************************************************
** Function name:           getRXBuf0FilterHit
** Description:             This fucntion returns a byte containing the contents of the FILHIT bits
**                          which indicate which filter allowed the message currently held in Recieve
**                          buffer 0 through. Each possible return has a macro associated with it
**                          for easy comparison.
**
**                          Return options are as follows:
**                          RXBUFFIL_HIT0 - Filter 0 allowed the message through.
**                          RXBUFFIL_HIT1 - Filter 1 allowed the message through.             
*********************************************************************************************************/
byte MCP_CAN::getRXBuf0FilterHit(){
    byte val = mcp2515_readRegister(RXB0CTRL);                          // Get the values in RXB0CTRL

    val = (val & FILHIT0);                                              // Mask out unrelated bits in the register

    return val;                                                         // The remaining bits will match one of the possible output macros
}

/*********************************************************************************************************
** Function name:           enableRXBuf1Filters
** Description:             This function allows the user to pick an opeation mode for the recieve buffers
**                          of the MCP2515. This rule determines which kinds of messages are placed in
**                          recieve buffer 1.
**
**                          Options for mode are as follows:
**                          RXBUFMSK_ALL - Disable Mask/Filters (All messages are allowed in RXB1).
**                          RXBUFMSK_EXT - Allow only valid messages using extended identifiers.
**                          RXBUFMSK_STD - Allow only valid messages using standard identifiers.
**                          RXBUFMSK_VLD - Allow all valid messages that match Masks/Filters.
*********************************************************************************************************/
void MCP_CAN::enableRXBuf1Filters(byte mode){
    mcp2515_setCANCTRL_Mode(MODE_CONFIG);                               // Enter Configuration Mode. *WARNING: THIS WILL FLUSH ALL STATUS BUFFERS*

    if(mode == RXBUFMSK_ALL){                                           // If mode is RXBUFMSK_ALL
        mcp2515_modifyRegister(RXB1CTRL, RXM1|RXM0, RXBUFMSK_ALL);      // Edit those bits to match
    }
    else if(mode == RXBUFMSK_EXT){                                      // If mode is RXBUFMSK_EXT
        mcp2515_modifyRegister(RXB1CTRL, RXM1|RXM0, RXBUFMSK_EXT);      // Edit those bits to match
    }                                                                   
    else if(mode == RXBUFMSK_STD){                                      // If mode is RXBUFMSK_STD
        mcp2515_modifyRegister(RXB1CTRL, RXM1|RXM0, RXBUFMSK_STD);      // Edit those bits to match
    }                                                                   
    else if(mode == RXBUFMSK_VLD){                                      // If mode is RXBUFMSK_VLD
        mcp2515_modifyRegister(RXB1CTRL, RXM1|RXM0, RXBUFMSK_VLD);      // Edit those bits to match
    }                                                                   

    mcp2515_setCANCTRL_Mode(MODE_NORMAL);                               // Return to normal operation

    return;
}

/*********************************************************************************************************
** Function name:           getRXBuf1FilterHit
** Description:             This fucntion returns a byte containing the contents of the FILHIT bits
**                          which indicate which filter allowed the message currently held in recieve
**                          buffer 1 through. Each possible return has a macro associated with it
**                          for easy comparison.
**
**                          Return options are as follows:
**                          RXBUFFIL_HIT0 - Filter 0 allowed the message through.
**                          RXBUFFIL_HIT1 - Filter 1 allowed the message through.
**                          RXBUFFIL_HIT2 - Filter 2 allowed the message through.
**                          RXBUFFIL_HIT3 - Filter 3 allowed the message through.
**                          RXBUFFIL_HIT4 - Filter 4 allowed the message through.
**                          RXBUFFIL_HIT5 - Filter 5 allowed the message through.                  
*********************************************************************************************************/
byte MCP_CAN::getRXBuf1FilterHit(){
    byte val = mcp2515_readRegister(RXB1CTRL);                          // Get the values in RXB1CTRL

    val = (val & (FILHIT2|FILHIT1|FILHIT0));                            // Mask out unrelated bits in the register

    return val;                                                         // The remaining bits will match one of the possible output macros
}

/*********************************************************************************************************
** Function name:           setRollover
** Description:             This function enables or disables the rollover of messages from recieve buffer
**                          0 to recieve buffer 0.
**
**                          Options for mode are as follows:
**                          true - Enable rollover
**                          false - Disable rollover
*********************************************************************************************************/
void MCP_CAN::setRollover(bool mode){
    mcp2515_setCANCTRL_Mode(MODE_CONFIG);                               // Enter Configuration Mode. *WARNING: THIS WILL FLUSH ALL STATUS BUFFERS*

    if(mode == true){                                                   // If mode is true we want enable rollover
        mcp2515_modifyRegister(RXB0CTRL, BUKT, RXBUF0BUKT_EN);          // Set rollover bit
    }
    else if(mode == false){                                             // If mode is false we want to disable rollover
        mcp2515_modifyRegister(RXB0CTRL, BUKT, RXBUF0BUKT_DIS);         // Clear rollover bit
    }

    mcp2515_setCANCTRL_Mode(MODE_NORMAL);                               // Return to normal operation
}

/*********************************************************************************************************
** Function name:           getBuf0RTR
** Description:             This function returns a byte containing the contents of the RXRTR bit, which
**                          indicates whether the current message in recieve buffer 0 is a Remote Transfer
**                          Request.
**
**                          Return options are as follows:
**                          RXBUFRTR_SET - The message is a Remote Transfer Request
**                          RXBUFRTR_CLR - The message is a normal transmission
*********************************************************************************************************/
byte MCP_CAN::getBuf0RTR(){
    byte val = mcp2515_readRegister(RXB0CTRL);                          // Get the values in RXB0CTRL

    val = (val & RXRTR);                                                // Mask out unrelated bits in the register

    return val;                                                         // The remaining bits will match one of the possible output macros
}

/*********************************************************************************************************
** Function name:           getBuf0RTR
** Description:             This function returns a byte containing the contents of the RXRTR bit, which
**                          indicates whether the current message in recieve buffer 1 is a Remote Transfer
**                          Request.
**
**                          Return options are as follows:
**                          RXBUFRTR_SET - The message is a Remote Transfer Request
**                          RXBUFRTR_CLR - The message is a normal transmission
*********************************************************************************************************/
byte MCP_CAN::getBuf1RTR(){
    
    byte val = mcp2515_readRegister(RXB1CTRL);                          // Get the values in RXB1CTRL

    val = (val & RXRTR);                                                // Mask out unrelated bits in the register

    return val;                                                         // The remaining bits will match one of the possible output macros
}


//------Masks and Filters Configuration-----------Cory J. Fowler (modified by SEG)

/*********************************************************************************************************
** Function name:           setRXFilt
** Description:             This function sets the mask registers for use. The mask registers indicate
**                          which bits of incoming messages should be subjected to the filters.
**
**                          Options are as follows:
**                          For num:
**                              0-5 - The mask to change is Mask 0-5
**                          For extended:
**                              false - Do not apply the data to the extended bits, clear all extended bits
**                              true - Apply the data to the extended bits
**                          For maskBits:
**                              Pass an unsigned long filled with the bytes for the mask. The extended
**                              identifier low byte is the lowest byte and the standard identifier high byte
**                              is the highest.
**                              NOTE: The upper 3 bits are unused
*********************************************************************************************************/
void MCP_CAN::setRXFilt(int num, bool extended, INT32U filtBits){
    mcp2515_setCANCTRL_Mode(MODE_CONFIG);                               // Enter Configuration Mode. *WARNING: THIS WILL FLUSH ALL STATUS BUFFERS*
    
    switch(num)
    {
        case 0:                                                         // If the filter we want to change is 0
        mcp2515_write_id(RXF0SIDH, extended, filtBits);                 // Write the data to the proper register
        break;

        case 1:                                                         // If the filter we want to change is 1
        mcp2515_write_id(RXF1SIDH, extended, filtBits);                 // Write the data to the proper register
        break;

        case 2:                                                         // If the filter we want to change is 2
        mcp2515_write_id(RXF2SIDH, extended, filtBits);                 // Write the data to the proper register
        break;

        case 3:                                                         // If the filter we want to change is 3
        mcp2515_write_id(RXF3SIDH, extended, filtBits);                 // Write the data to the proper register
        break;

        case 4:                                                         // If the filter we want to change is 4
        mcp2515_write_id(RXF4SIDH, extended, filtBits);                 // Write the data to the proper register
        break;

        case 5:                                                         // If the filter we want to change is 5
        mcp2515_write_id(RXF5SIDH, extended, filtBits);                 // Write the data to the proper register
        break;

        default:
        break;
    }
    
    mcp2515_setCANCTRL_Mode(MODE_NORMAL);                               // Return to normal operation

    return;
}

/*********************************************************************************************************
** Function name:           setRXMask
** Description:             This function sets the mask registers for use. The mask registers indicate
**                          which bits of incoming messages should be subjected to the filters.
**
**                          Options are as follows:
**                          For num:
**                              0 - The mask to change is Mask 0
**                              1 - The mask to change is Mask 1
**                          For extended:
**                              false - Do not apply the data to the extended bits, clear all extended bits
**                              true - Apply the data to the extended bits
**                          For maskBits:
**                              Pass an unsigned long filled with the bytes for the mask. The extended
**                              identifier low byte is the lowest byte and the standard identifier high byte
**                              is the highest.
**                              NOTE: The upper 3 bits are unused
*********************************************************************************************************/
void MCP_CAN::setRXMask(int num, bool extended, INT32U maskBits){
    mcp2515_setCANCTRL_Mode(MODE_CONFIG);                               // Enter Configuration Mode. *WARNING: THIS WILL FLUSH ALL STATUS BUFFERS*

    if (num == 0){                                                      // If the mask we want to change is 0
        mcp2515_write_id(RXM0SIDH, extended, maskBits);                 // Write the data to the proper register

    }
    else if(num == 1){                                                  // If the mask we want to change is 1
        mcp2515_write_id(RXM1SIDH, extended, maskBits);                 // Write the data to the proper register
    }
    
    mcp2515_setCANCTRL_Mode(MODE_NORMAL);

    return;                                                             // Return to normal operation
}

//------Interrupt Configuration-----------SEG

/*********************************************************************************************************
** Function name:           enableInt
** Description:             This function enables the various sources for interrupts on the interrupt pin
**
**                          Options for intBits are as follows:
**                          MERRE - Enables the message error interrupt
**                          WAKIE - Enables the wake interrupt
**                          ERRIE - Enables the error interrupt
**                          TX2IE - Enables the transmit buffer 2 empty interrupt
**                          TX1IE - Enables the transmit buffer 1 empty interrupt
**                          TX0IE - Enables the transmit buffer 0 empty interrupt
**                          RX1IE - Enables the recieve buffer 1 full interrupt
**                          RX0IE - Enables the recieve buffer 0 full interrupt
**                          Or any combinations of the above combined via OR.
**                          Ex: (MERRE|TX1IE|RX0IE) - message error, transmit 1 empty, and recieve 0 full
*********************************************************************************************************/
void MCP_CAN::enableInt(byte intBits){
    mcp2515_setRegister(CANINTE, intBits);                              // Set the value of the interrupt register to match wanted interrupts
}

/*********************************************************************************************************
** Function name:           getInt
** Description:             This function returns a 3 bit value representing which interrupt triggered
**                          the interrupt pin.
**
**                          Return options are as follows:
**                          NOI - No interrupt flags are enabled
**                          ERR - Error interrupt toggled pin
**                          WAK - Wake interrupt toggled pin
**                          TX0 - Transmit 0 interrupt toggled pin
**                          TX1 - Transmit 1 interrupt toggled pin
**                          TX2 - Transmit 2 interrupt toggled pin
**                          RX0 - Recieve 0 interrupt toggled pin
**                          RX1 - Recieve 1 interrupt toggled pin
**                          Note: If multiple sources toggle interrupt, you will read the code from the
**                          top of this list to the bottom, and as each are cleared a new value will be
**                          take its place.
*********************************************************************************************************/
byte MCP_CAN::getInt(){
    byte val = mcp2515_readRegister(CANSTAT);                           // Get the value of the register containing the ICOD values

    val = ((val & (ICOD2|ICOD1|ICOD0)) >> 1);                           // Mask out and shift the bits we want

    return val;                                                         // The bits of val will match one of the possible output macros
}

/*********************************************************************************************************
** Function name:           clearInt
** Description:             This function clears an interrupt, so that the interrupt pin will return to
**                          being high. This must be done after responding to the interrupt to move on
**                          to the next interrupt source or return to normal operation.
**
**                          Options for intBit are as follows:
**                          MERRF - Clear message error interrupt
**                          WAKIF - Clear wake interrupt
**                          ERRIF - Clear error interrupt
**                          TX2IF - Clear transmit buffer 2 empty interrupt
**                          TX1IF - Clear transmit buffer 1 empty interrupt
**                          TX0IF - Clear trasnmit buffer 0 empty interrupt
**                          RX1IF - Clear recieve buffer 1 full interrupt
**                          RX0IF - Clear recieve buffer 0 full interrupt
*********************************************************************************************************/
void MCP_CAN::clearInt(int intBit){
    mcp2515_modifyRegister(CANINTF, intBit, 0x00);                      // Clear the bit representing that interrupt being toggled
}







//!!!!!!!!!!!!!!!!GABRIEL MARIA FUNCTION FOR RTR!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
bool MCP_CAN::getCanRTR(void)
{
    return (bool)m_nRtr;
}


//!!!!!!!!!!!!!!!!GABRIEL MARIA FUNCTIONS FOR ERRORS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
byte MCP_CAN::getTxErrors(void)                                    
{
   return mcp2515_readRegister(TEC); //returns 0-255 amount of Tx errrors, this counter flips quickly, READ MANUAL!
}

byte MCP_CAN::getRxErrors(void)                                    
{
   return mcp2515_readRegister(REC); //returns 0-255 amount of Rx errrors, this counter flips quickly, READ MANUAL!
}

boolean MCP_CAN::getErrorFlag(byte mask) //mask = RX1OVR, RX0OVR, TXBO, TXEP, RXEP, TXWAR, RXWAR, or EWARN
{
    boolean bitVal = 0;
    byte regVal = mcp2515_readRegister(EFLG);
    if((regVal && mask) > 0){ return 1; }     //if say, regVal=00101100, mask=00100000=32=TXBO, && them will equal 00100000, the bit is true   
    else{ return 0; }                         // else it will equal 00000000 and the bit is false
}

void MCP_CAN::setErrorFlag(byte mask, boolean val) //only RX1OVR and RX0OVR
{
   if((mask == RX1OVR) || (mask == RX0OVR)) {
     mcp2515_modifyRegister(EFLG, mask, (byte)val);
   }
}


//PSRCAN error function
//returns true if TXWAR or RXWAR is true, that is, if TX or RX error counter is over 96
/*
boolean getBusError(){
    return (  (getErrorFlag(TXWAR);) | (getErrorFlag(RXWAR);)  )
}
*/




























/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
