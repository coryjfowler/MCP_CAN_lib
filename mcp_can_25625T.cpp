#include "mcp_can_25625T.h"

#define spi_readwrite mcpSPI->transfer
#define spi_read() spi_readwrite(0x00)

/*********************************************************************************************************
** Function name:           mcp25625_reset
** Descriptions:            Performs a software reset
*********************************************************************************************************/
void MCP_CAN::mcp25625_reset(void)                                      
{
    mcpSPI->beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
    MCP25625_SELECT();
    spi_readwrite(MCP25625_RESET);
    MCP25625_UNSELECT();
    mcpSPI->endTransaction();
    delay(5); // If the MCP25625 was in sleep mode when the reset command was issued then we need to wait a while for it to reset properly
}

/*********************************************************************************************************
** Function name:           mcp25625_readRegister
** Descriptions:            Read data register
*********************************************************************************************************/
INT8U MCP_CAN::mcp25625_readRegister(const INT8U address)                                                                     
{
    INT8U ret;

    mcpSPI->beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
    MCP25625_SELECT();
    spi_readwrite(MCP25625_READ);
    spi_readwrite(address);
    ret = spi_read();
    MCP25625_UNSELECT();
    mcpSPI->endTransaction();

    return ret;
}

/*********************************************************************************************************
** Function name:           mcp25625_readRegisterS
** Descriptions:            Reads successive data registers
*********************************************************************************************************/
void MCP_CAN::mcp25625_readRegisterS(const INT8U address, INT8U values[], const INT8U n)
{
    INT8U i;
    mcpSPI->beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
    MCP25625_SELECT();
    spi_readwrite(MCP25625_READ);
    spi_readwrite(address);
    for (i=0; i<n; i++) 
        values[i] = spi_read();

    MCP25625_UNSELECT();
    mcpSPI->endTransaction();
}

/*********************************************************************************************************
** Function name:           mcp25625_setRegister
** Descriptions:            Sets data register
*********************************************************************************************************/
void MCP_CAN::mcp25625_setRegister(const INT8U address, const INT8U value)
{
    mcpSPI->beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
    MCP25625_SELECT();
    spi_readwrite(MCP25625_WRITE);
    spi_readwrite(address);
    spi_readwrite(value);
    MCP25625_UNSELECT();
    mcpSPI->endTransaction();
}

/*********************************************************************************************************
** Function name:           mcp25625_setRegisterS
** Descriptions:            Sets successive data registers
*********************************************************************************************************/
void MCP_CAN::mcp25625_setRegisterS(const INT8U address, const INT8U values[], const INT8U n)
{
    INT8U i;
    mcpSPI->beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
    MCP25625_SELECT();
    spi_readwrite(MCP25625_WRITE);
    spi_readwrite(address);
       
    for (i=0; i<n; i++) 
        spi_readwrite(values[i]);
	
    MCP25625_UNSELECT();
    mcpSPI->endTransaction();
}

/*********************************************************************************************************
** Function name:           mcp25625_modifyRegister
** Descriptions:            Sets specific bits of a register
*********************************************************************************************************/
void MCP_CAN::mcp25625_modifyRegister(const INT8U address, const INT8U mask, const INT8U data)
{
    mcpSPI->beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
    MCP25625_SELECT();
    spi_readwrite(MCP25625_BITMOD);
    spi_readwrite(address);
    spi_readwrite(mask);
    spi_readwrite(data);
    MCP25625_UNSELECT();
    mcpSPI->endTransaction();
}

/*********************************************************************************************************
** Function name:           mcp25625_readStatus
** Descriptions:            Reads status register
*********************************************************************************************************/
INT8U MCP_CAN::mcp25625_readStatus(void)                             
{
    INT8U i;
    mcpSPI->beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
    MCP25625_SELECT();
    spi_readwrite(MCP25625_READ_STATUS);
    i = spi_read();
    MCP25625_UNSELECT();
    mcpSPI->endTransaction();
    return i;
}

/*********************************************************************************************************
** Function name:           setSleepWakeup
** Descriptions:            Enable or disable the wake up interrupt (If disabled the MCP25625 will not be woken up by CAN bus activity)
*********************************************************************************************************/
void MCP_CAN::setSleepWakeup(const INT8U enable)
{
    mcp25625_modifyRegister(MCP_CANINTE, MCP_WAKIF, enable ? MCP_WAKIF : 0);
}

/*********************************************************************************************************
** Function name:           setMode
** Descriptions:            Sets control mode
*********************************************************************************************************/
INT8U MCP_CAN::setMode(const INT8U opMode)
{
    mcpMode = opMode;
    return mcp25625_setCANCTRL_Mode(mcpMode);
}

/*********************************************************************************************************
** Function name:           mcp25625_setCANCTRL_Mode
** Descriptions:            Set control mode
*********************************************************************************************************/
INT8U MCP_CAN::mcp25625_setCANCTRL_Mode(const INT8U newmode)
{
	// If the chip is asleep and we want to change mode then a manual wake needs to be done
	// This is done by setting the wake up interrupt flag
	if((mcp25625_readRegister(MCP_CANSTAT) & MODE_MASK) == MCP_SLEEP && newmode != MCP_SLEEP)
	{
		// Make sure wake interrupt is enabled
		byte wakeIntEnabled = (mcp25625_readRegister(MCP_CANINTE) & MCP_WAKIF);
		if(!wakeIntEnabled)
			mcp25625_modifyRegister(MCP_CANINTE, MCP_WAKIF, MCP_WAKIF);

		// Set wake flag (this does the actual waking up)
		mcp25625_modifyRegister(MCP_CANINTF, MCP_WAKIF, MCP_WAKIF);

		// Wait for the chip to exit SLEEP and enter LISTENONLY mode.
		if(mcp25625_requestNewMode(MCP_LISTENONLY) != MCP25625_OK)
			return MCP25625_FAIL;

		// Turn wake interrupt back off if it was originally off
		if(!wakeIntEnabled)
			mcp25625_modifyRegister(MCP_CANINTE, MCP_WAKIF, 0);
	}

	// Clear wake flag
	mcp25625_modifyRegister(MCP_CANINTF, MCP_WAKIF, 0);
	
	return mcp25625_requestNewMode(newmode);
}

/*********************************************************************************************************
** Function name:           mcp25625_requestNewMode
** Descriptions:            Set control mode
*********************************************************************************************************/
INT8U MCP_CAN::mcp25625_requestNewMode(const INT8U newmode)
{
	byte startTime = millis();

	// Spam new mode request and wait for the operation to complete
	while(1)
	{
		// Request new mode
		mcp25625_modifyRegister(MCP_CANCTRL, MODE_MASK, newmode); 

		byte statReg = mcp25625_readRegister(MCP_CANSTAT);
		if((statReg & MODE_MASK) == newmode)
			return MCP25625_OK;
		else if((byte)(millis() - startTime) > 200)
			return MCP25625_FAIL;
	}
}

/*********************************************************************************************************
** Function name:           mcp25625_configRate
** Descriptions:            Set baudrate
*********************************************************************************************************/
INT8U MCP_CAN::mcp25625_configRate(const INT8U canSpeed, const INT8U canClock)            
{
    INT8U set, cfg1, cfg2, cfg3;
    set = 1;
    switch (canClock & MCP_CLOCK_SELECT)
    {
        case (MCP_8MHZ):
        switch (canSpeed) 
        {
            case (CAN_5KBPS):
            cfg1 = MCP_8MHz_5kBPS_CFG1;
            cfg2 = MCP_8MHz_5kBPS_CFG2;
            cfg3 = MCP_8MHz_5kBPS_CFG3;
            break;

            // Add all other cases as in the original library, updating the config constants
            default:
            set = 0;
	    return MCP25625_FAIL;
            break;
        }
        break;

        // Add cases for MCP_16MHZ and MCP_20MHZ as in the original library

        default:
        set = 0;
	return MCP25625_FAIL;
        break;
    }

    if (canClock & MCP_CLKOUT_ENABLE) {
        cfg3 &= (~SOF_ENABLE);
    }

    if (set) {
        mcp25625_setRegister(MCP_CNF1, cfg1);
        mcp25625_setRegister(MCP_CNF2, cfg2);
        mcp25625_setRegister(MCP_CNF3, cfg3);
        return MCP25625_OK;
    }
     
    return MCP25625_FAIL;
}

/*********************************************************************************************************
** Function name:           mcp25625_initCANBuffers
** Descriptions:            Initialize Buffers, Masks, and Filters
*********************************************************************************************************/
void MCP_CAN::mcp25625_initCANBuffers(void)
{
    INT8U i, a1, a2, a3;
    
    INT8U std = 0;               
    INT8U ext = 1;
    INT32U ulMask = 0x00, ulFilt = 0x00;

    mcp25625_write_mf(MCP_RXM0SIDH, ext, ulMask);
    mcp25625_write_mf(MCP_RXM1SIDH, ext, ulMask);
    
    mcp25625_write_mf(MCP_RXF0SIDH, ext, ulFilt);
    mcp25625_write_mf(MCP_RXF1SIDH, std, ulFilt);
    mcp25625_write_mf(MCP_RXF2SIDH, ext, ulFilt);
    mcp25625_write_mf(MCP_RXF3SIDH, std, ulFilt);
    mcp25625_write_mf(MCP_RXF4SIDH, ext, ulFilt);
    mcp25625_write_mf(MCP_RXF5SIDH, std, ulFilt);

    a1 = MCP_TXB0CTRL;
    a2 = MCP_TXB1CTRL;
    a3 = MCP_TXB2CTRL;
    for (i = 0; i < 14; i++) {
        mcp25625_setRegister(a1, 0);
        mcp25625_setRegister(a2, 0);
        mcp25625_setRegister(a3, 0);
        a1++;
        a2++;
        a3++;
    }
    mcp25625_setRegister(MCP_RXB0CTRL, 0);
    mcp25625_setRegister(MCP_RXB1CTRL, 0);
}

/*********************************************************************************************************
** Function name:           mcp25625_init
** Descriptions:            Initialize the controller
*********************************************************************************************************/
INT8U MCP_CAN::mcp25625_init(const INT8U canIDMode, const INT8U canSpeed, const INT8U canClock)
{
    INT8U res;

    mcp25625_reset();
    
    mcpMode = MCP_LOOPBACK;

    res = mcp25625_setCANCTRL_Mode(MODE_CONFIG);
    if(res > 0)
        return res;

    if(mcp25625_configRate(canSpeed, canClock))
        return res;

    if ( res == MCP25625_OK ) {
        mcp25625_initCANBuffers();
        mcp25625_setRegister(MCP_CANINTE, MCP_RX0IF | MCP_RX1IF);

        mcp25625_setRegister(MCP_BFPCTRL, MCP_BxBFS_MASK | MCP_BxBFE_MASK);
        mcp25625_setRegister(MCP_TXRTSCTRL, 0x00);

        switch(canIDMode)
        {
            case (MCP_ANY):
            mcp25625_modifyRegister(MCP_RXB0CTRL,
            MCP_RXB_RX_MASK | MCP_RXB_BUKT_MASK,
            MCP_RXB_RX_ANY | MCP_RXB_BUKT_MASK);
            mcp25625_modifyRegister(MCP_RXB1CTRL, MCP_RXB_RX_MASK,
            MCP_RXB_RX_ANY);
            break;
            case (MCP_STDEXT):
            mcp25625_modifyRegister(MCP_RXB0CTRL,
            MCP_RXB_RX_MASK | MCP_RXB_BUKT_MASK,
            MCP_RXB_RX_STDEXT | MCP_RXB_BUKT_MASK );
            mcp25625_modifyRegister(MCP_RXB1CTRL, MCP_RXB_RX_MASK,
            MCP_RXB_RX_STDEXT);
            break;
            default:
            return MCP25625_FAIL;
            break;
        }

        res = mcp25625_setCANCTRL_Mode(mcpMode);
        if(res)
            return res;
    }
    return res;
}

/*********************************************************************************************************
** Function name:           mcp25625_write_id
** Descriptions:            Write CAN ID
*********************************************************************************************************/
void MCP_CAN::mcp25625_write_id(const INT8U mcp_addr, const INT8U ext, const INT32U id)
{
    uint16_t canid;
    INT8U tbufdata[4];

    canid = (uint16_t)(id & 0x0FFFF);

    if (ext == 1)
    {
        tbufdata[MCP_EID0] = (INT8U) (canid & 0xFF);
        tbufdata[MCP_EID8] = (INT8U) (canid >> 8);
        canid = (uint16_t)(id >> 16);
        tbufdata[MCP_SIDL] = (INT8U) (canid & 0x03);
        tbufdata[MCP_SIDL] += (INT8U) ((canid & 0x1C) << 3);
        tbufdata[MCP_SIDL] |= MCP_TXB_EXIDE_M;
        tbufdata[MCP_SIDH] = (INT8U) (canid >> 5 );
    }
    else
    {
        tbufdata[MCP_SIDH] = (INT8U) (canid >> 3 );
        tbufdata[MCP_SIDL] = (INT8U) ((canid & 0x07 ) << 5);
        tbufdata[MCP_EID0] = 0;
        tbufdata[MCP_EID8] = 0;
    }
    
    mcp25625_setRegisterS(mcp_addr, tbufdata, 4);
}

/*********************************************************************************************************
** Function name:           mcp25625_write_mf
** Descriptions:            Write Masks and Filters
*********************************************************************************************************/
void MCP_CAN::mcp25625_write_mf(const INT8U mcp_addr, const INT8U ext, const INT32U id)
{
    uint16_t canid;
    INT8U tbufdata[4];

    canid = (uint16_t)(id & 0x0FFFF);

    if (ext == 1)
    {
        tbufdata[MCP_EID0] = (INT8U) (canid & 0xFF);
        tbufdata[MCP_EID8] = (INT8U) (canid >> 8);
        canid = (uint16_t)(id >> 16);
        tbufdata[MCP_SIDL] = (INT8U) (canid & 0x03);
        tbufdata[MCP_SIDL] += (INT8U) ((canid & 0x1C) << 3);
        tbufdata[MCP_SIDL] |= MCP_TXB_EXIDE_M;
        tbufdata[MCP_SIDH] = (INT8U) (canid >> 5 );
    }
    else
    {
        tbufdata[MCP_EID0] = (INT8U) (canid & 0xFF);
        tbufdata[MCP_EID8] = (INT8U) (canid >> 8);
        canid = (uint16_t)(id >> 16);
        tbufdata[MCP_SIDL] = (INT8U) ((canid & 0x07) << 5);
        tbufdata[MCP_SIDH] = (INT8U) (canid >> 3 );
    }
    
    mcp25625_setRegisterS(mcp_addr, tbufdata, 4);
}

/*********************************************************************************************************
** Function name:           mcp25625_read_id
** Descriptions:            Read CAN ID
*********************************************************************************************************/
void MCP_CAN::mcp25625_read_id(const INT8U mcp_addr, INT8U* ext, INT32U* id)
{
    INT8U tbufdata[4];

    *ext = 0;
    *id = 0;

    mcp25625_readRegisterS(mcp_addr, tbufdata, 4);

    *id = (tbufdata[MCP_SIDH]<<3) + (tbufdata[MCP_SIDL]>>5);

    if ((tbufdata[MCP_SIDL] & MCP_TXB_EXIDE_M) == MCP_TXB_EXIDE_M)
    {
        *id = (*id<<2) + (tbufdata[MCP_SIDL] & 0x03);
        *id = (*id<<8) + tbufdata[MCP_EID8];
        *id = (*id<<8) + tbufdata[MCP_EID0];
        *ext = 1;
    }
}

/*********************************************************************************************************
** Function name:           mcp25625_write_canMsg
** Descriptions:            Write message
*********************************************************************************************************/
void MCP_CAN::mcp25625_write_canMsg(const INT8U buffer_sidh_addr)
{
    INT8U mcp_addr;
    mcp_addr = buffer_sidh_addr;
    mcp25625_setRegisterS(mcp_addr+5, m_nDta, m_nDlc);
	
    if (m_nRtr == 1)
        m_nDlc |= MCP_RTR_MASK;

    mcp25625_setRegister((mcp_addr+4), m_nDlc);
    mcp25625_write_id(mcp_addr, m_nExtFlg, m_nID);
}

/*********************************************************************************************************
** Function name:           mcp25625_read_canMsg
** Descriptions:            Read message
*********************************************************************************************************/
void MCP_CAN::mcp25625_read_canMsg(const INT8U buffer_sidh_addr)
{
    INT8U mcp_addr, ctrl;

    mcp_addr = buffer_sidh_addr;

    mcp25625_read_id(mcp_addr, &m_nExtFlg, &m_nID);

    ctrl = mcp25625_readRegister(mcp_addr-1);
    m_nDlc = mcp25625_readRegister(mcp_addr+4);

    if (ctrl & 0x08)
        m_nRtr = 1;
    else
        m_nRtr = 0;

    m_nDlc &= MCP_DLC_MASK;
    mcp25625_readRegisterS(mcp_addr+5, &(m_nDta[0]), m_nDlc);
}

/*********************************************************************************************************
** Function name:           mcp25625_getNextFreeTXBuf
** Descriptions:            Send message
*********************************************************************************************************/
INT8U MCP_CAN::mcp25625_getNextFreeTXBuf(INT8U *txbuf_n)
{
    INT8U res, i, ctrlval;
    INT8U ctrlregs[MCP_N_TXBUFFERS] = { MCP_TXB0CTRL, MCP_TXB1CTRL, MCP_TXB2CTRL };

    res = MCP_ALLTXBUSY;
    *txbuf_n = 0x00;

    for (i=0; i<MCP_N_TXBUFFERS; i++) {
        ctrlval = mcp25625_readRegister(ctrlregs[i]);
        if ((ctrlval & MCP_TXB_TXREQ_M) == 0) {
            *txbuf_n = ctrlregs[i]+1;
            res = MCP25625_OK;
            return res;
        }
    }
    return res;
}

/*********************************************************************************************************
** Function name:           MCP_CAN
** Descriptions:            Public function to declare CAN class and the /CS pin.
*********************************************************************************************************/
MCP_CAN::MCP_CAN(INT8U _CS)
{
    MCPCS = _CS;
    MCP25625_UNSELECT();
    pinMode(MCPCS, OUTPUT);
    mcpSPI = &SPI;
}

/*********************************************************************************************************
** Function name:           MCP_CAN
** Descriptions:            Public function to declare CAN class with SPI and the /CS pin.
*********************************************************************************************************/
MCP_CAN::MCP_CAN(SPIClass *_SPI, INT8U _CS)
{
    MCPCS = _CS;
    MCP25625_UNSELECT();
    pinMode(MCPCS, OUTPUT);
    mcpSPI = _SPI;
}

/*********************************************************************************************************
** Function name:           begin
** Descriptions:            Public function to declare controller initialization parameters.
*********************************************************************************************************/
INT8U MCP_CAN::begin(INT8U idmodeset, INT8U speedset, INT8U clockset)
{
    INT8U res;

    mcpSPI->begin();
    res = mcp25625_init(idmodeset, speedset, clockset);
    if (res == MCP25625_OK)
        return CAN_OK;
    
    return CAN_FAILINIT;
}

/*********************************************************************************************************
** Function name:           init_Mask
** Descriptions:            Public function to set mask(s).
*********************************************************************************************************/
INT8U MCP_CAN::init_Mask(INT8U num, INT8U ext, INT32U ulData)
{
    INT8U res = MCP25625_OK;
    res = mcp25625_setCANCTRL_Mode(MODE_CONFIG);
    if(res > 0)
        return res;

    if (num == 0)
        mcp25625_write_mf(MCP_RXM0SIDH, ext, ulData);
    else if(num == 1)
        mcp25625_write_mf(MCP_RXM1SIDH, ext, ulData);
    else
        res =  MCP25625_FAIL;
    
    res = mcp25625_setCANCTRL_Mode(mcpMode);
    return res;
}

/*********************************************************************************************************
** Function name:           init_Mask
** Descriptions:            Public function to set mask(s).
*********************************************************************************************************/
INT8U MCP_CAN::init_Mask(INT8U num, INT32U ulData)
{
    INT8U res = MCP25625_OK;
    INT8U ext = 0;

    res = mcp25625_setCANCTRL_Mode(MODE_CONFIG);
    if(res > 0)
        return res;

    if((ulData & 0x80000000) == 0x80000000)
        ext = 1;
    
    if (num == 0)
        mcp25625_write_mf(MCP_RXM0SIDH, ext, ulData);
    else if(num == 1)
        mcp25625_write_mf(MCP_RXM1SIDH, ext, ulData);
    else
        res =  MCP25625_FAIL;
    
    res = mcp25625_setCANCTRL_Mode(mcpMode);
    return res;
}

/*********************************************************************************************************
** Function name:           init_Filt
** Descriptions:            Public function to set filter(s).
*********************************************************************************************************/
INT8U MCP_CAN::init_Filt(INT8U num, INT8U ext, INT32U ulData)
{
    INT8U res = MCP25625_OK;
    res = mcp25625_setCANCTRL_Mode(MODE_CONFIG);
    if(res > 0)
        return res;

    switch(num)
    {
        case 0:
        mcp25625_write_mf(MCP_RXF0SIDH, ext, ulData);
        break;

        case 1:
        mcp25625_write_mf(MCP_RXF1SIDH, ext, ulData);
        break;

        case 2:
        mcp25625_write_mf(MCP_RXF2SIDH, ext, ulData);
        break;

        case 3:
        mcp25625_write_mf(MCP_RXF3SIDH, ext, ulData);
        break;

        case 4:
        mcp25625_write_mf(MCP_RXF4SIDH, ext, ulData);
        break;

        case 5:
        mcp25625_write_mf(MCP_RXF5SIDH, ext, ulData);
        break;

        default:
        res = MCP25625_FAIL;
    }
    
    res = mcp25625_setCANCTRL_Mode(mcpMode);
    return res;
}

/*********************************************************************************************************
** Function name:           init_Filt
** Descriptions:            Public function to set filter(s).
*********************************************************************************************************/
INT8U MCP_CAN::init_Filt(INT8U num, INT32U ulData)
{
    INT8U res = MCP25625_OK;
    INT8U ext = 0;
    res = mcp25625_setCANCTRL_Mode(MODE_CONFIG);
    if(res > 0)
        return res;

    if((ulData & 0x80000000) == 0x80000000)
        ext = 1;
    
    switch(num)
    {
        case 0:
        mcp25625_write_mf(MCP_RXF0SIDH, ext, ulData);
        break;

        case 1:
        mcp25625_write_mf(MCP_RXF1SIDH, ext, ulData);
        break;

        case 2:
        mcp25625_write_mf(MCP_RXF2SIDH, ext, ulData);
        break;

        case 3:
        mcp25625_write_mf(MCP_RXF3SIDH, ext, ulData);
        break;

        case 4:
        mcp25625_write_mf(MCP_RXF4SIDH, ext, ulData);
        break;

        case 5:
        mcp25625_write_mf(MCP_RXF5SIDH, ext, ulData);
        break;

        default:
        res = MCP25625_FAIL;
    }
    
    res = mcp25625_setCANCTRL_Mode(mcpMode);
    return res;
}

/*********************************************************************************************************
** Function name:           setMsg
** Descriptions:            Set can message, such as dlc, id, dta[] and so on
*********************************************************************************************************/
INT8U MCP_CAN::setMsg(INT32U id, INT8U rtr, INT8U ext, INT8U len, INT8U *pData)
{
    int i = 0;
    m_nID     = id;
    m_nRtr    = rtr;
    m_nExtFlg = ext;
    m_nDlc    = len;
    for(i = 0; i<MAX_CHAR_IN_MESSAGE; i++)
        m_nDta[i] = *(pData+i);
	
    return MCP25625_OK;
}

/*********************************************************************************************************
** Function name:           clearMsg
** Descriptions:            Set all messages to zero
*********************************************************************************************************/
INT8U MCP_CAN::clearMsg()
{
    m_nID       = 0;
    m_nDlc      = 0;
    m_nExtFlg   = 0;
    m_nRtr      = 0;
    m_nfilhit   = 0;
    for(int i = 0; i<m_nDlc; i++)
      m_nDta[i] = 0x00;

    return MCP25625_OK;
}

/*********************************************************************************************************
** Function name:           sendMsg
** Descriptions:            Send message
*********************************************************************************************************/
INT8U MCP_CAN::sendMsg()
{
    INT8U res, res1, txbuf_n;
    uint32_t uiTimeOut, temp;

    temp = micros();
    do {
        res = mcp25625_getNextFreeTXBuf(&txbuf_n);
        uiTimeOut = micros() - temp;
    } while (res == MCP_ALLTXBUSY && (uiTimeOut < TIMEOUTVALUE));

    if(uiTimeOut >= TIMEOUTVALUE)
        return CAN_GETTXBFTIMEOUT;

    uiTimeOut = 0;
    mcp25625_write_canMsg(txbuf_n);
    mcp25625_modifyRegister(txbuf_n-1 , MCP_TXB_TXREQ_M, MCP_TXB_TXREQ_M);
    
    temp = micros();
    do
    {       
        res1 = mcp25625_readRegister(txbuf_n-1);
        res1 = res1 & 0x08;
        uiTimeOut = micros() - temp;
    } while (res1 && (uiTimeOut < TIMEOUTVALUE));   
    
    if(uiTimeOut >= TIMEOUTVALUE)
        return CAN_SENDMSGTIMEOUT;
    
    return CAN_OK;
}

/*********************************************************************************************************
** Function name:           sendMsgBuf
** Descriptions:            Send message to transmitt buffer
*********************************************************************************************************/
INT8U MCP_CAN::sendMsgBuf(INT32U id, INT8U ext, INT8U len, INT8U *buf)
{
    INT8U res;
	
    setMsg(id, 0, ext, len, buf);
    res = sendMsg();
    
    return res;
}

/*********************************************************************************************************
** Function name:           sendMsgBuf
** Descriptions:            Send message to transmitt buffer
*********************************************************************************************************/
INT8U MCP_CAN::sendMsgBuf(INT32U id, INT8U len, INT8U *buf)
{
    INT8U ext = 0, rtr = 0;
    INT8U res;
    
    if((id & 0x80000000) == 0x80000000)
        ext = 1;
 
    if((id & 0x40000000) == 0x40000000)
        rtr = 1;
        
    setMsg(id, rtr, ext, len, buf);
    res = sendMsg();
    
    return res;
}

/*********************************************************************************************************
** Function name:           readMsg
** Descriptions:            Read message
*********************************************************************************************************/
INT8U MCP_CAN::readMsg()
{
    INT8U stat, res;

    stat = mcp25625_readStatus();

    if (stat & MCP_STAT_RX0IF)
    {
        mcp25625_read_canMsg(MCP_RXBUF_0);
        mcp25625_modifyRegister(MCP_CANINTF, MCP_RX0IF, 0);
        res = CAN_OK;
    }
    else if (stat & MCP_STAT_RX1IF)
    {
        mcp25625_read_canMsg(MCP_RXBUF_1);
        mcp25625_modifyRegister(MCP_CANINTF, MCP_RX1IF, 0);
        res = CAN_OK;
    }
    else 
        res = CAN_NOMSG;
    
    return res;
}

/*********************************************************************************************************
** Function name:           readMsgBuf
** Descriptions:            Public function, Reads message from receive buffer.
*********************************************************************************************************/
INT8U MCP_CAN::readMsgBuf(INT32U *id, INT8U *ext, INT8U *len, INT8U buf[])
{
    if(readMsg() == CAN_NOMSG)
	return CAN_NOMSG;
	
    *id  = m_nID;
    *len = m_nDlc;
    *ext = m_nExtFlg;
    for(int i = 0; i<m_nDlc; i++)
        buf[i] = m_nDta[i];

    return CAN_OK;
}

/*********************************************************************************************************
** Function name:           readMsgBuf
** Descriptions:            Public function, Reads message from receive buffer.
*********************************************************************************************************/
INT8U MCP_CAN::readMsgBuf(INT32U *id, INT8U *len, INT8U buf[])
{
    if(readMsg() == CAN_NOMSG)
	return CAN_NOMSG;

    if (m_nExtFlg)
        m_nID |= 0x80000000;

    if (m_nRtr)
        m_nID |= 0x40000000;
	
    *id  = m_nID;
    *len = m_nDlc;
    
    for(int i = 0; i<m_nDlc; i++)
        buf[i] = m_nDta[i];

    return CAN_OK;
}

/*********************************************************************************************************
** Function name:           checkReceive
** Descriptions:            Public function, Checks for received data.  (Used if not using the interrupt output)
*********************************************************************************************************/
INT8U MCP_CAN::checkReceive(void)
{
    INT8U res;
    res = mcp25625_readStatus();
    if (res & MCP_STAT_RXIF_MASK)
        return CAN_MSGAVAIL;
    else 
        return CAN_NOMSG;
}

/*********************************************************************************************************
** Function name:           checkError
** Descriptions:            Public function, Returns error register data.
*********************************************************************************************************/
INT8U MCP_CAN::checkError(void)
{
    INT8U eflg = mcp25625_readRegister(MCP_EFLG);

    if (eflg & MCP_EFLG_ERRORMASK)
        return CAN_CTRLERROR;
    else
        return CAN_OK;
}

/*********************************************************************************************************
** Function name:           getError
** Descriptions:            Returns error register value.
*********************************************************************************************************/
INT8U MCP_CAN::getError(void)
{
    return mcp25625_readRegister(MCP_EFLG);
}

/*********************************************************************************************************
** Function name:           mcp25625_errorCountRX
** Descriptions:            Returns REC register value
*********************************************************************************************************/
INT8U MCP_CAN::errorCountRX(void)
{
    return mcp25625_readRegister(MCP_REC);
}

/*********************************************************************************************************
** Function name:           mcp25625_errorCountTX
** Descriptions:            Returns TEC register value
*********************************************************************************************************/
INT8U MCP_CAN::errorCountTX(void)
{
    return mcp25625_readRegister(MCP_TEC);
}

/*********************************************************************************************************
** Function name:           mcp25625_enOneShotTX
** Descriptions:            Enables one shot transmission mode
*********************************************************************************************************/
INT8U MCP_CAN::enOneShotTX(void)
{
    mcp25625_modifyRegister(MCP_CANCTRL, MODE_ONESHOT, MODE_ONESHOT);
    if((mcp25625_readRegister(MCP_CANCTRL) & MODE_ONESHOT) != MODE_ONESHOT)
	    return CAN_FAIL;
    else
	    return CAN_OK;
}

/*********************************************************************************************************
** Function name:           mcp25625_disOneShotTX
** Descriptions:            Disables one shot transmission mode
*********************************************************************************************************/
INT8U MCP_CAN::disOneShotTX(void)
{
    mcp25625_modifyRegister(MCP_CANCTRL, MODE_ONESHOT, 0);
    if((mcp25625_readRegister(MCP_CANCTRL) & MODE_ONESHOT) != 0)
        return CAN_FAIL;
    else
        return CAN_OK;
}

/*********************************************************************************************************
** Function name:           mcp25625_abortTX
** Descriptions:            Aborts any queued transmissions
*********************************************************************************************************/
INT8U MCP_CAN::abortTX(void)
{
    mcp25625_modifyRegister(MCP_CANCTRL, ABORT_TX, ABORT_TX);
    if((mcp25625_readRegister(MCP_CANCTRL) & ABORT_TX) != ABORT_TX)
	    return CAN_FAIL;
    else
	    return CAN_OK;
}

/*********************************************************************************************************
** Function name:           setGPO
** Descriptions:            Public function, Sets GPO
*********************************************************************************************************/
INT8U MCP_CAN::setGPO(INT8U data)
{
    mcp25625_modifyRegister(MCP_BFPCTRL, MCP_BxBFS_MASK, (data<<4));
    return 0;
}

/*********************************************************************************************************
** Function name:           getGPI
** Descriptions:            Public function, Gets GPI
*********************************************************************************************************/
INT8U MCP_CAN::getGPI(void)
{
    INT8U res;
    res = mcp25625_readRegister(MCP_TXRTSCTRL) & MCP_BxRTS_MASK;
    return (res >> 3);
}

/*********************************************************************************************************
  END FILE
*********************************************************************************************************/

