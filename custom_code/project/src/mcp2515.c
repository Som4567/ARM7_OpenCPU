/*
 * mcp2515.c
 *
 */

#include "mcp2515.h"
#include "ql_type.h"
#include "ql_trace.h"
#include "ql_stdlib.h"
#include "ql_gpio.h"
#include "ql_spi.h"
#include "ql_uart.h"
#include "ql_error.h"
#include "ql_system.h"

#define N_TXBUFFERSS		3
#define N_RXBUFFERSS		2

#define USR_SPI_CHANNEL		     (1)
uint8_t spi_usr_type = 0;

uint8_t SPICS;
static const int N_TXBUFFERS = 3;
//static const int N_RXBUFFERS = 2;

struct TXBn_REGS {
    REGISTER CTRL;
    REGISTER SIDH;
    REGISTER DATA;
  };

struct RXBn_REGS {
    REGISTER CTRL;
    REGISTER SIDH;
    REGISTER DATA;
    CANINTF  CANINTF_RXnIF;
  };


const struct TXBn_REGS  TXB[N_TXBUFFERSS] = {
    {MCP_TXB0CTRL, MCP_TXB0SIDH, MCP_TXB0DATA},
    {MCP_TXB1CTRL, MCP_TXB1SIDH, MCP_TXB1DATA},
    {MCP_TXB2CTRL, MCP_TXB2SIDH, MCP_TXB2DATA}
};


uint8_t INSTRUCTION_WRITE       = 0x02;
uint8_t	INSTRUCTION_READ        = 0x03;
uint8_t	INSTRUCTION_BITMOD      = 0x05;
uint8_t INSTRUCTION_LOAD_TX0    = 0x40;
uint8_t INSTRUCTION_LOAD_TX1    = 0x42;
uint8_t INSTRUCTION_LOAD_TX2    = 0x44;
uint8_t INSTRUCTION_RTS_TX0     = 0x81;
uint8_t INSTRUCTION_RTS_TX1     = 0x82;
uint8_t INSTRUCTION_RTS_TX2     = 0x84;
uint8_t INSTRUCTION_RTS_ALL     = 0x87;
uint8_t INSTRUCTION_READ_RX0    = 0x90;
uint8_t INSTRUCTION_READ_RX1    = 0x94;
uint8_t INSTRUCTION_READ_STATUS = 0xA0;
uint8_t INSTRUCTION_RX_STATUS   = 0xB0;
uint8_t INSTRUCTION_RESET       = 0xC0;

//const struct TXBn_REGS  TXB[3][3];
//    TXB[0][0] = MCP_TXB0CTRL;
//    TXB[0][1] = MCP_TXB0SIDH;
//    TXB[0][2] = MCP_TXB0DATA;
//    TXB[1][0] = MCP_TXB1CTRL;
//    TXB[1][1] = MCP_TXB1SIDH;
//    TXB[1][2] = MCP_TXB1DATA;
//    TXB[2][0] = MCP_TXB2CTRL;
//    TXB[2][1] = MCP_TXB2SIDH;
//    TXB[2][2] = MCP_TXB2DATA;
//    		, MCP_TXB0SIDH, MCP_TXB0DATA},
//    {MCP_TXB1CTRL, MCP_TXB1SIDH, MCP_TXB1DATA},
//    {MCP_TXB2CTRL, MCP_TXB2SIDH, MCP_TXB2DATA}
//};


const struct  RXBn_REGS  RXB[N_RXBUFFERSS] = {
    {MCP_RXB0CTRL, MCP_RXB0SIDH, MCP_RXB0DATA, CANINTF_RX0IF},
    {MCP_RXB1CTRL, MCP_RXB1SIDH, MCP_RXB1DATA, CANINTF_RX1IF}
};
void spi_flash_cs(bool CS)
{
	if (!spi_usr_type)
	{
		if (CS)
			Ql_GPIO_SetLevel(PINNAME_PCM_CLK,PINLEVEL_HIGH);
		else
			Ql_GPIO_SetLevel(PINNAME_PCM_CLK,PINLEVEL_LOW);
	}
}

int init(const uint8_t _CS)
{
//    SPI.begin();
	s32 ret=-1;
    ret = Ql_SPI_Init(USR_SPI_CHANNEL,PINNAME_PCM_IN,PINNAME_PCM_SYNC,PINNAME_PCM_OUT,PINNAME_PCM_CLK,spi_usr_type);
    Ql_Debug_Trace("HI in INIT \r\n");

    if(ret <0)
        {
    	Ql_Debug_Trace("\r\n<-- Failed!! Ql_SPI_Init fail , ret =%d-->\r\n",ret);
		//Ql_Debug_Trace("\r\n<-- Failed!! Ql_SPI_Init fail , ret =%d-->\r\n",ret);
        }
        else
        {
        	Ql_Debug_Trace("\r\n<-- Ql_SPI_Init ret =%d -->\r\n",ret);
        	ret =-1;
        	ret = Ql_SPI_Config(USR_SPI_CHANNEL,1,0,0,SPI_CLOCK); //config sclk about 10MHz;
        	        if(ret <0)
        	        {
        	           Ql_Debug_Trace("\r\n<--Failed!! Ql_SPI_Config fail  ret=%d -->\r\n",ret);
        	        }
        	        else
        	        {
        	            Ql_Debug_Trace("\r\n<-- Ql_SPI_Config  =%d -->\r\n",ret);
        	        }
        }


    //int32_t ret=-1;


    //init cs pin
    	if (!spi_usr_type)
    	{
    		Ql_GPIO_Init(PINNAME_PCM_CLK,PINDIRECTION_OUT,PINLEVEL_HIGH,PINPULLSEL_PULLUP);   //CS high
    	}
//    SPICS = _CS;
////    pinMode(SPICS, OUTPUT);
//    //Ql_GPIO_Init(SPICS, PINDIRECTION_OUT, 1, PINPULLSEL_DISABLE);  // for CS pin
//    if(!spi_usr_type)
//    {
//    	endSPI();
//    }
//    ret = Ql_SPI_Uninit(USR_SPI_CHANNEL);    // i think it's not required here

    //spi_flash_cs()
     endSPI();
    return(ret);
}

void startSPI(void)  // configuration of SPI
{
//    SPI.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE0));

//    digitalWrite(SPICS, LOW);
//    if(!spi_usr_type)
//    {
//    	Ql_GPIO_SetLevel(SPICS, PINLEVEL_LOW);  // for CS pin
//    }

    spi_flash_cs(0);
}

void  endSPI(void) {
//    digitalWrite(SPICS, HIGH);
//    if(!spi_usr_type)
//    {
//    	Ql_GPIO_SetLevel(SPICS, PINLEVEL_HIGH);  // for CS pin
//    }
//    SPI.endTransaction();
	Ql_Debug_Trace("\r\n<-- In end SPI -->\r\n");
	spi_flash_cs(1);

}


 ERROR  reset(void)
{
    startSPI();
    int32_t ret=-1;
//    SPI.transfer(INSTRUCTION_RESET);
    ret = Ql_SPI_Write(USR_SPI_CHANNEL,&INSTRUCTION_RESET,1);
    if(ret == 1)
       {
    	Ql_Debug_Trace("\r\n<-- in reset sending pass  Instruction= %d-->\r\n", INSTRUCTION_RESET);
       }
       else
       {
    	   Ql_Debug_Trace("\r\n<-- in reset sending fail  Instruction= %d-->\r\n", INSTRUCTION_RESET);
       }
    endSPI();

    Ql_Sleep(10);

    uint8_t zeros[14];
    Ql_memset(zeros, 0, sizeof(zeros));
    setRegisters(MCP_TXB0CTRL, zeros, 14);
    setRegisters(MCP_TXB1CTRL, zeros, 14);
    setRegisters(MCP_TXB2CTRL, zeros, 14);

    setRegister(MCP_RXB0CTRL, 0);
    setRegister(MCP_RXB1CTRL, 0);

    setRegister(MCP_CANINTE, CANINTF_RX0IF | CANINTF_RX1IF | CANINTF_ERRIF | CANINTF_MERRF);

    // receives all valid messages using either Standard or Extended Identifiers that
    // meet filter criteria. RXF0 is applied for RXB0, RXF1 is applied for RXB1
    modifyRegister(MCP_RXB0CTRL,
                   RXBnCTRL_RXM_MASK | RXB0CTRL_BUKT | RXB0CTRL_FILHIT_MASK,
                   RXBnCTRL_RXM_STDEXT | RXB0CTRL_BUKT | RXB0CTRL_FILHIT);
    modifyRegister(MCP_RXB1CTRL,
                   RXBnCTRL_RXM_MASK | RXB1CTRL_FILHIT_MASK,
                   RXBnCTRL_RXM_STDEXT | RXB1CTRL_FILHIT);

    // clear filters and masks
    // do not filter any standard frames for RXF0 used by RXB0
    // do not filter any extended frames for RXF1 used by RXB1
    RXF filters[] = {RXF0, RXF1, RXF2, RXF3, RXF4, RXF5};
    for (int i=0; i<6; i++) {
        bool ext = (i == 1);
        ERROR result = setFilter(filters[i], ext, 0);
        if (result != ERROR_OK) {
        	Ql_Debug_Trace("\r\n<-- setFilter fail-->\r\n");
            return result;
        }
    }

    MASK masks[] = {MASK0, MASK1};
    for (int i=0; i<2; i++) {
        ERROR result = setFilterMask(masks[i], true, 0);
        if (result != ERROR_OK) {
        	Ql_Debug_Trace("\r\n<-- setFilterMask fail-->\r\n");
            return result;
        }
    }
    Ql_Debug_Trace("\r\n<-- successfully completred reset-->\r\n");
    return ERROR_OK;
}

uint8_t  readRegister( REGISTER reg)
{
    startSPI();

//    SPI.transfer(INSTRUCTION_READ);
//    SPI.transfer(reg);
//    uint8_t ret = SPI.transfer(0x00);
    uint8_t ret;
    uint8_t ret1=0;
//    Ql_SPI_WriteRead(USR_SPI_CHANNEL,INSTRUCTION_READ,1,&ret,1);
    //int32_t ret=-1;
    ret = Ql_SPI_Write(USR_SPI_CHANNEL,&INSTRUCTION_READ,1);
    if(ret == 1)
           {
        	Ql_Debug_Trace("\r\n<-- in readRegister sending pass  INSTRUCTION_READ= %x-->\r\n", INSTRUCTION_READ);
           }
           else
           {
        	   Ql_Debug_Trace("\r\n<-- in readRegister sending fail  INSTRUCTION_READ= %x-->\r\n", INSTRUCTION_READ);
           }
    ret=-1;


    ret = Ql_SPI_Write(USR_SPI_CHANNEL,&reg,1);
        if(ret == 1)
               {
            	Ql_Debug_Trace("\r\n<-- in readRegister sending pass  INSTRUCTION_READ= %x-->\r\n", INSTRUCTION_READ);
               }
               else
               {
            	   Ql_Debug_Trace("\r\n<-- in readRegister sending fail  INSTRUCTION_READ= %x-->\r\n", INSTRUCTION_READ);
               }
        ret=-1;

   // ret = Ql_SPI_Write(USR_SPI_CHANNEL,&reg,1);
    ret = Ql_SPI_WriteRead(USR_SPI_CHANNEL,&reg,0, &ret1,1);
    if(ret == 1)
           {
        	Ql_Debug_Trace("\r\n<-- in readRegister sending pass  reg= %x-->\r\n", reg);
           }
           else
           {
        	   Ql_Debug_Trace("\r\n<-- in readRegister sending fail  reg= %x-->\r\n", reg);
           }
   // ret=0;
   // Ql_SPI_Read(USR_SPI_CHANNEL,&ret,1);
        	Ql_Debug_Trace("\r\n<-- in readRegister sending pass  ret1= %d-->\r\n", ret1);
              endSPI();

    return ret1;
}

void  readRegisters( REGISTER reg, uint8_t values[], const uint8_t n)
{
    startSPI();
//    SPI.transfer(INSTRUCTION_READ);
//    SPI.transfer(reg);
    // mcp2515 has auto-increment of address-pointer
    uint8_t ret=-1;
    ret= Ql_SPI_Write(USR_SPI_CHANNEL,&INSTRUCTION_READ,1);
    if(ret == 1)
               {
            	Ql_Debug_Trace("\r\n<-- in readRegisters sending pass  INSTRUCTION_READ= %x-->\r\n", INSTRUCTION_READ);
               }
               else
               {
            	   Ql_Debug_Trace("\r\n<-- in readRegisters sending fail  INSTRUCTION_READ= %x-->\r\n", INSTRUCTION_READ);
               }
    ret=-1;
    ret = Ql_SPI_Write(USR_SPI_CHANNEL,&reg,1);
    if(ret == 1)
               {
            	Ql_Debug_Trace("\r\n<-- in readRegisters sending pass  reg= %x-->\r\n", reg);
               }
               else
               {
            	   Ql_Debug_Trace("\r\n<-- in readRegisters sending fail  reg= %x-->\r\n", reg);
               }
    for (uint8_t i=0; i<n; i++) {
    	ret=0;
    	u8 wr_buff=0;
        //ret= Ql_SPI_Read(USR_SPI_CHANNEL,&values[i],1);
        ret = Ql_SPI_WriteRead(USR_SPI_CHANNEL,wr_buff,0,&values[i],1);

        if(ret == 1)
                       {
        	 Ql_Debug_Trace("\r\n<-- in readRegister reading pass  Value= %x, i= %d-->\r\n", values[i],i);
                       }
                       else
                       {
                    	   Ql_Debug_Trace("\r\n<-- in readRegister reading fail  Value= %x, i= %d-->\r\n", values[i],i);
                       }

    	//        values[i] = SPI.transfer(0x00);
    }
    endSPI();
}

void  setRegister( REGISTER reg,  uint8_t value)
{
    startSPI();
    uint8_t ret=0;
//    SPI.transfer(INSTRUCTION_WRITE);
//    SPI.transfer(reg);
//    SPI.transfer(value);

   ret=  Ql_SPI_Write(USR_SPI_CHANNEL,&INSTRUCTION_WRITE,1);
    if(ret == 1)
                   {
                	Ql_Debug_Trace("\r\n<-- in setRegister sending pass  INSTRUCTION_WRITE= %d-->\r\n", INSTRUCTION_WRITE);
                   }
                   else
                   {
                	   Ql_Debug_Trace("\r\n<-- in setRegister sending fail  INSTRUCTION_WRITE= %d-->\r\n", INSTRUCTION_WRITE);
                   }
    ret=0;

    ret = Ql_SPI_Write(USR_SPI_CHANNEL,&reg,1);
    if(ret == 1)
                       {
                    	Ql_Debug_Trace("\r\n<-- in setRegister sending pass  reg= %d-->\r\n", reg);
                       }
                       else
                       {
                    	   Ql_Debug_Trace("\r\n<-- in setRegister sending fail  reg= %d-->\r\n", reg);
                       }
    ret=0;
    ret = Ql_SPI_Write(USR_SPI_CHANNEL,&value,1);
    if(ret == 1)
                           {
                        	Ql_Debug_Trace("\r\n<-- in setRegister sending pass  value= %d-->\r\n", value);
                           }
                           else
                           {
                        	   Ql_Debug_Trace("\r\n<-- in setRegister sending fail  value= %d-->\r\n", value);
                           }


    endSPI();
}

void  setRegisters(REGISTER reg, uint8_t values[], const uint8_t n)
{
    startSPI();
    uint8_t ret=0;
//    SPI.transfer(INSTRUCTION_WRITE);
//    SPI.transfer(reg);
    ret = Ql_SPI_Write(USR_SPI_CHANNEL,&INSTRUCTION_WRITE,1);
    if(ret == 1)
     {
           Ql_Debug_Trace("\r\n<-- in setRegisters sending pass  INSTRUCTION_WRITE= %d-->\r\n", INSTRUCTION_WRITE);
     }
      else
       {
            Ql_Debug_Trace("\r\n<-- in setRegisters sending fail  INSTRUCTION_WRITE= %d-->\r\n", INSTRUCTION_WRITE);
       }

    ret=0;
    ret = Ql_SPI_Write(USR_SPI_CHANNEL,&reg,1);
    if(ret == 1)
         {
               Ql_Debug_Trace("\r\n<-- in setRegisters sending pass  reg= %d-->\r\n", reg);
         }
          else
           {
                Ql_Debug_Trace("\r\n<-- in setRegisters sending fail  reg= %d-->\r\n", reg);
           }


    for (uint8_t i=0; i<n; i++) {
//        SPI.transfer(values[i]);



        ret=0;
        ret = Ql_SPI_Write(USR_SPI_CHANNEL,&values[i],1);
           if(ret == 1)
                {
                      Ql_Debug_Trace("\r\n<-- in setRegisters sending pass  &values[i]= %d-->\r\n", values[i]);
                }
                 else
                  {
                       Ql_Debug_Trace("\r\n<-- in setRegisters sending fail  &values[i]= %d-->\r\n", values[i]);
                  }

    }
    endSPI();
}

void  modifyRegister( REGISTER reg, uint8_t mask, const uint8_t data)
{
    startSPI();

    uint8_t ret=0;
    ret=0;
    ret = Ql_SPI_Write(USR_SPI_CHANNEL,&INSTRUCTION_BITMOD,1);
    if(ret == 1)
    {
         Ql_Debug_Trace("\r\n<-- in modifyRegister sending pass  INSTRUCTION_BITMOD= %d-->\r\n", INSTRUCTION_BITMOD);
    }
    else
    {
         Ql_Debug_Trace("\r\n<-- in modifyRegister sending fail  INSTRUCTION_BITMOD= %d-->\r\n", INSTRUCTION_BITMOD);
    }


    ret=0;
        ret = Ql_SPI_Write(USR_SPI_CHANNEL,&reg,1);
        if(ret == 1)
        {
             Ql_Debug_Trace("\r\n<-- in modifyRegister sending pass  reg= %d-->\r\n", reg);
        }
        else
        {
             Ql_Debug_Trace("\r\n<-- in modifyRegister sending fail  reg= %d-->\r\n", reg);
        }

    ret=0;
        ret = Ql_SPI_Write(USR_SPI_CHANNEL,&mask,1);
        if(ret == 1)
        {
             Ql_Debug_Trace("\r\n<-- in modifyRegister sending pass  mask= %d-->\r\n", mask);
        }
        else
        {
             Ql_Debug_Trace("\r\n<-- in modifyRegister sending fail  mask= %d-->\r\n", mask);
        }

    ret=0;
        ret = Ql_SPI_Write(USR_SPI_CHANNEL,&data,1);
        if(ret == 1)
        {
             Ql_Debug_Trace("\r\n<-- in modifyRegister sending pass  data= %d-->\r\n", data);
        }
        else
        {
             Ql_Debug_Trace("\r\n<-- in modifyRegister sending fail  data= %d-->\r\n", data);
        }
//    SPI.transfer(INSTRUCTION_BITMOD);
//    SPI.transfer(reg);
//    SPI.transfer(mask);
//    SPI.transfer(data);
    endSPI();
}

uint8_t  getStatus(void)
{
    startSPI();
//    SPI.transfer(INSTRUCTION_READ_STATUS);
//    uint8_t i = SPI.transfer(0x00);
    uint8_t i;
    uint8_t ret;


    ret = Ql_SPI_Write(USR_SPI_CHANNEL,&INSTRUCTION_READ_STATUS,1);
           if(ret == 1)
                  {
               	Ql_Debug_Trace("\r\n<-- in readRegister sending pass  INSTRUCTION_READ= %x-->\r\n", INSTRUCTION_READ);
                  }
                  else
                  {
               	   Ql_Debug_Trace("\r\n<-- in readRegister sending fail  INSTRUCTION_READ= %x-->\r\n", INSTRUCTION_READ);
                  }
           ret=-1;
           Ql_SPI_WriteRead(USR_SPI_CHANNEL,&INSTRUCTION_READ_STATUS,0,&i,1);

           Ql_Debug_Trace("\r\n<--getStatus i = %x-->\r\n", i);


    endSPI();

    return i;
}

 ERROR  setConfigMode(void)
{ Ql_Debug_Trace("\r\n<-- setConfigMode-->\r\n");
    return setMode(CANCTRL_REQOP_CONFIG);
}

 ERROR  setListenOnlyMode(void)

{ Ql_Debug_Trace("\r\n<-- setListenOnlyMode-->\r\n");

	 return setMode(CANCTRL_REQOP_LISTENONLY);
}

 ERROR  setSleepMode(void)
{
    return setMode(CANCTRL_REQOP_SLEEP);
}

 ERROR  setLoopbackMode(void)
{
    return setMode(CANCTRL_REQOP_LOOPBACK);
}

 ERROR  setNormalMode(void)
{
    return setMode(CANCTRL_REQOP_NORMAL);
}

 ERROR  setMode(const CANCTRL_REQOP_MODE mode)
{
    modifyRegister(MCP_CANCTRL, CANCTRL_REQOP, mode);

    unsigned long endTime = Ql_GetMsSincePwrOn() + 10;
    Ql_Debug_Trace("\r\n<-- IN SETmODE Ql_GetMsSincePwrOn()= %d ,endTime = %d-->\r\n", Ql_GetMsSincePwrOn(),endTime);
    bool modeMatch = false;
    Ql_Debug_Trace("\r\n<-- IN SETmODE mode = %x-->\r\n", mode);
    while (Ql_GetMsSincePwrOn() < endTime) {
        uint8_t newmode = readRegister(MCP_CANSTAT);
        Ql_Debug_Trace("\r\n<-- IN SETmODE newmode = %x-->\r\n", newmode);
        newmode &= CANSTAT_OPMOD;
        Ql_Debug_Trace("\r\n<-- IN SETmODE newmode &= CANSTAT_OPMOD;  newmode = %x-->\r\n", newmode);
        modeMatch = newmode == mode;


        if (modeMatch) {
            break;
        }
    }
    Ql_Debug_Trace("\r\n<-- IN SETmODE modeMatch = %d-->\r\n", modeMatch);
    return modeMatch ? ERROR_OK : ERROR_FAIL;

}

 ERROR  setBitrate(const CAN_SPEED canSpeed)
{
    return setBitrates(canSpeed, MCP_16MHZ);
}

 ERROR  setBitrates(const CAN_SPEED canSpeed, CAN_CLOCK canClock)
{
    ERROR error = setConfigMode();
    if (error != ERROR_OK) {
        return error;
    }

    uint8_t set, cfg1, cfg2, cfg3;
    set = 1;
    switch (canClock)
    {
        case (MCP_8MHZ):
        switch (canSpeed)
        {
            case (CAN_5KBPS):                                               //   5KBPS
            cfg1 = MCP_8MHz_5kBPS_CFG1;
            cfg2 = MCP_8MHz_5kBPS_CFG2;
            cfg3 = MCP_8MHz_5kBPS_CFG3;
            break;

            case (CAN_10KBPS):                                              //  10KBPS
            cfg1 = MCP_8MHz_10kBPS_CFG1;
            cfg2 = MCP_8MHz_10kBPS_CFG2;
            cfg3 = MCP_8MHz_10kBPS_CFG3;
            break;

            case (CAN_20KBPS):                                              //  20KBPS
            cfg1 = MCP_8MHz_20kBPS_CFG1;
            cfg2 = MCP_8MHz_20kBPS_CFG2;
            cfg3 = MCP_8MHz_20kBPS_CFG3;
            break;

            case (CAN_31K25BPS):                                            //  31.25KBPS
            cfg1 = MCP_8MHz_31k25BPS_CFG1;
            cfg2 = MCP_8MHz_31k25BPS_CFG2;
            cfg3 = MCP_8MHz_31k25BPS_CFG3;
            break;

            case (CAN_33KBPS):                                              //  33.333KBPS
            cfg1 = MCP_8MHz_33k3BPS_CFG1;
            cfg2 = MCP_8MHz_33k3BPS_CFG2;
            cfg3 = MCP_8MHz_33k3BPS_CFG3;
            break;

            case (CAN_40KBPS):                                              //  40Kbps
            cfg1 = MCP_8MHz_40kBPS_CFG1;
            cfg2 = MCP_8MHz_40kBPS_CFG2;
            cfg3 = MCP_8MHz_40kBPS_CFG3;
            break;

            case (CAN_50KBPS):                                              //  50Kbps
            cfg1 = MCP_8MHz_50kBPS_CFG1;
            cfg2 = MCP_8MHz_50kBPS_CFG2;
            cfg3 = MCP_8MHz_50kBPS_CFG3;
            break;

            case (CAN_80KBPS):                                              //  80Kbps
            cfg1 = MCP_8MHz_80kBPS_CFG1;
            cfg2 = MCP_8MHz_80kBPS_CFG2;
            cfg3 = MCP_8MHz_80kBPS_CFG3;
            break;

            case (CAN_100KBPS):                                             // 100Kbps
            cfg1 = MCP_8MHz_100kBPS_CFG1;
            cfg2 = MCP_8MHz_100kBPS_CFG2;
            cfg3 = MCP_8MHz_100kBPS_CFG3;
            break;

            case (CAN_125KBPS):                                             // 125Kbps
            cfg1 = MCP_8MHz_125kBPS_CFG1;
            cfg2 = MCP_8MHz_125kBPS_CFG2;
            cfg3 = MCP_8MHz_125kBPS_CFG3;
            break;

            case (CAN_200KBPS):                                             // 200Kbps
            cfg1 = MCP_8MHz_200kBPS_CFG1;
            cfg2 = MCP_8MHz_200kBPS_CFG2;
            cfg3 = MCP_8MHz_200kBPS_CFG3;
            break;

            case (CAN_250KBPS):                                             // 250Kbps
            cfg1 = MCP_8MHz_250kBPS_CFG1;
            cfg2 = MCP_8MHz_250kBPS_CFG2;
            cfg3 = MCP_8MHz_250kBPS_CFG3;
            break;

            case (CAN_500KBPS):                                             // 500Kbps
            cfg1 = MCP_8MHz_500kBPS_CFG1;
            cfg2 = MCP_8MHz_500kBPS_CFG2;
            cfg3 = MCP_8MHz_500kBPS_CFG3;
            break;

            case (CAN_1000KBPS):                                            //   1Mbps
            cfg1 = MCP_8MHz_1000kBPS_CFG1;
            cfg2 = MCP_8MHz_1000kBPS_CFG2;
            cfg3 = MCP_8MHz_1000kBPS_CFG3;
            break;

            default:
            set = 0;
            break;
        }
        break;

        case (MCP_16MHZ):
        switch (canSpeed)
        {
            case (CAN_5KBPS):                                               //   5Kbps
            cfg1 = MCP_16MHz_5kBPS_CFG1;
            cfg2 = MCP_16MHz_5kBPS_CFG2;
            cfg3 = MCP_16MHz_5kBPS_CFG3;
            break;

            case (CAN_10KBPS):                                              //  10Kbps
            cfg1 = MCP_16MHz_10kBPS_CFG1;
            cfg2 = MCP_16MHz_10kBPS_CFG2;
            cfg3 = MCP_16MHz_10kBPS_CFG3;
            break;

            case (CAN_20KBPS):                                              //  20Kbps
            cfg1 = MCP_16MHz_20kBPS_CFG1;
            cfg2 = MCP_16MHz_20kBPS_CFG2;
            cfg3 = MCP_16MHz_20kBPS_CFG3;
            break;

            case (CAN_33KBPS):                                              //  33.333Kbps
            cfg1 = MCP_16MHz_33k3BPS_CFG1;
            cfg2 = MCP_16MHz_33k3BPS_CFG2;
            cfg3 = MCP_16MHz_33k3BPS_CFG3;
            break;

            case (CAN_40KBPS):                                              //  40Kbps
            cfg1 = MCP_16MHz_40kBPS_CFG1;
            cfg2 = MCP_16MHz_40kBPS_CFG2;
            cfg3 = MCP_16MHz_40kBPS_CFG3;
            break;

            case (CAN_50KBPS):                                              //  50Kbps
            cfg1 = MCP_16MHz_50kBPS_CFG1;
            cfg2 = MCP_16MHz_50kBPS_CFG2;
            cfg3 = MCP_16MHz_50kBPS_CFG3;
            break;

            case (CAN_80KBPS):                                              //  80Kbps
            cfg1 = MCP_16MHz_80kBPS_CFG1;
            cfg2 = MCP_16MHz_80kBPS_CFG2;
            cfg3 = MCP_16MHz_80kBPS_CFG3;
            break;

            case (CAN_83K3BPS):                                             //  83.333Kbps
            cfg1 = MCP_16MHz_83k3BPS_CFG1;
            cfg2 = MCP_16MHz_83k3BPS_CFG2;
            cfg3 = MCP_16MHz_83k3BPS_CFG3;
            break;

            case (CAN_100KBPS):                                             // 100Kbps
            cfg1 = MCP_16MHz_100kBPS_CFG1;
            cfg2 = MCP_16MHz_100kBPS_CFG2;
            cfg3 = MCP_16MHz_100kBPS_CFG3;
            break;

            case (CAN_125KBPS):                                             // 125Kbps
            cfg1 = MCP_16MHz_125kBPS_CFG1;
            cfg2 = MCP_16MHz_125kBPS_CFG2;
            cfg3 = MCP_16MHz_125kBPS_CFG3;
            break;

            case (CAN_200KBPS):                                             // 200Kbps
            cfg1 = MCP_16MHz_200kBPS_CFG1;
            cfg2 = MCP_16MHz_200kBPS_CFG2;
            cfg3 = MCP_16MHz_200kBPS_CFG3;
            break;

            case (CAN_250KBPS):                                             // 250Kbps
            cfg1 = MCP_16MHz_250kBPS_CFG1;
            cfg2 = MCP_16MHz_250kBPS_CFG2;
            cfg3 = MCP_16MHz_250kBPS_CFG3;
            break;

            case (CAN_500KBPS):                                             // 500Kbps
            cfg1 = MCP_16MHz_500kBPS_CFG1;
            cfg2 = MCP_16MHz_500kBPS_CFG2;
            cfg3 = MCP_16MHz_500kBPS_CFG3;
            break;

            case (CAN_1000KBPS):                                            //   1Mbps
            cfg1 = MCP_16MHz_1000kBPS_CFG1;
            cfg2 = MCP_16MHz_1000kBPS_CFG2;
            cfg3 = MCP_16MHz_1000kBPS_CFG3;
            break;

            default:
            set = 0;
            break;
        }
        break;

        case (MCP_20MHZ):
        switch (canSpeed)
        {
            case (CAN_33KBPS):                                              //  33.333Kbps
            cfg1 = MCP_20MHz_33k3BPS_CFG1;
            cfg2 = MCP_20MHz_33k3BPS_CFG2;
            cfg3 = MCP_20MHz_33k3BPS_CFG3;
	    break;

            case (CAN_40KBPS):                                              //  40Kbps
            cfg1 = MCP_20MHz_40kBPS_CFG1;
            cfg2 = MCP_20MHz_40kBPS_CFG2;
            cfg3 = MCP_20MHz_40kBPS_CFG3;
            break;

            case (CAN_50KBPS):                                              //  50Kbps
            cfg1 = MCP_20MHz_50kBPS_CFG1;
            cfg2 = MCP_20MHz_50kBPS_CFG2;
            cfg3 = MCP_20MHz_50kBPS_CFG3;
            break;

            case (CAN_80KBPS):                                              //  80Kbps
            cfg1 = MCP_20MHz_80kBPS_CFG1;
            cfg2 = MCP_20MHz_80kBPS_CFG2;
            cfg3 = MCP_20MHz_80kBPS_CFG3;
            break;

            case (CAN_83K3BPS):                                             //  83.333Kbps
            cfg1 = MCP_20MHz_83k3BPS_CFG1;
            cfg2 = MCP_20MHz_83k3BPS_CFG2;
            cfg3 = MCP_20MHz_83k3BPS_CFG3;
	    break;

            case (CAN_100KBPS):                                             // 100Kbps
            cfg1 = MCP_20MHz_100kBPS_CFG1;
            cfg2 = MCP_20MHz_100kBPS_CFG2;
            cfg3 = MCP_20MHz_100kBPS_CFG3;
            break;

            case (CAN_125KBPS):                                             // 125Kbps
            cfg1 = MCP_20MHz_125kBPS_CFG1;
            cfg2 = MCP_20MHz_125kBPS_CFG2;
            cfg3 = MCP_20MHz_125kBPS_CFG3;
            break;

            case (CAN_200KBPS):                                             // 200Kbps
            cfg1 = MCP_20MHz_200kBPS_CFG1;
            cfg2 = MCP_20MHz_200kBPS_CFG2;
            cfg3 = MCP_20MHz_200kBPS_CFG3;
            break;

            case (CAN_250KBPS):                                             // 250Kbps
            cfg1 = MCP_20MHz_250kBPS_CFG1;
            cfg2 = MCP_20MHz_250kBPS_CFG2;
            cfg3 = MCP_20MHz_250kBPS_CFG3;
            break;

            case (CAN_500KBPS):                                             // 500Kbps
            cfg1 = MCP_20MHz_500kBPS_CFG1;
            cfg2 = MCP_20MHz_500kBPS_CFG2;
            cfg3 = MCP_20MHz_500kBPS_CFG3;
            break;

            case (CAN_1000KBPS):                                            //   1Mbps
            cfg1 = MCP_20MHz_1000kBPS_CFG1;
            cfg2 = MCP_20MHz_1000kBPS_CFG2;
            cfg3 = MCP_20MHz_1000kBPS_CFG3;
            break;

            default:
            set = 0;
            break;
        }
        break;

        default:
        set = 0;
        break;
    }

    if (set) {
        setRegister(MCP_CNF1, cfg1);
        setRegister(MCP_CNF2, cfg2);
        setRegister(MCP_CNF3, cfg3);
        return ERROR_OK;
    }
    else {
        return ERROR_FAIL;
    }
}

 ERROR  setClkOut(const CAN_CLKOUT divisor)
{
    if (divisor == CLKOUT_DISABLE) {
	/* Turn off CLKEN */
	modifyRegister(MCP_CANCTRL, CANCTRL_CLKEN, 0x00);

	/* Turn on CLKOUT for SOF */
	modifyRegister(MCP_CNF3, CNF3_SOF, CNF3_SOF);
        return ERROR_OK;
    }

    /* Set the prescaler (CLKPRE) */
    modifyRegister(MCP_CANCTRL, CANCTRL_CLKPRE, divisor);

    /* Turn on CLKEN */
    modifyRegister(MCP_CANCTRL, CANCTRL_CLKEN, CANCTRL_CLKEN);

    /* Turn off CLKOUT for SOF */
    modifyRegister(MCP_CNF3, CNF3_SOF, 0x00);
    return ERROR_OK;
}

void  prepareId(uint8_t *buffer, const bool ext, const uint32_t id)
{
    uint16_t canid = (uint16_t)(id & 0x0FFFF);

    if (ext) {
        buffer[MCP_EID0] = (uint8_t) (canid & 0xFF);
        buffer[MCP_EID8] = (uint8_t) (canid >> 8);
        canid = (uint16_t)(id >> 16);
        buffer[MCP_SIDL] = (uint8_t) (canid & 0x03);
        buffer[MCP_SIDL] += (uint8_t) ((canid & 0x1C) << 3);
        buffer[MCP_SIDL] |= TXB_EXIDE_MASK;
        buffer[MCP_SIDH] = (uint8_t) (canid >> 5);
    } else {
        buffer[MCP_SIDH] = (uint8_t) (canid >> 3);
        buffer[MCP_SIDL] = (uint8_t) ((canid & 0x07 ) << 5);
        buffer[MCP_EID0] = 0;
        buffer[MCP_EID8] = 0;
    }
}

 ERROR  setFilterMask(const MASK mask, const bool ext, const uint32_t ulData)
{
    ERROR res = setConfigMode();
    if (res != ERROR_OK) {
    	   Ql_Debug_Trace("\r\n<-- fail inside setFilterMask -->\r\n");
        return res;
    }

    uint8_t tbufdata[4];
    prepareId(tbufdata, ext, ulData);

    REGISTER reg;
    switch (mask) {
        case MASK0: reg = MCP_RXM0SIDH; break;
        case MASK1: reg = MCP_RXM1SIDH; break;
        default:
            return ERROR_FAIL;
    }

    setRegisters(reg, tbufdata, 4);
    Ql_Debug_Trace("\r\n<-- successfully completed setFilterMask -->\r\n");
    return ERROR_OK;
}

 ERROR  setFilter(const RXF num, const bool ext, const uint32_t ulData)
{
    ERROR res = setConfigMode();
    if (res != ERROR_OK) {
    	Ql_Debug_Trace("\r\n<-- fail inside setFilter due to setConfigMode  -->\r\n");
        return res;
    }

    REGISTER reg;

    switch (num) {
        case RXF0: reg = MCP_RXF0SIDH; break;
        case RXF1: reg = MCP_RXF1SIDH; break;
        case RXF2: reg = MCP_RXF2SIDH; break;
        case RXF3: reg = MCP_RXF3SIDH; break;
        case RXF4: reg = MCP_RXF4SIDH; break;
        case RXF5: reg = MCP_RXF5SIDH; break;
        default:
            return ERROR_FAIL;
    }

    uint8_t tbufdata[4];
    prepareId(tbufdata, ext, ulData);
    setRegisters(reg, tbufdata, 4);
    Ql_Debug_Trace("\r\n<-- successfully completed setFilter -->\r\n");
    return ERROR_OK;
}

 ERROR  sendMessages(const TXBn txbn, const struct can_frame *frame)
{
    if (frame->can_dlc > CAN_MAX_DLEN) {
        return ERROR_FAILTX;
    }

    const struct TXBn_REGS *txbuf = &TXB[txbn];

    uint8_t data[13];

    bool ext = (frame->can_id & CAN_EFF_FLAG);
    bool rtr = (frame->can_id & CAN_RTR_FLAG);
    uint32_t id = (frame->can_id & (ext ? CAN_EFF_MASK : CAN_SFF_MASK));

    prepareId(data, ext, id);

    data[MCP_DLC] = rtr ? (frame->can_dlc | RTR_MASK) : frame->can_dlc;

    Ql_memcpy(&data[MCP_DATA], frame->data, frame->can_dlc);

    setRegisters(txbuf->SIDH, data, 5 + frame->can_dlc);

    modifyRegister(txbuf->CTRL, TXB_TXREQ, TXB_TXREQ);

    uint8_t ctrl = readRegister(txbuf->CTRL);
    if ((ctrl & (TXB_ABTF | TXB_MLOA | TXB_TXERR)) != 0) {
        return ERROR_FAILTX;
    }
    return ERROR_OK;
}

 ERROR  sendMessage(const struct can_frame *frame)
{
//	    frame->can_id = id;
//	    frame->can_dlc = dlc;

    if ((frame->can_dlc) > CAN_MAX_DLEN) {
        return ERROR_FAILTX;
    }

//    TXBn txBuffers[N_TXBUFFERS] = {TXB0, TXB1, TXB2};
    TXBn txBuffers[3];
    txBuffers[0] = TXB0;
    txBuffers[1] = TXB1;
    txBuffers[2] = TXB2;
//    , TXB1, TXB2};

    for (int i=0; i<N_TXBUFFERS; i++) {
        const struct TXBn_REGS *txbuf = &TXB[txBuffers[i]];
        uint8_t ctrlval = readRegister(txbuf->CTRL);
        if ( (ctrlval & TXB_TXREQ) == 0 ) {
            return sendMessages(txBuffers[i], frame);
        }
    }

    return ERROR_ALLTXBUSY;
}

 ERROR  readMessages(const RXBn rxbn, struct can_frame *frame)
{
    const struct RXBn_REGS *rxb = &RXB[rxbn];

    uint8_t tbufdata[5]={0,0,0,0,0};


    readRegisters(rxb->SIDH, tbufdata, 5);

    uint32_t id = (tbufdata[MCP_SIDH]<<3) + (tbufdata[MCP_SIDL]>>5);

    if ( (tbufdata[MCP_SIDL] & TXB_EXIDE_MASK) ==  TXB_EXIDE_MASK ) {
        id = (id<<2) + (tbufdata[MCP_SIDL] & 0x03);
        id = (id<<8) + tbufdata[MCP_EID8];
        id = (id<<8) + tbufdata[MCP_EID0];
        id |= CAN_EFF_FLAG;
    }

    uint8_t dlc = (tbufdata[4] & DLC_MASK);
    if (dlc > CAN_MAX_DLEN) {
        return ERROR_FAIL;  // ERROR_FAIL;
    }

    uint8_t ctrl = readRegister(rxb->CTRL);
    if (ctrl & RXBnCTRL_RTR) {
        id |= CAN_RTR_FLAG;
    }

    frame->can_id = id;
    frame->can_dlc = dlc;

    readRegisters(rxb->DATA, frame->data, dlc);

    modifyRegister(MCP_CANINTF, rxb->CANINTF_RXnIF, 0);

    return ERROR_OK;
}

 ERROR  readMessage(struct can_frame *frame)
{
    ERROR rc;
    uint8_t stat = getStatus();

    if ( stat & STAT_RX0IF ) {
        rc = readMessages(RXB0, frame);
    } else if ( stat & STAT_RX1IF ) {
        rc = readMessages(RXB1, frame);
    } else {
        rc = ERROR_NOMSG;
    }

    return rc;
}

bool  checkReceive(void)
{
    uint8_t res = getStatus();
    if ( res & STAT_RXIF_MASK ) {
        return true;
    } else {
        return false;
    }
}

bool  checkError(void)
{
    uint8_t eflg = getErrorFlags();

    if ( eflg & EFLG_ERRORMASK ) {
        return true;
    } else {
        return false;
    }
}

uint8_t  getErrorFlags(void)
{
    return readRegister(MCP_EFLG);
}

void  clearRXnOVRFlags(void)
{
	modifyRegister(MCP_EFLG, EFLG_RX0OVR | EFLG_RX1OVR, 0);
}

uint8_t  getInterrupts(void)
{
    return readRegister(MCP_CANINTF);
}

void  clearInterrupts(void)
{
    setRegister(MCP_CANINTF, 0);
}

uint8_t  getInterruptMask(void)
{
    return readRegister(MCP_CANINTE);
}

void  clearTXInterrupts(void)
{
    modifyRegister(MCP_CANINTF, (CANINTF_TX0IF | CANINTF_TX1IF | CANINTF_TX2IF), 0);
}

void  clearRXnOVR(void)
{
	uint8_t eflg = getErrorFlags();
	if (eflg != 0) {
		clearRXnOVRFlags();
		clearInterrupts();
		//modifyRegister(MCP_CANINTF, CANINTF_ERRIF, 0);
	}

}

void  clearMERR(void)
{
	//modifyRegister(MCP_EFLG, EFLG_RX0OVR | EFLG_RX1OVR, 0);
	//clearInterrupts();
	modifyRegister(MCP_CANINTF, CANINTF_MERRF, 0);
}

void  clearERRIF(void)
{
    //modifyRegister(MCP_EFLG, EFLG_RX0OVR | EFLG_RX1OVR, 0);
    //clearInterrupts();
    modifyRegister(MCP_CANINTF, CANINTF_ERRIF, 0);
}

uint8_t  errorCountRX(void)
{
    return readRegister(MCP_REC);
}

uint8_t  errorCountTX(void)
{
    return readRegister(MCP_TEC);
}

