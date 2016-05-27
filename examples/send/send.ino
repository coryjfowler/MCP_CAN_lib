// demo: CAN-BUS Shield, send data
#include <mcp_can.h>
#include <SPI.h>

MCP_CAN CAN0(10);                                      // Set CS to pin 10

void setup()
{
  Serial.begin(115200);
  // init can bus, baudrate: 500k
  if (CAN0.begin(CAN_500KBPS) == CAN_OK)
    Serial.println("CAN init ok");
  else
    Serial.println("CAN init failed");
}

unsigned char stmp[8] = {0, 1, 2, 3, 4, 5, 6, 7};
void loop()
{
  // send data:  id = 0x00, standard frame, data len = 8, stmp: data buf
  CAN0.sendMsgBuf(0x00, 0, 8, stmp);  
  delay(100);                       // send data per 100ms
}

/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
