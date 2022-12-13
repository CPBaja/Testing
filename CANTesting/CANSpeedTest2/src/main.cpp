#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>

void sentMessages(void);

MCP2515 mcp(10);
IntervalTimer timer;

unsigned long t1;
unsigned long t2;

unsigned long diff;

struct can_frame msg;

volatile uint32_t counter;
uint16_t my_data = 1024;

void setup() {
  Serial.begin(115200);

  mcp.reset();
  mcp.setBitrate(CAN_1000KBPS);
  mcp.setNormalMode();

  msg.can_dlc = 2;
  msg.can_id = 0;
  msg.data[0] = (my_data | 0xFF00) >> 8;
  msg.data[1] = (my_data | 0x00FF);

  counter = 0;

  Serial.println("Here1");

  t1 = millis();
  t2 = t1;
}

void loop() {
  mcp.sendMessage(&msg);
  counter++;
  diff = t2 - t1;
  t2 = millis();
  if(diff > 10) {
    //Serial.println(counter);
    //Serial.printf("CAN messages sent: %n\n", counter);
    
    Serial.print(msg.data[0]);
    Serial.println(msg.data[1]);
    counter = 0;
    t1 = t2;
  }
  msg.data[1] = msg.data[1]+1;
}

void sentMessages() {
  
  
}

// MCP2515 mcp(10);

// struct can_frame msg;

// unsigned long t1;
// unsigned long t2;

// unsigned long diff;

// void setup() {
//   Serial.begin(115200);

//   mcp.reset();
//   mcp.setBitrate(CAN_1000KBPS);
//   mcp.setNormalMode();

//   msg.can_dlc = 2;
//   msg.can_id = 0;
//   msg.data[0] = 255;
//   msg.data[1] = 255;
//   t1 = millis();
//   t2 = t1;
// }

// void loop() {
  
//   mcp.sendMessage(&msg);
//   diff = t2 - t1;
//   if(diff > 1000) {
//     Serial.println("Serial");
//     t2 = t1;
//   }
//   t1 = millis();
// }