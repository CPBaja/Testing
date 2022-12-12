#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>

void readMessages(void);

MCP2515 mcp(10);
IntervalTimer timer;

struct can_frame msg;

volatile uint32_t counter;
volatile uint16_t my_data = 1024;
volatile uint16_t read_data;

unsigned long t1 = 0;
unsigned long t2 = 0;
unsigned long time;

void setup() {
  Serial.begin(115200);

  mcp.reset();
  mcp.setBitrate(CAN_1000KBPS);
  mcp.setNormalMode();

  counter = 0;

  timer.begin(readMessages, 1000000);
}

void loop() {
  if(mcp.readMessage(&msg) == MCP2515::ERROR_OK) {
    counter++;
    read_data = ((msg.data[0] << 8) | msg.data[1]);
  }

  t1 = millis();

  time = t1 - t2;

  if(time > 10) {
    t2 = t1;
    Serial.println(counter);
    counter = 0;
  }
  
}

void readMessages() {
  Serial.printf("CAN messages sent: %n\n", counter);
  counter = 0;
}