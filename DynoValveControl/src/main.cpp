#include <Arduino.h>
#include <HX711.h>
#include <avr/io.h>
#include <avr/interrupt.h>

HX711 lc;
float val = 0;

volatile byte rpmcount=0;

unsigned int rpm=0;
unsigned long timeold=0;

void myInterrupt(){
    rpmcount++;
    if(rpmcount >=8){
        unsigned int curtime = millis();
        rpm = ((rpmcount/8)*2*60000)/(curtime-timeold);
        timeold = curtime;
        rpmcount = 0;
    }
}

void setup() {
    lc.begin(12, 11);
    lc.set_offset(0);
    Serial.begin(115200);
    pinMode(0, INPUT);
    attachInterrupt(digitalPinToInterrupt(0), myInterrupt, RISING);
}

void loop() {
        val = float(lc.read()) / 9852-3.5;
        Serial.print(millis());Serial.print(",");Serial.print(val);Serial.print(","); Serial.println(rpm);
}