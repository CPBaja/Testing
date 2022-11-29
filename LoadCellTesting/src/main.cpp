#include <Arduino.h>
#include <HX711.h>

HX711 lc;
int val = 0;

void setup() {
    lc.begin(12, 11);
    lc.set_offset(0);
    Serial.begin(115200);
    Serial.println("Starting...");

}

void loop() {
    val = lc.read() / 10056;
    Serial.println(val);
}