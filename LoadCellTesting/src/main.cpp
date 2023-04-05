#include <Arduino.h>
#include <HX711.h>

HX711 lcP;
HX711 lcS;
int valP = 0;
int valS = 0;

void setup() {
    lcP.begin(2, 3);
    lcP.set_offset(0);
    lcS.begin(5, 4);
    lcS.set_offset(0);
    Serial.begin(115200);
    Serial.println("Starting...");

}

void loop() {
    valP = lcP.read() / 10056;
    valS = lcS.read() / 10056;
    Serial.printf("Primary LC: %i, Secondary LC: %i\n", valP, valS);
}