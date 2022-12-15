#include <Arduino.h>
#include <HX711.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#define stp 23
#define dir 22
#define MS1 13
#define MS2 14
#define MS3 15
#define EN 39

HX711 lc;
float val = 0;

volatile byte rpmcount = 0;

unsigned int engineSpeed = 0;                                           // RPM
unsigned long lastTime = 0;                                             // ms
const int triggersPerRot = 4;                                           // -
int HFRejectionRPM = 4500;                                              // RPM
unsigned long DeltaTime = 60000000 / (HFRejectionRPM * triggersPerRot); // ms
unsigned long SpeedHistory[triggersPerRot * 2] = {0};
bool updateEngineSpeed = 0;
int steps = 200;       // Generic stepper motor
int speed = steps * 1; // add conversion "something to give me speed"

IntervalTimer ContorllerTimer;

void engineHallISR()
{
    unsigned int currTime = micros();
    if (currTime - lastTime > DeltaTime)
    {
        rpmcount++;
        lastTime = currTime;
        SpeedHistory[rpmcount] = engineSpeed;
    }
    if (rpmcount > 7)
    {
        rpmcount = 0;
    }
    updateEngineSpeed = 1;
}

void ControllerISR()
{
}

void setup()
{
    lc.begin(12, 11);
    lc.set_offset(0);
    Serial.begin(115200);
    pinMode(0, INPUT);
    attachInterrupt(digitalPinToInterrupt(0), engineHallISR, RISING);
    ContorllerTimer.begin(ControllerISR, 5000); // blinkLED to run every 0.005 seconds
}

void loop()
{
    updateEngine();
    updateLC();
    updateStepper();
    updateData();
}

void updateEngine()
{
    if (updateEngineSpeed)
    {
        engineSpeed = 120000000 / (SpeedHistory[rpmcount] - SpeedHistory[(rpmcount + 1) % (2 * triggersPerRot + 1)]);
    }
    else if (micros() - lastTime > 500000)
    {
        engineSpeed = 0;
    }
}

void updateLC()
{
    if (lc.is_ready())
    {
        val = float(lc.read()) / 9852 - 3.5;
    }
}

unsigned long lastWriteTime = 0;

void updateData()
{
    unsigned int currTime = micros();
    if (currTime - lastWriteTime > 10000)
    {
        lastWriteTime = currTime;
        Serial.print(millis());
        Serial.print(",");
        Serial.print(val);
        Serial.print(",");
        Serial.println(engineSpeed);
    }
}

enum StepperState
{
    INIT,
    ZERO_RETURN,
    REST,
    CONTROL
};
StepperState stepperState = INIT;

int curPos = 0; // Steps

void updateStepper()
{

    switch (stepperState)
    {
    case INIT:

    case ZERO_RETURN:

    case REST:

    case CONTROL:
    }
}

void step(int speed) // Input Speed in RPM
{
    digitalWrite(dir, LOW);
    unsigned long currTime;
    unsigned long prevTime;

    currTime = micros();
    if (currTime - prevTime >= speed)
    {
        digitalWrite(stp, HIGH); // Trigger one step forward
        prevTime = currTime;
    };

    currTime = micros();
    if (currTime - prevTime >= speed)
    {
        digitalWrite(stp, LOW); // Pull step pin low so it can be triggered again
        prevTime = currTime;
    };

    resetBEDPins();
}

void resetBEDPins()
{
    digitalWrite(stp, LOW);
    digitalWrite(dir, LOW);
    digitalWrite(MS1, LOW);
    digitalWrite(MS2, LOW);
    digitalWrite(EN, HIGH);
}