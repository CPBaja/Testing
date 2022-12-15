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
#define LS 00 // Pin for limit switch

#define LC1 12 // Pin for limit switch
#define LC2 11 // Pin for limit switch

#define HS 0 // Pin for hall effect sensor

HX711 lc;
float val = 0;

volatile byte rpmcount = 0;

unsigned long lastStepTime = 0;
bool STP = 0;

unsigned int engineSpeed = 0;                                           // RPM
unsigned long lastEngineTime = 0;                                       // us
const int triggersPerRot = 4;                                           // -
int HFRejectionRPM = 4500;                                              // RPM
unsigned long DeltaTime = 60000000 / (HFRejectionRPM * triggersPerRot); // us
unsigned long SpeedHistory[triggersPerRot * 2] = {0};
bool updateEngineSpeed = 0;
int steps = 200;             // Generic stepper motor
int stepangle = 360 / steps; // step angle

int minEngineSpeed = 2400;                                                // RPM
int maxEngineSpeed = 3600;                                                // RPM
int enginePeriod = 4 * 60 * 1000 * 1000;                                  // min*60*1000*1000 = us
int engineSlope = (enginePeriod / 2) / (maxEngineSpeed - minEngineSpeed); // delta us / engine speed
int currEngineTarget = maxEngineSpeed;                                    // RPM
bool speedUpEngine = 0;                                                   // 1 = increase engine speed, 0 = slow down engine speed

bool engineOn = 0;
int engineKP = 1;
int engineKI = 0.1;
int engineIntegral = 0;

int valveTargetPos = 0; // Target valve pos

int valveMinPos = 0;       // Fully open valve
int valveMaxPos = 1000;    // Fully closed valve
int minPosSteps = 1000;    // Number of steps at open valve
int maxPosSteps = 2000;    // Number of steps at closed valve
int absMaxPosSteps = 2500; // Checked for saftey when attempting to acutate stepper, won't step if curStep >=  absMaxPosSteps

int targetSteps = 0; // target postion in steps, ie scaled valveTargetPos
int curSteps = 0;    // Count the number of steps (current position)

bool stepperOn = 0; // Turn on and off stepper controller
int stepperKP = 1;
int stepperKI = 0.1;
int stepperIntegral = 0;
int stepperSpeed = 0;      // how fast to move the stepper
int maxStepperSpeed = 100; // RPM max stepper speed

int restPostion = 100; // Move away from the zero postion at rest by this much

enum StepperState
{
    INIT,
    ZERO_RETURN,
    REST,
    CONTROL
};
StepperState stepperState = INIT;
IntervalTimer ContorllerTimer;

void engineHallISR();
void ControllerISR();
void updateEngine();
void updateLC();
void updateStepper();
void updateData();
void step(int speed);
void setupBEDPins();

void setup()
{
    Serial.begin(115200);
    while (!Serial)
    {
    }

    delay(1000); // Pause for 1000 ms

    lc.begin(LC1, LC2); // Setup loadcell
    lc.set_offset(0);

    pinMode(HS, INPUT); // Setup hall effect sensor

    setupBEDPins(); // setup stepper driver

    pinMode(LS, INPUT_PULLUP); // setup limit switch

    attachInterrupt(digitalPinToInterrupt(HS), engineHallISR, RISING); // Attach engine speed interupt to the hall effect sensor
    ContorllerTimer.begin(ControllerISR, 5000);                        // Run control loop every 5 ms
}

void loop()
{
    updateEngine();
    updateLC();
    updateStepper();
    updateData();
}

void engineHallISR()
{
    unsigned int currTime = micros();
    if (currTime - lastEngineTime > DeltaTime)
    {
        rpmcount++;
        lastEngineTime = currTime;
        SpeedHistory[rpmcount] = engineSpeed;
    }
    if (rpmcount > 7)
    {
        rpmcount = 0;
    }
    updateEngineSpeed = 1;
}
/*
int minEngineSpeed = 2400;             // RPM
int maxEngineSpeed = 3600;             // RPM
int period = 4 * 60 * 1000 * 1000;     // MIN*60*1000*1000
int currEngineTarget = maxEngineSpeed; // RPM
bool speedUpEngine = 0;                // 1 = increase engine speed, 0 = slow down engine speed

bool engineOn = 0;
int engineKP = 1;
int engineKI = 0.1;
int engineIntegral = 0;

int valveTargetPos = 0; // Target valve pos

int valveMinPos = 0;       // Fully open valve
int valveMaxPos = 1000;    // Fully closed valve
int minPosSteps = 1000;    // Number of steps at open valve
int maxPosSteps = 2000;    // Number of steps at closed valve
int absMaxPosSteps = 2500; // Checked for saftey when attempting to acutate stepper, won't step if curStep >=  absMaxPosSteps

int targetSteps = 0; // target postion in steps, ie scaled valveTargetPos
int curSteps = 0;    // Count the number of steps (current position)

bool stepperOn = 0; // Turn on and off stepper controller
int stepperKP = 1;
int stepperKI = 0.1;
int stepperIntegral = 0;
int stepperSpeed = 0;      // how fast to move the stepper
int maxStepperSpeed = 100; // RPM max stepper speed
*/
void ControllerISR()
{
    if (engineOn)
    {
    }
    if (stepperOn)
    {
    }
}

void updateEngine()
{
    if (updateEngineSpeed)
    {
        engineSpeed = 120000000 / (SpeedHistory[rpmcount] - SpeedHistory[(rpmcount + 1) % (2 * triggersPerRot + 1)]);
    }
    if (micros() - lastEngineTime > 500000)
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

void updateStepper()
{
    switch (stepperState)
    {
    case INIT:
        stepperState = ZERO_RETURN;
    case ZERO_RETURN:
        if (!digitalRead(LS)) // Trigger when the limit switch drives the pin to ground
        {
            cli();
            step(0); // stop the stepper motor
            stepperSpeed = 0;
            engineOn = false;
            targetSteps = restPostion;
            stepperOn = true;
            stepperState = REST;
            sei();
        }
        else
        {
            step(-50);
        }

    case REST:
        step(stepperSpeed);
        if (engineSpeed > 3500 && abs(curSteps - restPostion) < 0.05 * restPostion)
        {
            currEngineTarget = maxEngineSpeed;
            engineOn = true;
            stepperState = CONTROL;
        }
    case CONTROL:
        step(stepperSpeed);
        if (engineSpeed < 1500)
        {
            cli();
            stepperSpeed = 0;
            engineOn = false;
            targetSteps = restPostion;
            stepperOn = true;
            stepperState = REST;
            sei();
        }
    }
}

void step(int speed) // Input Speed in RPM, saturate to max speed (delay of 1 ms)
{
    if (speed != 0)
    {
        if (speed > 0)
        {
            digitalWrite(dir, LOW);
        }
        else if (speed < 0)
        {
            digitalWrite(dir, HIGH);
            speed = -speed;
        }
        if (speed > maxStepperSpeed)
        {
            speed = maxStepperSpeed;
        }
        unsigned long stepPeriod = 1 / ((speed * 0.10472) / (0.006056 * 1000000)); // convert RPM to micros/step
        unsigned long currTime = micros();

        if (currTime - lastStepTime > stepPeriod)
        {
            STP = HIGH;
            digitalWrite(STP, HIGH);
            lastStepTime = currTime;
        }
        else if (STP && currTime - lastStepTime > 20)
        {
            STP = LOW;
            digitalWrite(STP, LOW);
        }
    }
}

void setupBEDPins()
{
    pinMode(stp, OUTPUT);
    pinMode(dir, OUTPUT);
    pinMode(MS1, OUTPUT);
    pinMode(MS2, OUTPUT);
    pinMode(MS3, OUTPUT);
    pinMode(EN, OUTPUT);

    digitalWrite(stp, LOW);
    digitalWrite(dir, LOW);
    digitalWrite(MS1, LOW);
    digitalWrite(MS2, LOW);
    digitalWrite(EN, HIGH);
}