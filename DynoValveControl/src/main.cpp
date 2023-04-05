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
#define LS 32 // Pin for limit switch

#define LC1 12 // Pin for limit switch
#define LC2 11 // Pin for limit switch

#define HS 0 // Pin for hall effect sensor

HX711 lc;
float val = 0;

int rpmcount = 0;

unsigned long lastStepTime = 0;
bool STP = 0;

unsigned int engineSpeed = 0;                                           // RPM
unsigned long lastEngineTime = 0;                                       // us
const int triggersPerRot = 4;                                           // -
int HFRejectionRPM = 4500;                                              // RPM
unsigned long DeltaTime = 60000000 / (HFRejectionRPM * triggersPerRot); // us
unsigned long SpeedHistory[triggersPerRot * 2 + 1] = {0};
bool updateEngineSpeed = 0;
int steps = 200;             // Generic stepper motor
int stepangle = 360 / steps; // step angle

int minEngineSpeed = 2400;                                                          // RPM
int maxEngineSpeed = 3600;                                                          // RPM
unsigned long enginePeriod = 0.5 * 60 * 1000 * 1000;                                // min*60*1000*1000 = us
unsigned long engineSlope = (enginePeriod / 2) / (maxEngineSpeed - minEngineSpeed); // delta us / engine speed
int engineTarget = maxEngineSpeed;                                                  // RPM
bool speedUpEngine = 0;                                                             // 1 = increase engine speed, 0 = slow down engine speed
unsigned long lastMaxMinEngineTime = 0;                                             // last time the target engine speed was at a min or max value

bool engineOn = 0;
float engineKP = 0.5;
float engineKI = 0.01;
long engineIntegral = 0;

int valveTargetPos = 0; // Target valve pos

// 17HS15-1684S-PG5
// 5.18 : 1 gearbox
// 200 steps per motor rotation
// 1036 steps per output rotation
// ~200 RPM max
// ~8.5 rotations full open -> full close
// zero 1.5 rotations from open
// ~7 rotations zero -> full close
// 0.5 rotation from zero = rest position, 500 steps
// 1 rotations from zero = min position, 1000 steps
// 6 rotations from zero = max position, 6000 steps
// 0.5 from close = safety ~6.5*1036 ~ 6700 rotations absolute max

int valveMinPos = 0;       // Fully open valve
int valveMaxPos = 1000;    // Fully closed valve
int minPosSteps = 3000;    // Number of steps at open valve
int maxPosSteps = 7300;    // Number of steps at closed valve
int absMaxPosSteps = 7300; // Checked for saftey when attempting to acutate stepper, won't step if curStep >=  absMaxPosSteps
int restPostion = 1000;    // Move away from the zero postion at rest by this much

int targetSteps = 0; // target postion in steps, ie scaled valveTargetPos
int curSteps = 0;    // Count the number of steps (current position)

bool stepperOn = 0; // Turn on and off stepper controller
float stepperKP = 0.25;
float stepperKI = 0;
int stepperIntegral = 0;
int stepperSpeed = 0;     // how fast to move the stepper
int maxStepperSpeed = 10; // RPM max stepper speed

unsigned long lastDataWriteTime = 0;

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

    lc.begin(LC1, LC2); // Setup loadcell
    lc.set_offset(0);

    pinMode(HS, INPUT); // Setup hall effect sensor

    setupBEDPins(); // setup stepper driver

    pinMode(LS, INPUT_PULLUP); // setup limit switch

    delay(1000); // Pause for 1000 ms

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
    unsigned long currTime = micros();
    if (currTime - lastEngineTime > DeltaTime)
    {
        rpmcount++;
        if (rpmcount > 2 * triggersPerRot)
        {
            rpmcount = 0;
        }
        lastEngineTime = currTime;
        SpeedHistory[rpmcount] = currTime;
        updateEngineSpeed = 1;
        // Serial.println("LF Accept");
    }
    else
    {
        // Serial.println("HF Reject");
    }
}

void ControllerISR()
{
    unsigned long currTime = micros();
    // if (speedUpEngine)
    // {
    //     engineTarget = (currTime - lastMaxMinEngineTime) / engineSlope + minEngineSpeed;
    // }
    // else
    // {
    //     engineTarget = maxEngineSpeed - (currTime - lastMaxMinEngineTime) / engineSlope;
    // }

    // if (engineTarget > maxEngineSpeed || engineTarget < minEngineSpeed)
    // {
    //     speedUpEngine = !speedUpEngine;
    //     lastMaxMinEngineTime = micros();
    // }
    engineTarget = 3000;

    if (engineOn)
    {
        int engineError = engineSpeed - engineTarget;
        Serial.println(engineError);

        if (valveTargetPos < valveMaxPos && valveTargetPos > valveMinPos)
        {
            engineIntegral += engineError;
        }
        valveTargetPos = engineKP * engineError + engineKI * engineIntegral;
        if (valveTargetPos > valveMaxPos)
        {
            valveTargetPos = valveMaxPos;
        }
        else if (valveTargetPos < valveMinPos)
        {
            valveTargetPos = valveMinPos;
        }
        targetSteps = minPosSteps + (maxPosSteps - minPosSteps) * (valveTargetPos - valveMinPos) / (valveMaxPos - valveMinPos);
    }
    if (stepperOn)
    {
        int stepperError = targetSteps - curSteps;
        if (abs(stepperSpeed) < maxStepperSpeed)
        {
            stepperIntegral += stepperError;
        }
        stepperSpeed = stepperKP * stepperError + stepperKI * stepperIntegral;
    }
}

void updateEngine()
{
    cli();
    if (updateEngineSpeed)
    {
        engineSpeed = 120000000 / (SpeedHistory[rpmcount] - SpeedHistory[(rpmcount + 1) % (2 * triggersPerRot + 1)]);
        updateEngineSpeed = false;
    }
    if (micros() - lastEngineTime > 500000)
    {
        engineSpeed = 0;
    }
    sei();
}

void updateLC()
{
    if (lc.is_ready())
    {
        val = float(lc.read()) / 9852 - 3.5;
    }
}

void updateData()
{
    unsigned int currTime = micros();
    if (false && currTime - lastDataWriteTime > 50000)
    {
        lastDataWriteTime = currTime;
        Serial.print(curSteps);
        Serial.print(",");
        Serial.print(targetSteps);
        Serial.print(",");
        Serial.print(valveTargetPos);
        Serial.print(",");
        Serial.print(engineSpeed);
        Serial.print(",");
        Serial.print(engineIntegral);
        Serial.print(",");
        Serial.print(engineSpeed - engineTarget);
        Serial.print(",");
        Serial.print(engineTarget);
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
        break;
    case ZERO_RETURN:
        if (!digitalRead(LS)) // Trigger when the limit switch drives the pin to ground
        {
            Serial.println("Triggered!");
            step(0); // stop the stepper motor
            stepperSpeed = 0;
            engineOn = false;
            targetSteps = restPostion;
            stepperOn = true;
            stepperState = REST;
            curSteps = 0;
        }
        else
        {
            step(-10);
        }
        break;

    case REST:
        step(stepperSpeed);
        if (engineSpeed > 2000)
        {
            engineTarget = maxEngineSpeed;
            lastMaxMinEngineTime = micros();
            engineIntegral = 0;
            engineOn = true;
            stepperState = CONTROL;
        }
        break;
    case CONTROL:
        step(stepperSpeed);
        if (engineSpeed < 1200)
        {
            stepperSpeed = 0;
            engineOn = false;
            targetSteps = restPostion;
            stepperOn = true;
            stepperState = REST;
        }
        break;
    }
}

void step(int speed) // Input Speed in RPM, saturate to max speed (delay of 1 ms)
{
    if (speed != 0 && !(curSteps > absMaxPosSteps && speed > 0) && !(!digitalRead(LS) && speed < 0))
    {
        bool posSpeed = 1;
        if (speed > 0)
        {
            digitalWrite(dir, LOW);
        }
        else if (speed < 0)
        {
            digitalWrite(dir, HIGH);
            speed = -speed;
            posSpeed = 0;
        }
        if (speed > maxStepperSpeed)
        {
            speed = maxStepperSpeed;
        }
        unsigned long stepPeriod = 57915 / speed; // 1 / ((speed * 0.10472) / (0.006056 * 1000000)); // convert RPM to micros/step

        cli();
        unsigned long currTime = micros();
        if (currTime - lastStepTime > stepPeriod)
        {
            STP = HIGH;
            digitalWrite(stp, HIGH);
            lastStepTime = currTime;
            if (posSpeed)
            {
                curSteps += 1;
            }
            else
            {
                curSteps -= 1;
            }
            // Serial.print("Step On: ");
            // Serial.println(currTime);
        }
        else if (STP && currTime - lastStepTime > 5)
        {
            STP = LOW;
            digitalWrite(stp, LOW);
            // Serial.print("Step Off");
            // Serial.println(currTime);
        }
        sei();
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
    digitalWrite(MS3, LOW);
    digitalWrite(EN, LOW);
}