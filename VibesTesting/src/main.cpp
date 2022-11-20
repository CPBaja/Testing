#include <SD.h> 
#include <SPI.h>
#include <Wire.h>
#include <Bounce2.h> // https://github.com/thomasfredericks/Bounce2
#include <Adafruit_LSM6DSO32.h>

#define BUTTON_PIN 24
#define LED_PIN 13

#define ADDRESS1 0x68
//#define ADDRESS2 0x69

String directory = "teensy";
String subsystem = "Ben";

//Bounce2::Button button = Bounce2::Button(); // create a button object
Button button = Button();

File out;

Adafruit_LSM6DSO32 lsm6d1 = Adafruit_LSM6DSO32();
Adafruit_LSM6DSO32 lsm6d2 = Adafruit_LSM6DSO32();
Adafruit_LSM6DSO32 lsm6d3 = Adafruit_LSM6DSO32();

typedef union accel_t_gyro_union
{
    struct
    {
        uint8_t x_accel_h;
        uint8_t x_accel_l;
        uint8_t y_accel_h;
        uint8_t y_accel_l;
        uint8_t z_accel_h;
        uint8_t z_accel_l;
        uint8_t t_h;
        uint8_t t_l;
        uint8_t x_gyro_h;
        uint8_t x_gyro_l;
        uint8_t y_gyro_h;
        uint8_t y_gyro_l;
        uint8_t z_gyro_h;
        uint8_t z_gyro_l;
    } reg;
    struct
    {
        int16_t x_accel;
        int16_t y_accel;
        int16_t z_accel;
        int16_t temperature;
        int16_t x_gyro;
        int16_t y_gyro;
        int16_t z_gyro;
    } value;
} accel_t_gyro_union;

sensors_event_t accel1;
sensors_event_t gyro1;
sensors_event_t temp1;
sensors_event_t accel2;
sensors_event_t gyro2;
sensors_event_t temp2;
sensors_event_t accel3;
sensors_event_t gyro3;
sensors_event_t temp3;

int MPU6050_read(int address, int start, uint8_t *buffer, int size);
int MPU6050_write_reg(int address, int reg, uint8_t data);
int MPU6050_write(int address, int start, const uint8_t *pData, int size);
int ledState;

static bool isRunning = false;

void run();
void createFile();
void writeToFile(sensors_event_t accel1, sensors_event_t accel2, sensors_event_t accel3);


void setup()
{
    ledState = 0;

    button.attach (BUTTON_PIN, INPUT_PULLUP); // attach debouncer to pin 24 with INPUT_PULLUP mode
    button.interval(50); // debounce intervall of 50 ms
    button.setPressedState(LOW);
    
    pinMode(LED_PIN, OUTPUT);    // sets the pin 13 as output
    digitalWrite(LED_PIN, ledState); // turn off LED

    // // Open serial communications and wait for port to open:
    Serial.begin(9600);

    //Attempt to connect gyro. If gyro is not found, freeze the program
    if (!lsm6d1.begin_I2C() || !lsm6d2.begin_I2C() || !lsm6d3.begin_I2C()) {
        while (1) {
            delay(10);
    }
  }
    //sensor classes
    lsm6d1.setAccelRange(LSM6DSO32_ACCEL_RANGE_32_G);
    lsm6d1.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
    lsm6d1.setAccelDataRate(LSM6DS_RATE_6_66K_HZ);
    lsm6d1.setGyroDataRate(LSM6DS_RATE_6_66K_HZ);

    lsm6d2.setAccelRange(LSM6DSO32_ACCEL_RANGE_32_G);
    lsm6d2.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
    lsm6d2.setAccelDataRate(LSM6DS_RATE_6_66K_HZ);
    lsm6d2.setGyroDataRate(LSM6DS_RATE_6_66K_HZ);

    lsm6d3.setAccelRange(LSM6DSO32_ACCEL_RANGE_32_G);
    lsm6d3.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
    lsm6d3.setAccelDataRate(LSM6DS_RATE_6_66K_HZ);
    lsm6d3.setGyroDataRate(LSM6DS_RATE_6_66K_HZ);

    // Setup SD card
    // Serial.print("Initializing SD card...");
    if (!SD.begin(BUILTIN_SDCARD))
    {
        // Serial.println("initialization failed!");
        return;
    }
    // Serial.println("initialization done.");

    // Create arduino directory
    const char *dir = directory.c_str();
    if (!SD.exists(dir))
    {
        SD.mkdir(dir);
        // Serial.println("Created directory '" + directory + "'.");
    }
    else
    {
        // Serial.println("Directory '" + directory + "' already exists.");
    }
}

void loop() 
{
    button.update(); // update bounce instance

    if (button.pressed()) 
    {
        ledState = !ledState;
        digitalWrite(LED_PIN, ledState);

        if(!isRunning)
        {
            isRunning = true;
            Serial.println("button pressed & is now running");
            
            createFile();
            //Serial.println("file created");

        }
        else
        {
            isRunning = false;

            out.flush();
            out.close();
            Serial.println("button pressed & closing file");
        }
    }

    if(isRunning)
    {
        run();
        //Serial.println(accel1);
        //Serial.println(accel2);
        //Serial.println(accel3);
        //Serial.println(writeToFile);  
    }
}
 

void createFile()
{
    // Create new file
    int fileNumber = 0;
    const char *filename;
    do
    {
        filename = (directory + "/" + ++fileNumber + ".csv").c_str();
    } while (SD.exists(filename));

    out = SD.open(filename, FILE_WRITE_BEGIN);
    Serial.print("Created file "); //
    Serial.print(fileNumber);
    Serial.print(".csv in directory.");
    Serial.println();   //

    // Setup file
    out.println("Hello " + subsystem + "!");
    out.println(F("The accelerometer is configured for a 16G range: 2048 counts = 1G."));
    out.println(F("The gyroscope is configured for 250 deg/sec: 131 counts = 1 deg/sec."));
    out.println(F("Sample time records time between the current row and the previous row."));
    out.println(F(" -Rahul"));
    out.println();
    out.println();
    Serial.println("file created");

    out.print("Accel1_X,Accel1_Y,Accel1_Z,Accel2_X,Accel2_Y,Accel2_Z,Accel3_X,Accel3_Y,Accel3_Z,Sample_Time (µs)\n");

}

void writeToFile(sensors_event_t accel1, sensors_event_t accel2, sensors_event_t accel3) {
    unsigned long time = micros();
    out.printf("%i,%i,%i,%i,%i,%i,%i,%i,%i,%lu\n", accel1.acceleration.x, accel1.acceleration.y, accel1.acceleration.z, accel2.acceleration.x, accel2.acceleration.y, accel2.acceleration.z, accel3.acceleration.x, accel3.acceleration.y, accel3.acceleration.z, time);
}

void run()
{
    lsm6d1.getEvent(&accel1, &gyro1, &temp1);
    lsm6d2.getEvent(&accel2, &gyro2, &temp2);
    lsm6d3.getEvent(&accel3, &gyro3, &temp3);

    writeToFile(accel1, accel2, accel3);
}