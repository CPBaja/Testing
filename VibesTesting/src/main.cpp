#include <SD.h>
#include <SPI.h>
#include <Wire.h>
#include <Bounce2.h>
#include <Adafruit_LSM6DSO32.h>

#define BUTTON_PIN 20

#define ADDRESS1 0x68
//#define ADDRESS2 0x69

String directory = "teensy";
String subsystem = "Ben";

Bounce button = Bounce();
File out;

Adafruit_LSM6DSO32 lsm6d = Adafruit_LSM6DSO32();

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

sensors_event_t accel;
sensors_event_t gyro;
sensors_event_t temp;

int MPU6050_read(int address, int start, uint8_t *buffer, int size);
int MPU6050_write_reg(int address, int reg, uint8_t data);
int MPU6050_write(int address, int start, const uint8_t *pData, int size);
void run();
void createFile();
void writeToFile(sensors_event_t accel, sensors_event_t gyro);

void setup()
{
    button.attach(BUTTON_PIN, INPUT);
    button.interval(5);

    pinMode(LED_BUILTIN, OUTPUT);

    // // Open serial communications and wait for port to open:
    // Serial.begin(9600);
    // while (!Serial)
    // {
    //     ; // Wait for serial port to connect. Needed for native USB port only.
    // }

    // Setup Accelerometer
    //  Wire.begin();
    // Clear the 'sleep' bit to start the sensor.
    //  MPU6050_write_reg(ADDRESS1, 0x6B, 0);
    // MPU6050_write_reg(ADDRESS2, 0x6B, 0);
    //  Set to 16G range
    //  MPU6050_write_reg(ADDRESS1, 0x1C, bit(3) | bit(4));
    // MPU6050_write_reg(ADDRESS2, 0x1C, bit(3)|bit(4));

    //Attempt to connect gyro. If gyro is not found, freeze the program
    if (!lsm6d.begin_I2C()) {
        while (1) {
            delay(10);
    }
  }

    lsm6d.setAccelRange(LSM6DSO32_ACCEL_RANGE_32_G);
    lsm6d.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
    lsm6d.setAccelDataRate(LSM6DS_RATE_6_66K_HZ);
    lsm6d.setGyroDataRate(LSM6DS_RATE_6_66K_HZ);

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
    static bool isRunning = false;
    button.update();
    if (button.changed() && !button.read())
    {
        isRunning = !isRunning;
        if (isRunning)
        {
            createFile();
            digitalWrite(LED_BUILTIN, HIGH);
        }
        else
        {
            out.flush();
            out.close();
            digitalWrite(LED_BUILTIN, LOW);
        }
    }
    if (isRunning)
    {
        run();
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
    // Serial.print("Created file ");
    // Serial.print(fileNumber);
    // Serial.print(".csv in directory.");
    // Serial.println();

    // Setup file
    out.println("Hello " + subsystem + "!");
    out.println(F("The accelerometer is configured for a 16G range: 2048 counts = 1G."));
    out.println(F("The gyroscope is configured for 250 deg/sec: 131 counts = 1 deg/sec."));
    out.println(F("Sample time records time between the current row and the previous row."));
    out.println(F(" -Rahul"));
    out.println();
    out.println();
    //  if (SD.exists("readme.txt")){
    //    Serial.println("README file exists!");
    //    File readMe = SD.open("readme.txt", FILE_READ);
    //    while (readMe.peek() != -1){
    //      out.print(char(readMe.read()));
    //    }
    //    out.println();
    //    out.println();
    //  }
    out.print("Accel1_X,Accel1_Y,Accel1_Z,Gyro1_X,Gyro1_Y,Gyro1_Z,Sample_Time (µs)\n");
    //  out.print("Accel2_X,Accel2_Y,Accel2_Z,Gyro2_X,Gyro2_Y,Gyro2_Z,,");
    //  out.println("Sample_Time (µs)");
}

void writeToFile(sensors_event_t accel, sensors_event_t gyro) {
    unsigned long time = micros();
    out.printf("%i,%i,%i,%f,%f,%f,%lu\n", accel.acceleration.x, accel.acceleration.y, accel.acceleration.z, gyro.gyro.x, gyro.gyro.y, gyro.gyro.z, time);
    //out.flush();
}

void run()
{

//     accel_t_gyro_union accel_t_gyro1;
//     // accel_t_gyro_union accel_t_gyro2;

//     // Read raw values
//     MPU6050_read(ADDRESS1, 0x3B, (uint8_t *)&accel_t_gyro1, sizeof(accel_t_gyro1));
//     // MPU6050_read(ADDRESS2, 0x3B, (uint8_t *) &accel_t_gyro2, sizeof(accel_t_gyro2));

//     // Swap values
//     uint8_t swap;
// #define SWAP(x, y) \
//     swap = x;      \
//     x = y;         \
//     y = swap

//     SWAP(accel_t_gyro1.reg.x_accel_h, accel_t_gyro1.reg.x_accel_l);
//     SWAP(accel_t_gyro1.reg.y_accel_h, accel_t_gyro1.reg.y_accel_l);
//     SWAP(accel_t_gyro1.reg.z_accel_h, accel_t_gyro1.reg.z_accel_l);
//     SWAP(accel_t_gyro1.reg.x_gyro_h, accel_t_gyro1.reg.x_gyro_l);
//     SWAP(accel_t_gyro1.reg.y_gyro_h, accel_t_gyro1.reg.y_gyro_l);
//     SWAP(accel_t_gyro1.reg.z_gyro_h, accel_t_gyro1.reg.z_gyro_l);

    // SWAP (accel_t_gyro2.reg.x_accel_h, accel_t_gyro2.reg.x_accel_l);
    // SWAP (accel_t_gyro2.reg.y_accel_h, accel_t_gyro2.reg.y_accel_l);
    // SWAP (accel_t_gyro2.reg.z_accel_h, accel_t_gyro2.reg.z_accel_l);
    // SWAP (accel_t_gyro2.reg.x_gyro_h, accel_t_gyro2.reg.x_gyro_l);
    // SWAP (accel_t_gyro2.reg.y_gyro_h, accel_t_gyro2.reg.y_gyro_l);
    // SWAP (accel_t_gyro2.reg.z_gyro_h, accel_t_gyro2.reg.z_gyro_l);

    // Print raw values

    // // Print the raw acceleration values (1)
    // out.print(accel_t_gyro1.value.x_accel, DEC);
    // out.print(F(","));
    // out.print(accel_t_gyro1.value.y_accel, DEC);
    // out.print(F(","));
    // out.print(accel_t_gyro1.value.z_accel, DEC);
    // out.print(F(","));

    // // Print the raw gyro values (1)
    // out.print(accel_t_gyro1.value.x_gyro, DEC);
    // out.print(F(","));
    // out.print(accel_t_gyro1.value.y_gyro, DEC);
    // out.print(F(","));
    // out.print(accel_t_gyro1.value.z_gyro, DEC);
    // out.print(F(",,"));

    // // // Print the raw acceleration values (2)
    // // out.print(accel_t_gyro2.value.x_accel, DEC);
    // // out.print(F(","));
    // // out.print(accel_t_gyro2.value.y_accel, DEC);
    // // out.print(F(","));
    // // out.print(accel_t_gyro2.value.z_accel, DEC);
    // // out.print(F(","));

    // // // Print the raw gyro values (2)
    // // out.print(accel_t_gyro2.value.x_gyro, DEC);
    // // out.print(F(","));
    // // out.print(accel_t_gyro2.value.y_gyro, DEC);
    // // out.print(F(","));
    // // out.print(accel_t_gyro2.value.z_gyro, DEC);
    // // out.print(F(",,"));

    // // Print time
    // out.print(micros());
    // out.println(F(""));

    // // // Cleanup
    // // out.flush();
    lsm6d.getEvent(&accel, &gyro, &temp);

    writeToFile(accel, gyro);
}

// --------------------------------------------------------
// MPU6050_read
//
// This is a common function to read multiple bytes
// from an I2C device.
//
// It uses the boolean parameter for Wire.endTransMission()
// to be able to hold or release the I2C-bus.
// This is implemented in Arduino 1.0.1.
//
// Only this function is used to read.
// There is no function for a single byte.
//
int MPU6050_read(int address, int start, uint8_t *buffer, int size)
{
    int i, n;

    Wire.beginTransmission(address);
    n = Wire.write(start);
    if (n != 1)
        return (-10);

    n = Wire.endTransmission(false); // hold the I2C-bus
    if (n != 0)
        return (n);

    // Third parameter is true: relase I2C-bus after data is read.
    Wire.requestFrom(address, size, 1);
    i = 0;
    while (Wire.available() && i < size)
    {
        buffer[i++] = Wire.read();
    }
    if (i != size)
        return (-11);

    return (0); // return : no error
}

// --------------------------------------------------------
// MPU6050_write
//
// This is a common function to write multiple bytes to an I2C device.
//
// If only a single register is written,
// use the function MPU_6050_write_reg().
//
// Parameters:
//   start : Start address, use a define for the register
//   pData : A pointer to the data to write.
//   size  : The number of bytes to write.
//
// If only a single register is written, a pointer
// to the data has to be used, and the size is
// a single byte:
//   int data = 0;        // the data to write
//   MPU6050_write (address, 0x6B, &c, 1);
//
int MPU6050_write(int address, int start, const uint8_t *pData, int size)
{
    int n, error;

    Wire.beginTransmission(address);
    n = Wire.write(start); // write the start address
    if (n != 1)
        return (-20);

    n = Wire.write(pData, size); // write data bytes
    if (n != size)
        return (-21);

    error = Wire.endTransmission(true); // release the I2C-bus
    if (error != 0)
        return (error);

    return (0); // return : no error
}

// --------------------------------------------------------
// MPU6050_write_reg
//
// An extra function to write a single register.
// It is just a wrapper around the MPU_6050_write()
// function, and it is only a convenient function
// to make it easier to write a single register.
//
int MPU6050_write_reg(int address, int reg, uint8_t data)
{
    int error;

    error = MPU6050_write(address, reg, &data, 1);

    return (error);
}