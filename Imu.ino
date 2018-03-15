/*  LSM9DS1 --------- Arduino Mega 2560
          CS_AG ------------ 22
          CS_M ------------- 23
          SDO_AG ----------- 50
          SDO_M ------------ 50 (tied to SDO_AG)
          SCL -------------- 52
          SDA -------------- 51
          VDD -------------- 3.3V
          GND -------------- GND*/

#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>

LSM9DS1 imu;

#define LSM9DS1_M_CS  23 
#define LSM9DS1_AG_CS 22  

#define PRINT_CALCULATED
#define PRINT_SPEED 250 // 250 ms between prints
#define DECLINATION 3.26 // Declination (degrees) in Gothenburg, SE

struct Coor{
  double x;
  double y;
  double z;
};

Coor Acc;
Coor Gyro; 

void ImuSetup() 
{
  
  imu.settings.device.commInterface = IMU_MODE_SPI;
  imu.settings.device.mAddress = LSM9DS1_M_CS;
  imu.settings.device.agAddress = LSM9DS1_AG_CS;
  
  if (!imu.begin())
  {
    Serial.println("Failed to communicate with LSM9DS1.");
    Serial.println("Double-check wiring.");
    Serial.println("LSM9DS1 --------- Arduino Mega 2560" \
                      "CS_AG ------------ 22" \
                      "CS_M ------------- 23" \
                      "SDO_AG ----------- 50" \
                      "SDO_M ------------ 50 (tied to SDO_AG)" \
                      "SCL -------------- 52" \
                      "SDA -------------- 51" \
                      "VDD -------------- 3.3V" \
                      "GND -------------- GND");
    while (1) // Stop program if not able to comunicate with imu
      ;
  }
}

void ImuLoop()
{

  readGyroscope();
  readAcc();
  
 /* printGyro();  // Print "G: gx, gy, gz"
  printAccel(); // Print "A: ax, ay, az"
  printMag();   // Print "M: mx, my, mz"
  
  // Print the heading and orientation for fun!
  // Call print attitude. The LSM9DS1's magnetometer x and y
  // axes are opposite to the accelerometer, so my and mx are
  // substituted for each other.
  printAttitude(imu.ax, imu.ay, imu.az, -imu.my, -imu.mx, imu.mz);
  Serial.println();
  
  delay(PRINT_SPEED);*/
}

void readGyroscope(){
  imu.readGyro();
  Gyro = {imu.calcGyro(imu.gx), imu.calcGyro(imu.gy), imu.calcGyro(imu.gz)};
}

void readAcc(){
  imu.readAccel();
  Acc = {imu.calcAccel(imu.ax), imu.calcAccel(imu.ay), imu.calcAccel(imu.az)};
}

void printGyro()
{
  // To read from the gyroscope, you must first call the
  // readGyro() function. When this exits, it'll update the
  // gx, gy, and gz variables with the most current data.
  imu.readGyro();
  
  // Now we can use the gx, gy, and gz variables as we please.
  // Either print them as raw ADC values, or calculated in DPS.
  Serial.print("G: ");
#ifdef PRINT_CALCULATED
  // If you want to print calculated values, you can use the
  // calcGyro helper function to convert a raw ADC value to
  // DPS. Give the function the value that you want to convert.
  Serial.print(imu.calcGyro(imu.gx), 2);
  Serial.print(", ");
  Serial.print(imu.calcGyro(imu.gy), 2);
  Serial.print(", ");
  Serial.println(imu.calcGyro(imu.gz), 2);
#elif defined PRINT_RAW
  Serial.print(imu.gx);
  Serial.print(", ");
  Serial.print(imu.gy);
  Serial.print(", ");
  Serial.println(imu.gz);
#endif
}

void printAccel()
{
  // To read from the accelerometer, you must first call the
  // readAccel() function. When this exits, it'll update the
  // ax, ay, and az variables with the most current data.
  imu.readAccel();
  
  // Now we can use the ax, ay, and az variables as we please.
  // Either print them as raw ADC values, or calculated in g's.
  Serial.print("A: ");
#ifdef PRINT_CALCULATED
  // If you want to print calculated values, you can use the
  // calcAccel helper function to convert a raw ADC value to
  // g's. Give the function the value that you want to convert.
  Serial.print(imu.calcAccel(imu.ax), 2);
  Serial.print(", ");
  Serial.print(imu.calcAccel(imu.ay), 2);
  Serial.print(", ");
  Serial.println(imu.calcAccel(imu.az), 2);
#elif defined PRINT_RAW 
  Serial.print(imu.ax);
  Serial.print(", ");
  Serial.print(imu.ay);
  Serial.print(", ");
  Serial.println(imu.az);
#endif

}

void printMag()
{
  // To read from the magnetometer, you must first call the
  // readMag() function. When this exits, it'll update the
  // mx, my, and mz variables with the most current data.
  imu.readMag();
  
  // Now we can use the mx, my, and mz variables as we please.
  // Either print them as raw ADC values, or calculated in Gauss.
  Serial.print("M: ");
#ifdef PRINT_CALCULATED
  // If you want to print calculated values, you can use the
  // calcMag helper function to convert a raw ADC value to
  // Gauss. Give the function the value that you want to convert.
  Serial.print(imu.calcMag(imu.mx), 2);
  Serial.print(", ");
  Serial.print(imu.calcMag(imu.my), 2);
  Serial.print(", ");
  Serial.println(imu.calcMag(imu.mz), 2);
#elif defined PRINT_RAW
  Serial.print(imu.mx);
  Serial.print(", ");
  Serial.print(imu.my);
  Serial.print(", ");
  Serial.println(imu.mz);
#endif
}

// Calculate pitch, roll, and heading.
// Pitch/roll calculations take from this app note:
// http://cache.freescale.com/files/sensors/doc/app_note/AN3461.pdf?fpsp=1
// Heading calculations taken from this app note:
// http://www51.honeywell.com/aero/common/documents/myaerospacecatalog-documents/Defense_Brochures-documents/Magnetic__Literature_Application_notes-documents/AN203_Compass_Heading_Using_Magnetometers.pdf
void printAttitude(float ax, float ay, float az, float mx, float my, float mz)
{
  float roll = atan2(ay, az);
  float pitch = atan2(-ax, sqrt(ay * ay + az * az));
  
  float heading;
  if (my == 0)
    heading = (mx < 0) ? PI : 0;
  else
    heading = atan2(mx, my);
    
  heading -= DECLINATION * PI / 180;
  
  if (heading > PI) heading -= (2 * PI);
  else if (heading < -PI) heading += (2 * PI);
  else if (heading < 0) heading += 2 * PI;
  
  // Convert everything from radians to degrees:
  heading *= 180.0 / PI;
  pitch *= 180.0 / PI;
  roll  *= 180.0 / PI;
  
  Serial.print("Pitch, Roll: ");
  Serial.print(pitch, 2);
  Serial.print(", ");
  Serial.println(roll, 2);
  Serial.print("Heading: "); Serial.println(heading, 2);
}

struct Coor getAcc(){
  return Acc;
}

struct Coor getGyro(){
  return Gyro;
}

