#include <Math.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_10DOF.h>
#include <TinyGPS.h>

/* Assign a unique ID to the sensors */
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_BMP085_Unified       bmp   = Adafruit_BMP085_Unified(18001);
Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified(20);
Adafruit_10DOF dof = Adafruit_10DOF();

//GPS Varaible
long lat, lon;
TinyGPS gps;

void gpsdump(TinyGPS &gps);
void printFloat(double f, int digits = 2);

//Internal Speed calc variables
double last_x = 0.0;
double last_y = 0.0;
double last_z = 0.0;
double last_velocity_x = 0.0;
double last_velocity_y = 0.0;
double last_velocity_z = 0.0;

//Accelerometer axis variables
double accel_x = 0.0;
double accel_y = 0.0;
double accel_z = 0.0;

double gyro_x = 0.0;
double gyro_y = 0.0;
double gyro_z = 0.0;

double z_offset = 0.0;
double z_offset_gyro = 0.0;

//DEFINE CAMERA PINS
int cam_tx_pin = 14;
int cam_rx_pin = 15;

//DEFINE WHEEL ENCODER PINS
int right_encoder_pin = 32;
int left_encoder_pin = 33;

//DEFINE RIGHT SONIC RANGE PINS
int right_trig_pin = 34;
int right_echo_pin = 35;

//DEFINE LEFT SONIC RANGE PINS
int left_trig_pin = 36;
int right_trip_pin = 37;

//DEFINE SD CARD READER PINS
int sd_miso_pin = 50;
int sd_mosi_pin = 51;
int sd_sck_pin = 52;


int pinI1 = 8;//define I1 interface
int pinI2 = 11;//define I2 interface 
int speedpinA = 9;//enable motor A
int pinI3 = 12;//define I3 interface 
int pinI4 = 13;//define I4 interface 
int speedpinB = 10;//enable motor B
int spead = 127;//define the spead of motor

void displaySensorDetails(void)
{
  sensor_t sensor;

  accel.getSensor(&sensor);
  Serial.println(F("----------- ACCELEROMETER ----------"));
  Serial.print  (F("Sensor:       ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(sensor.max_value); Serial.println(F(" m/s^2"));
  Serial.print  (F("Min Value:    ")); Serial.print(sensor.min_value); Serial.println(F(" m/s^2"));
  Serial.print  (F("Resolution:   ")); Serial.print(sensor.resolution); Serial.println(F(" m/s^2"));
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));

  gyro.getSensor(&sensor);
  Serial.println(F("------------- GYROSCOPE -----------"));
  Serial.print  (F("Sensor:       ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(sensor.max_value); Serial.println(F(" rad/s"));
  Serial.print  (F("Min Value:    ")); Serial.print(sensor.min_value); Serial.println(F(" rad/s"));
  Serial.print  (F("Resolution:   ")); Serial.print(sensor.resolution); Serial.println(F(" rad/s"));
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));
  
  mag.getSensor(&sensor);
  Serial.println(F("----------- MAGNETOMETER -----------"));
  Serial.print  (F("Sensor:       ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(sensor.max_value); Serial.println(F(" uT"));
  Serial.print  (F("Min Value:    ")); Serial.print(sensor.min_value); Serial.println(F(" uT"));
  Serial.print  (F("Resolution:   ")); Serial.print(sensor.resolution); Serial.println(F(" uT"));  
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));

  bmp.getSensor(&sensor);
  Serial.println(F("-------- PRESSURE/ALTITUDE ---------"));
  Serial.print  (F("Sensor:       ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(sensor.max_value); Serial.println(F(" hPa"));
  Serial.print  (F("Min Value:    ")); Serial.print(sensor.min_value); Serial.println(F(" hPa"));
  Serial.print  (F("Resolution:   ")); Serial.print(sensor.resolution); Serial.println(F(" hPa"));  
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));
}

void setup(void)
{
 
  Serial2.begin(4800);
  //Setup pins for motor controller
  pinMode(pinI1,OUTPUT);
  pinMode(pinI2,OUTPUT);
  pinMode(speedpinA,OUTPUT);
  pinMode(pinI3,OUTPUT);
  pinMode(pinI4,OUTPUT);
  pinMode(speedpinB,OUTPUT);
  //end motor controller setup
  Serial.begin(9600);
  
   //GPS SERIAL INIT
  
  //Serial.begin(115200);
  Serial.println(F("Adafruit 10DOF Tester")); Serial.println("");
  
  /* Initialise the sensors */
  if(!accel.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
    while(1);
  }
  if(!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }
  if(!bmp.begin())
  {
    /* There was a problem detecting the BMP085 ... check your connections */
    Serial.print("Ooops, no BMP085 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  if(!gyro.begin())
  {
    /* There was a problem detecting the L3GD20 ... check your connections */
    Serial.print("Ooops, no L3GD20 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  sensors_event_t event;

  sensors_vec_t   orientation;
  
  /* Display the results (acceleration is measured in m/s^2) */
  accel.getEvent(&event);
  
  z_offset = -9.6;
  Serial.print("Z OFFSET ACCELEROMETER: ");
  Serial.println(z_offset);

  z_offset_gyro = -0.03;
  Serial.print("Z OFFSET GYRO : ");
  Serial.println(z_offset_gyro);
  /* Display some basic information on this sensor */
  displaySensorDetails();
}

int i = 0;
void loop(void)
{
  bool newdata = false;
  unsigned long start = millis();
  while (millis() - start < 5000) {  // Update every 5 seconds
    if (feedgps())
      newdata = true;
  }
  if (newdata) {
    gpsdumpSimple(gps);
  }

 /*
 if(i < 100){
  forward();
 }
 else if(i < 130){
  backward();
 }
 else if(i < 160){
  right();
 }
 else if(i < 260){
  left();
}
 
i++;
if(i > 300){
  i = 0;
}
*/
  unsigned long starttime;
  unsigned long endTime;
  
  starttime = millis();
  
  /* Get a new sensor event */
  sensors_event_t gyro_event;
  sensors_event_t accel_event;
  sensors_event_t mag_event;
  sensors_event_t bmp_event;
  sensors_vec_t orientation;
   
  /* Display the results (acceleration is measured in m/s^2) */
  accel.getEvent(&accel_event);

  if( ((fabs(accel_x) - fabs(accel_event.acceleration.x))/accel_event.acceleration.x) > .005){
    accel_x = accel_event.acceleration.x +1.8;
  }
  else{
    accel_x = 0.0;
  }
  
  if(((fabs(accel_y) - fabs(accel_event.acceleration.y))/accel_event.acceleration.y) > .005){
    accel_y = accel_event.acceleration.y+0.45;
  }
  else{
    accel_y = 0.0;
  }

  if(((fabs(accel_z) - fabs(accel_event.acceleration.z))/accel_event.acceleration.z) > .005){
    accel_z = accel_event.acceleration.z-z_offset;
  }
  else{
    accel_z = 0.0;
  }

  Serial.print(F("ACCEL "));
  Serial.print("X: "); Serial.print(accel_x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(accel_y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(accel_z); Serial.print("  ");Serial.println("m/s^2 ");
  
  /* Display the results (magnetic vector values are in micro-Tesla (uT)) */
  //mag.getEvent(&event);
 
  
    //Serial.print(F("MAG   "));
    //Serial.print("X: "); Serial.print(event.magnetic.x); Serial.print("  ");
    //Serial.print("Y: "); Serial.print(event.magnetic.y); Serial.print("  ");
    //Serial.print("Z: "); Serial.print(event.magnetic.z); Serial.print("  ");Serial.println("uT");

/* Calculate the heading using the magnetometer */
mag.getEvent(&mag_event);
if (dof.magGetOrientation(SENSOR_AXIS_Y, &mag_event, &orientation))
{
  float heading = orientation.heading - 160.8;

  /* 'orientation' should have valid .heading data now */
  Serial.print(F("Heading: "));
  Serial.print(heading * 18.0);
  Serial.print(F("; "));
}
  float Pi = 3.14159;
  
  // Calculate the angle of the vector y,x
  //float heading = atan2(event.magnetic.y-29.4,event.magnetic.x-25.0);
  
  // Normalize to 0-360
  //if (heading < 0)
  //{
  //  heading = 360 + heading;
  //}
  //Serial.print("Compass Heading: ");
  //Serial.println(heading);

  /* Display the results (gyrocope values in rad/s) */
  gyro.getEvent(&gyro_event);
    if( ((fabs(gyro_x) - fabs(gyro_event.gyro.x))/gyro_event.gyro.x) > .005){
    gyro_x = gyro_event.gyro.x - .02;
  }
  else{
    gyro_x = 0.0;
  }
  
  if(((fabs(gyro_y) - fabs(gyro_event.gyro.y))/gyro_event.gyro.y) > .005){
    gyro_y = gyro_event.gyro.y - 0.03;
  }
  else{
    gyro_y = 0.0;
  }

  if(((fabs(gyro_z) - fabs(gyro_event.gyro.z))/gyro_event.gyro.z) > .005){
    gyro_z = gyro_event.gyro.z-z_offset_gyro;
  }
  else{
    gyro_z = 0.0;
  }
  Serial.print(F("GYRO  "));
  Serial.print("X: "); Serial.print(gyro_x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(gyro_y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(gyro_z); Serial.print("  ");Serial.print("rad/s "); Serial.print("OFFSET :: ");Serial.println(z_offset_gyro);  
  
  /* Display the pressure sensor results (barometric pressure is measure in hPa) */
  bmp.getEvent(&bmp_event);
  if (bmp_event.pressure)
  {
    /* Display atmospheric pressure in hPa */
    Serial.print(F("PRESS "));
    Serial.print(bmp_event.pressure);
    Serial.print(F(" hPa, "));
    /* Display ambient temperature in C */
    float temperature;
    bmp.getTemperature(&temperature);
    Serial.print(temperature);
    Serial.print(F(" C, "));
    /* Then convert the atmospheric pressure, SLP and temp to altitude    */
    /* Update this next line with the current SLP for better results      */
    float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;
    Serial.print(bmp.pressureToAltitude(seaLevelPressure,
                                        bmp_event.pressure,
                                        temperature)); 
    Serial.println(F(" m"));
  }

  endTime = millis();
  long delta_time = abs(endTime-starttime);

  Serial.print("DELTA TIME : "); Serial.println(delta_time);
 
  /* Display the results (cooridnate values ) */
  double velocity = 0.0;
  
  last_x = getNewCoordVal(convertInchesToMeters(5.0), last_x, last_velocity_x, accel_x, gyro_x, delta_time);
  last_y = getNewCoordVal(convertInchesToMeters(2.0), last_y, last_velocity_y, accel_y, gyro_y, delta_time);
  last_z = getNewCoordVal(convertInchesToMeters(4.625), last_z, last_velocity_z, accel_z, gyro_z, delta_time);
  
  last_velocity_x = last_velocity_x + (accel_x*(delta_time/1000.0));
  last_velocity_y = last_velocity_y + (accel_y*(delta_time/1000.0));
  last_velocity_z = last_velocity_z + (accel_z*(delta_time/1000.0));

  
  if(last_velocity_x < 0.0){
    last_velocity_x = 0.0;
  }
  if(last_velocity_y < 0.0){
    last_velocity_y < 0.0;
  }
  if(last_velocity_z < 0.0){
    last_velocity_z < 0.0;
  }
  
  Serial.print(F("COORDINATES  "));
  Serial.print("X: "); Serial.print(last_x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(last_y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(last_z); Serial.println("  "); 

  Serial.println(F(""));
  delay(500);
}

/**
 * 
 */
double getNewCoordVal(double radius, double x, double velocity, double accel, double radsPerSec, long delta_time){
  double rad = getRotationalDistance(radius, radsPerSec, delta_time);
  double dist = getTraveledDistance(velocity, accel, delta_time, radsPerSec > 0.0);
  
  return x + dist;
}

/**
 * Gets Rotational distance based on rotational velocity and time passed
 * @param radius - radius of arc from rotational edge
 * @param radPerSec - angular velocity in radians/sec
 * @param delta_time - time duration in milliseconds
 */
double getRotationalDistance(double radius, double radsPerSec, long delta_time){
  return radius * (radsPerSec/1000) * delta_time;
}

/**
 * Gets distrance traveled based on velocity, acceration and how much time has passed in milliseconds
 * @param velocity - velocity in m/s
 * @param accel - accelration in meters/s^2
 * @param delta_time - time duration in milliseconds
 */
double getTraveledDistance(double velocity, double accel, long delta_time, long isPositive){
  double distance = velocity*((double)delta_time/1000.0) + (1.0/2.0) * accel * (double)(pow(delta_time/1000.0, 2));
  if(!isPositive){
    return -1.0 * distance;
  }
  return distance;
}

/**
 * Converts inches to meters
 */
double convertInchesToMeters(double inches){
  return inches * 0.0254;
}

/**
 * 
 */
void print_current_time_with_ms (void)
{
  unsigned long time;
  Serial.print("Time: ");
  time = millis();
  //prints time since program started
  Serial.println(time);
}

/**
 * 
 * MOTOR SHIELD
 * 
 */
 void forward()
{
     analogWrite(speedpinA,spead);//input a simulation value to set the speed
     analogWrite(speedpinB,spead);
     digitalWrite(pinI4,HIGH);//turn DC Motor B move clockwise
     digitalWrite(pinI3,LOW);
     digitalWrite(pinI2,LOW);//turn DC Motor A move anticlockwise
     digitalWrite(pinI1,HIGH);
}
void backward()//
{
     analogWrite(speedpinA,spead);//input a simulation value to set the speed
     analogWrite(speedpinB,spead);
     digitalWrite(pinI4,LOW);//turn DC Motor B move anticlockwise
     digitalWrite(pinI3,HIGH);
     digitalWrite(pinI2,HIGH);//turn DC Motor A move clockwise
     digitalWrite(pinI1,LOW);
}
void left()//
{
     analogWrite(speedpinA,spead);//input a simulation value to set the speed
     analogWrite(speedpinB,spead);
     digitalWrite(pinI4,HIGH);//turn DC Motor B move clockwise
     digitalWrite(pinI3,LOW);
     digitalWrite(pinI2,HIGH);//turn DC Motor A move clockwise
     digitalWrite(pinI1,LOW);
}
void right()//
{
     analogWrite(speedpinA,spead);//input a simulation value to set the speed
     analogWrite(speedpinB,spead);
     digitalWrite(pinI4,LOW);//turn DC Motor B move anticlockwise
     digitalWrite(pinI3,HIGH);
     digitalWrite(pinI2,LOW);//turn DC Motor A move clockwise
     digitalWrite(pinI1,HIGH);
}
void stop()//
{
     digitalWrite(speedpinA,LOW);// Unenble the pin, to stop the motor. this should be done to avid damaging the motor. 
     digitalWrite(speedpinB,LOW);
     delay(1000);
 
}


void gpsdump(TinyGPS &gps)
{
  long lat, lon;
  float flat, flon;
  unsigned long age, date, time, chars;
  int year;
  byte month, day, hour, minute, second, hundredths;
  unsigned short sentences, failed;

  gps.get_position(&lat, &lon, &age);
  Serial.print("Lat/Long(10^-5 deg): "); Serial.print(lat); Serial.print(", "); Serial.print(lon); 
  Serial.print(" Fix age: "); Serial.print(age); Serial.println("ms.");
  
  // On Arduino, GPS characters may be lost during lengthy Serial.print()
  // On Teensy, Serial prints to USB, which has large output buffering and
  //   runs very fast, so it's not necessary to worry about missing 4800
  //   baud GPS characters.

  gps.f_get_position(&flat, &flon, &age);
  Serial.print("Lat/Long(float): "); printFloat(flat, 5); Serial.print(", "); printFloat(flon, 5);
  Serial.print(" Fix age: "); Serial.print(age); Serial.println("ms.");

  gps.get_datetime(&date, &time, &age);
  Serial.print("Date(ddmmyy): "); Serial.print(date); Serial.print(" Time(hhmmsscc): ");
    Serial.print(time);
  Serial.print(" Fix age: "); Serial.print(age); Serial.println("ms.");

  gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);
  Serial.print("Date: "); Serial.print(static_cast<int>(month)); Serial.print("/"); 
    Serial.print(static_cast<int>(day)); Serial.print("/"); Serial.print(year);
  Serial.print("  Time: "); Serial.print(static_cast<int>(hour)); Serial.print(":"); 
    Serial.print(static_cast<int>(minute)); Serial.print(":"); Serial.print(static_cast<int>(second));
    Serial.print("."); Serial.print(static_cast<int>(hundredths));
  Serial.print("  Fix age: ");  Serial.print(age); Serial.println("ms.");

  Serial.print("Alt(cm): "); Serial.print(gps.altitude()); Serial.print(" Course(10^-2 deg): ");
    Serial.print(gps.course()); Serial.print(" Speed(10^-2 knots): "); Serial.println(gps.speed());
  Serial.print("Alt(float): "); printFloat(gps.f_altitude()); Serial.print(" Course(float): ");
    printFloat(gps.f_course()); Serial.println();
  Serial.print("Speed(knots): "); printFloat(gps.f_speed_knots()); Serial.print(" (mph): ");
    printFloat(gps.f_speed_mph());
  Serial.print(" (mps): "); printFloat(gps.f_speed_mps()); Serial.print(" (kmph): ");
    printFloat(gps.f_speed_kmph()); Serial.println();

  gps.stats(&chars, &sentences, &failed);
  Serial.print("Stats: characters: "); Serial.print(chars); Serial.print(" sentences: ");
    Serial.print(sentences); Serial.print(" failed checksum: "); Serial.println(failed);
}

void printFloat(double number, int digits)
{
  // Handle negative numbers
  if (number < 0.0) {
     Serial.print('-');
     number = -number;
  }

  // Round correctly so that print(1.999, 2) prints as "2.00"
  double rounding = 0.5;
  for (uint8_t i=0; i<digits; ++i)
    rounding /= 10.0;
  
  number += rounding;

  // Extract the integer part of the number and print it
  unsigned long int_part = (unsigned long)number;
  double remainder = number - (double)int_part;
  Serial.print(int_part);

  // Print the decimal point, but only if there are digits beyond
  if (digits > 0)
    Serial.print("."); 

  // Extract digits from the remainder one at a time
  while (digits-- > 0) {
    remainder *= 10.0;
    int toPrint = int(remainder);
    Serial.print(toPrint);
    remainder -= toPrint;
  }
}


// Get and process GPS data
void gpsdumpSimple(TinyGPS &gps) {
  float flat, flon;
  unsigned long age;
  gps.f_get_position(&flat, &flon, &age);
  Serial.print(flat, 4); Serial.print(", "); 
  Serial.println(flon, 4);
}

// Feed data as it becomes available 
bool feedgps() {
  while (Serial2.available()) {
    if (gps.encode(Serial2.read()))
      return true;
  }
  return false;
}
