#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_10DOF.h>

/* Assign a unique ID to the sensors */
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_BMP085_Unified       bmp   = Adafruit_BMP085_Unified(18001);
Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified(20);

double last_x = 0.0;
double last_y = 0.0;
double last_z = 0.0;
double last_velocity_x = 0.0;
double last_velocity_y = 0.0;
double last_velocity_z = 0.0;

double accel_x = 0.0;
double accel_y = 0.0;
double accel_z = 0.0;

double gyro_x = 0.0;
double gyro_y = 0.0;
double gyro_z = 0.0;

double z_offset = 0.0;
double z_offset_gyro = 0.0;

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
  Serial.begin(115200);
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
   
  /* Display the results (acceleration is measured in m/s^2) */
  accel.getEvent(&event);
  
  z_offset = event.acceleration.z;
  Serial.print("Z OFFSET ACCELEROMETER: ");
  Serial.println(z_offset);

  gyro.getEvent(&event);
  z_offset_gyro = event.gyro.z;
  Serial.print("Z OFFSET GYRO : ");
  Serial.println(z_offset_gyro);
  /* Display some basic information on this sensor */
  displaySensorDetails();
}


void loop(void)
{
  unsigned long starttime;
  unsigned long endTime;
  
  starttime = millis();
  
  /* Get a new sensor event */
  sensors_event_t event;
   
  /* Display the results (acceleration is measured in m/s^2) */
  accel.getEvent(&event);

  if( (((accel_x + (event.acceleration.x))/2 - accel_x)/accel_x) > .15){
    accel_x = (event.acceleration.x +1.8)/2;
  }
  else{
    accel_x = 0.0;
  }
  
  if((((accel_y + event.acceleration.y)/2 - accel_y)/accel_y) > .15){
    accel_y = (event.acceleration.y+0.45)/2;
  }
  else{
    accel_y = 0.0;
  }

  if(((((event.acceleration.z-z_offset)- accel_z)/4)/accel_z) > .15){
    accel_z = (event.acceleration.z-z_offset)/2;
  }
  else{
    accel_z = 0.0;
  }

  if( (((gyro_x + event.gyro.x)/2 - gyro_x)/gyro_x) > .1){
    gyro_x = (event.gyro.x - .02)/2;
  }
  else{
    gyro_x = 0.0;
  }
  
  if((((gyro_y + event.gyro.y)/2 - gyro_y)/gyro_y) > .1){
    gyro_y = (event.gyro.y - 0.03)/2;
  }
  else{
    gyro_y = 0.0;
  }

  if((((event.gyro.z - gyro_z)/2)/gyro_z) > .1){
    gyro_z = event.gyro.z-z_offset_gyro;
  }
  else{
    gyro_z = 0.0;
  }

  Serial.print(F("ACCEL "));
  Serial.print("X: "); Serial.print(accel_x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(accel_y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(accel_z); Serial.print("  ");Serial.println("m/s^2 ");
  
  /* Display the results (magnetic vector values are in micro-Tesla (uT)) */
  mag.getEvent(&event);
  Serial.print(F("MAG   "));
  Serial.print("X: "); Serial.print(event.magnetic.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(event.magnetic.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(event.magnetic.z); Serial.print("  ");Serial.println("uT");

  /* Display the results (gyrocope values in rad/s) */
  gyro.getEvent(&event);
  Serial.print(F("GYRO  "));
  Serial.print("X: "); Serial.print(gyro_x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(gyro_y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(gyro_z); Serial.print("  ");Serial.println("rad/s ");  
  
  /* Display the pressure sensor results (barometric pressure is measure in hPa) */
  bmp.getEvent(&event);
  if (event.pressure)
  {
    /* Display atmospheric pressure in hPa */
    Serial.print(F("PRESS "));
    Serial.print(event.pressure);
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
                                        event.pressure,
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
  
  last_velocity_x = last_velocity_x + (accel_x*(delta_time/1000));
  last_velocity_y = last_velocity_y + (accel_y*(delta_time/1000));
  last_velocity_z = last_velocity_z + (accel_z*(delta_time/1000));

  
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
  delay(50);
}

/**
 * 
 */
double getNewCoordVal(double radius, double x, double velocity, double accel, double radsPerSec, long delta_time){
  double rad = getRotationalDistance(radius, radsPerSec, delta_time);
  double dist = getTraveledDistance(velocity, accel, delta_time);

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
double getTraveledDistance(double velocity, double accel, long delta_time){
  double v_f = velocity + accel*delta_time;
  double distance = (((velocity+v_f)/1000)*delta_time)/2;
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
