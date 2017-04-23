#include <Arduino.h>
#include <SPI.h>
#if not defined (_VARIANT_ARDUINO_DUE_X_) && not defined (_VARIANT_ARDUINO_ZERO_)
#include <SoftwareSerial.h>
#endif

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS0.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"

#define FACTORYRESET_ENABLE         1
#define MINIMUM_FIRMWARE_VERSION    "0.6.6"
#define MODE_LED_BEHAVIOUR          "MODE"

Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(1000);  // Use I2C, ID #1000

#define LSM9DS0_XM_CS 10
#define LSM9DS0_GYRO_CS 9

#define LSM9DS0_SCLK 13
#define LSM9DS0_MISO 12
#define LSM9DS0_MOSI 11

Adafruit_BluefruitLE_UART ble(BLUEFRUIT_HWSERIAL_NAME, BLUEFRUIT_UART_MODE_PIN);

String message;
double bx, by, bz, BM;
double bgx, bgy, bgz;
boolean fall = false;
boolean trigger1 = false;
boolean trigger2 = false;
boolean trigger3 = false;
byte trigger1count = 0;
byte trigger2count = 0;
byte trigger3count = 0;
int FALL;
int luku;

void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

void setup(void)
{
  while (!Serial);  // required for Flora & Micro
  delay(500);

  Serial.begin(115200);

  Serial.println("Mummo 0.4"); Serial.println("");

  /* Initialise the sensor */
  if (!lsm.begin())
  {
    /* There was a problem detecting the LSM9DS0 ... check your connections */
    Serial.print("Ooops, no LSM9DS0 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  /* Display some basic information on this sensor */
  displaySensorDetails();

  /* Setup the sensor gain and integration time */
  configureSensor();

  /* We're ready to go! */
  Serial.println("");

  Serial.println(F("Adafruit Bluefruit Command <-> Data Mode Example"));
  Serial.println(F("------------------------------------------------"));

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ) {
      error(F("Couldn't factory reset"));
    }
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  Serial.println(F("Please use Adafruit Bluefruit LE app to connect in UART mode"));
  Serial.println(F("Then Enter characters to send to Bluefruit"));
  Serial.println();

  ble.verbose(false);  // debug info is a little annoying after this point!

  /* Wait for connection */
  while (! ble.isConnected()) {
    delay(500);
  }

  Serial.println(F("******************************"));

  // LED Activity command is only supported from 0.6.6
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    // Change Mode LED Activity
    Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
  }

  // Set module to DATA mode
  Serial.println( F("Switching to DATA mode!") );
  ble.setMode(BLUEFRUIT_MODE_DATA);

  Serial.println(F("******************************"));
}

void loop(void)
{
  /* Get a new sensor event */
  sensors_event_t accel, mag, gyro, temp;

  lsm.getEvent(&accel, &mag, &gyro, &temp);




  double x, y, z, gx, gy, gz;
  double AM, GM;
  double gyroAngle = 0;
  x = accel.acceleration.x;
  y = accel.acceleration.y;
  z = accel.acceleration.z;

  gx = gyro.gyro.x;
  gy = gyro.gyro.y;
  gz = gyro.gyro.z;

  AM = pow(pow(x, 2) + pow(y, 2) + pow(z, 2), 0.5); //calculating Amplitute vector for 3 axis
  GM = pow(pow(gx, 2) + pow(gy, 2) + pow(gz, 2), 0.5); //never used, just for verification

  if (trigger3 == true) {
    trigger3count++;

    if (trigger3count >= 10) {
      gyroAngle = pow(pow(gx, 2) + pow(gy, 2) + pow(gz, 2), 0.5); //fall alarmed if the angle stays.
      Serial.println(gyroAngle); Serial.print("trigger 3:n kulma");
      if (gyroAngle >= 0 && gyroAngle <= 10) {
        fall = true; trigger3 = false; trigger3count = 0;
      }
      else {
        trigger3 = false; trigger3count = 0;
        Serial.println("TRIGGER 3 DEACTIVATED");
      }
    }
  }
  if (fall == true) {
    Serial.println("FALL DETECTED");
    FALL = FALL + 1; //fall counter;
    fall = false;
    message = "# FALL DETECTED, FALL " + String(FALL);
    ble.println(message); //only sends fall message if it happens
  }

  if (trigger2count >= 6) {
    trigger2 = false; trigger2count = 0;
    Serial.println("TRIGGER 2 DEACTIVATED");
  }

  if (trigger1count >= 6) {
    trigger1 = false; trigger1count = 0;
    Serial.println("TRIGGER 1 DEACTIVATGED");
  }

  if (trigger2 == true) {
    trigger2count++;
    gyroAngle = pow(pow(gx, 2) + pow(gy, 2) + pow(gz, 2), 0.5); Serial.print(gyroAngle);
    if (gyroAngle >= 30 && gyroAngle <= 400) {
      trigger3 = true; trigger2 = false; trigger2count = 0;
      Serial.println(gyroAngle);
      Serial.println("TRIGGER 3 ACTIVATED");
    }
  }
  //if (trigger1 == true) {
  //  trigger1count++;
  if (AM >= 12) {
    trigger2 = true;
    Serial.println("TRIGGER 2 ACTIVATGED");
    trigger1 = false; trigger1count = 0;
  }
  // }

  // if (AM <= 2 && trigger2==false) {
  // trigger1 = true;
  //Serial.println("TRIGGER 1 ACTIVATED");
  //}
  //double testgyroAngle;
  //testgyroAngle = pow(pow(gx,2)+pow(gy,2)+pow(gz,2),0.5);
  //Serial.print("AM "); Serial.print(AM); Serial.print(" Angle: "); Serial.print(testgyroAngle);
  //message = ("# Accelerometer" + String(AM) + "FALL" + String(trigger1));
  //ble.println(message);

  delay(100);
}

void displaySensorDetails(void)
{
  sensor_t accel, mag, gyro, temp;

  lsm.getSensor(&accel, &mag, &gyro, &temp);

  Serial.println(F("------------------------------------"));
  Serial.print  (F("Sensor:       ")); Serial.println(accel.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(accel.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(accel.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(accel.max_value); Serial.println(F(" m/s^2"));
  Serial.print  (F("Min Value:    ")); Serial.print(accel.min_value); Serial.println(F(" m/s^2"));
  Serial.print  (F("Resolution:   ")); Serial.print(accel.resolution); Serial.println(F(" m/s^2"));
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));

  Serial.println(F("------------------------------------"));
  Serial.print  (F("Sensor:       ")); Serial.println(mag.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(mag.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(mag.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(mag.max_value); Serial.println(F(" uT"));
  Serial.print  (F("Min Value:    ")); Serial.print(mag.min_value); Serial.println(F(" uT"));
  Serial.print  (F("Resolution:   ")); Serial.print(mag.resolution); Serial.println(F(" uT"));
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));

  Serial.println(F("------------------------------------"));
  Serial.print  (F("Sensor:       ")); Serial.println(gyro.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(gyro.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(gyro.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(gyro.max_value); Serial.println(F(" rad/s"));
  Serial.print  (F("Min Value:    ")); Serial.print(gyro.min_value); Serial.println(F(" rad/s"));
  Serial.print  (F("Resolution:   ")); Serial.print(gyro.resolution); Serial.println(F(" rad/s"));
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));

  Serial.println(F("------------------------------------"));
  Serial.print  (F("Sensor:       ")); Serial.println(temp.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(temp.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(temp.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(temp.max_value); Serial.println(F(" C"));
  Serial.print  (F("Min Value:    ")); Serial.print(temp.min_value); Serial.println(F(" C"));
  Serial.print  (F("Resolution:   ")); Serial.print(temp.resolution); Serial.println(F(" C"));
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));

  delay(500);
}

void configureSensor(void)
{
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_6G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_16G);

  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_12GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_2000DPS);
}








