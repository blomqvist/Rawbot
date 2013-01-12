#include <LiquidCrystal.h>
#include <Wire.h>
#include "MPU6050.h"

const int CALIB_SAMPLES = 200;
const int GYRO_SENSITIVITY = 131;

/* Kalman variables */
const float KALMAN_Q_angle = 0.001;
const float KALMAN_Q_gyro  = 0.003;
const float KALMAN_R_angle = 0.03;
//float KALMAN_x_angle = 0;
float KALMAN_x_angle = 88.6;
float KALMAN_x_bias = 0;
float KALMAN_P[2][2] = {{0,0},{0,0}};

/* Time variables for loop control */
int loopTime = 8; // 200Hz
int latestLoop = loopTime, latestTime = loopTime;
unsigned long startTime = 0;

/* temperature */
float temperature;

/*
 * RS 	   = pin 22 (controls the memory of the LCD)
 * RW	   = pin 23 (GND will be set to low)
 * Enable  = pin 24 (enables writing to the registers)
 * d4 - d7 = pin 25 - 28 (data pins, 4 bits)
 */
LiquidCrystal lcd(22, 24, 25, 26, 27, 28);
const int pinLCDR = 4, pinLCDG = 3, pinLCDB = 2;

void setup() 
{
  setupLCD();
  lcd.print(".");
  Wire.begin();
  lcd.print(".");
  setupMPU();
  lcd.print(".");
  setupTemperature();
  lcd.print(".");
  printTemperature();
  calibrate();
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Done calibrating");
  lcd.setCursor(0,1);
  lcd.print("Starting in 3");
  lcd.setCursor(12,1);
  delay(1000);
  lcd.print("2");
  lcd.setCursor(12,1);
  delay(1000);
  lcd.print("1");
  delay(1000);
  lcd.clear();  
}

void loop() 
{
  static int laps = 0;
  static float angle;
  static float gyro_y;
  static float k_angle;
  
  angle = readAngle();
  gyro_y = readRotation();
  k_angle = kalmanCalculate(angle, gyro_y, float(latestLoop));
  
  if(laps == 200)
  {
    laps = 0;
    temperature = getTemperature();
    printTemperature();
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("k_angle: ");
    lcd.print(k_angle);  
    lcd.setCursor(0,1);
    lcd.print("Temp ");
    lcd.print(temperature);
  }

  laps++;
  loopControl();
}

float getTemperature()
{
  byte byteBuffer[2];
  int t;
  readFromI2C(MPU6050_I2C_ADDRESS, MPU6050_TMP_AR_HIGH, 2, byteBuffer);
  t = ((byteBuffer[0] << 8) | byteBuffer[1]);
  return ((float)t/340 + 36.53);
}

void printTemperature()
{
  static float oldTemperature = temperature;
  int limit = 0.1;
  int r_pwm = 0;
  int g_pwm = 0;
  int b_pwm = 0;
    
  if(temperature >= oldTemperature - limit || temperature <= oldTemperature + limit)
  {
    g_pwm = 255;
  }
  else if(temperature < oldTemperature - limit)
  {
    b_pwm = 255;
  }
  else if(temperature > oldTemperature + limit)
  {
    r_pwm = 255;  
  }
  else
  {
    r_pwm = 255;
    g_pwm = 255;
    b_pwm = 255;
  }
  
  analogWrite(pinLCDR, r_pwm);
  analogWrite(pinLCDG, g_pwm);
  analogWrite(pinLCDB, g_pwm);
}

float readAngle()
{
  static byte bufferAcc[4];
  static int16_t acc_y, acc_z;
  static float acc_theta;
  
  readFromI2C(MPU6050_I2C_ADDRESS, MPU6050_YACC_AR_HIGH, 4, bufferAcc);
  acc_y = ((bufferAcc[0] << 8) | bufferAcc[1]); // HIGH - LOW
  acc_z = ((bufferAcc[2] << 8) | bufferAcc[3]); // HIGH - LOW
  
  acc_theta = atan2(-acc_y, acc_z) * RAD_TO_DEG;
  
  return acc_theta;
}

float readRotation()
{
  static int16_t gyro_x;
  static byte buffer_gyro[2];
  
  readFromI2C(MPU6050_I2C_ADDRESS, MPU6050_XGYR_AR_HIGH, 2, buffer_gyro);
  //convert raw gyro data to deg/s
  gyro_x = ((buffer_gyro[0]<<8) | buffer_gyro[1]);
  return float(gyro_x / GYRO_SENSITIVITY);
}


void loopControl()
{
  /* When was the last time we were here? */
  latestLoop = millis() - startTime;
  /* if the last loop was done in less time than loopTime */
  if(latestLoop < loopTime)
  {
    /* delay so the total delay time is >= loopTime
     * NOTE! As it is now this can never be >1 ms */
    delay(loopTime - latestLoop);
  }
  /* save the time when we were done here */
  startTime = millis();
}
