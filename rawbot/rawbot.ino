#include <Kalman.h>
#include <LCDBackground.h>
#include <PID.h>
#include <LiquidCrystal.h>
#include <Wire.h>
#include "MPU6050.h"

const int M1_DIRA = 2;
const int M1_DIRB = 4;
const int M2_DIRA = 7;
const int M2_DIRB = 8;
const int M1_PWM  = 9;
const int M2_PWM  = 10;
const int CALIB_SAMPLES = 200;
const int GYRO_SENSITIVITY = 131;

enum frequency
{
  HZ_1000 = 1,
  HZ_500  = 2,
  HZ_333  = 3,
  HZ_250  = 4,
  HZ_200  = 5,
  HZ_167  = 6,
  HZ_143  = 7,
  HZ_125  = 8  
};
const int loopTime = HZ_200;
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
LCDBackground lcdB(11, 5, 3);

PID pid(200, 20, 10, 0, loopTime);

Kalman kalman;

void setup() 
{
  pinMode(53, OUTPUT);
  pinMode(51, OUTPUT);
  digitalWrite(51, HIGH);  
  
  setupLCD();
  Wire.begin();
  lcd.print(".");
  setupMotorShield();
  lcd.print(".");
  setupMPU();
  lcd.print(".");
  setupTemperature();
  lcd.print(".");
  calibrate();
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Done calibrating");
  lcd.setCursor(0,1);
  lcd.print("Starting in ");
  for(int i = 3; i > 0; i--)
  {
    lcd.setCursor(12, 1);    
    lcd.print(i);
    delay(1000);
  }
  lcd.clear();  
}

void loop() 
{
  static int longLoop  = 0;
  static int shortLoop = 0;
  static float angle;
  static float gyro_y;
  static float k_angle;
  static float u_o, u_w;
  
  angle = readAngle();
  gyro_y = readRotation();
  //k_angle = kalmanCalculate(angle, gyro_y, float(latestLoop));
  k_angle = kalman.Kalkulate(angle, gyro_y);
  //u = PID(k_angle);
  u_o = pid.Calculate_original(k_angle);
  u_w = pid.Calculate_wiki(k_angle);
  
  if(k_angle > 15 || k_angle < -15)
  {
    stop();  
  }
  else
  {
    drive(u_o);
  }
  
  //10 times per sec
  if(shortLoop == 100/shortLoop)
  {
    shortLoop = 0;
    lcdB.Tick();
  }
  
  //once per sec
  if(longLoop == 1000/loopTime)
  {
    static int internalLongLoop = 0;

    longLoop = 0;
    temperature = getTemperature();
  
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("ka ");
    lcd.print(k_angle);
    lcd.print(" T ");
    lcd.print(temperature);    
    lcd.setCursor(0,1);
    lcd.print("o ");
    lcd.print(u_o);     
    lcd.print(" w ");
    lcd.print(u_w);  
            
    /*should only be done every 10th time
    * ie active 1 second and 
    * inactive 9 seconds
    */
    if(internalLongLoop == 10)
    {
      internalLongLoop = 0;
    }
    else
    {
    }
    
    internalLongLoop++;
  }
  
  shortLoop++;
  longLoop++;
  loopControl();
}

void drive(float u_o)
{
  int pwm = (int)u_o;
  
  //set direction
  if(pwm < 0)
  {
    digitalWrite(M1_DIRA, LOW);
    digitalWrite(M1_DIRB, HIGH);
    digitalWrite(M2_DIRA, HIGH);
    digitalWrite(M2_DIRB, LOW);
  }
  else
  {
    digitalWrite(M1_DIRA, HIGH);
    digitalWrite(M1_DIRB, LOW);
    digitalWrite(M2_DIRA, LOW);
    digitalWrite(M2_DIRB, HIGH);
  }
  pwm = 20 +  abs(pwm);
  if(pwm > 255)
  {
    pwm = 255; 
  }
  
  analogWrite(M1_PWM, pwm);
  analogWrite(M2_PWM, pwm);
}

void stop()
{
  digitalWrite(M1_DIRA, LOW);
  digitalWrite(M1_DIRB, LOW);
  digitalWrite(M2_DIRA, LOW);
  digitalWrite(M2_DIRB, LOW);
  
  analogWrite(M1_PWM, 0);
  analogWrite(M2_PWM, 0);
}

float getTemperature()
{
  byte byteBuffer[2];
  int t;
  readFromI2C(MPU6050_I2C_ADDRESS, MPU6050_TMP_AR_HIGH, 2, byteBuffer);
  t = ((byteBuffer[0] << 8) | byteBuffer[1]);
  return ((float)t/340 + 36.53);
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
    digitalWrite(53, HIGH);
    /* delay so the total delay time is >= loopTime
     * NOTE! As it is now this can never be >1 ms */
    delay(loopTime - latestLoop);
    digitalWrite(53, LOW);  
  }
  /* save the time when we were done here */
  startTime = millis();
}
