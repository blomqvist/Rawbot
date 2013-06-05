#include <Kalman.h>
#include <LCDBackground.h>
#include <PID.h>
#include <LiquidCrystal.h>
#include <Wire.h>
#include "MPU6050.h"

#define BAUD_SPEED 9600

const int M1_DIRA = 2;
const int M1_DIRB = 4;
const int M2_DIRA = 7;
const int M2_DIRB = 8;
const int M1_PWM  = 9;
const int M2_PWM  = 10;
const int CALIB_SAMPLES = 200;
const int GYRO_SENSITIVITY = 131;
const float WANTED_ANGLE = 0.9; // 11,6 262,5 0,22 PID

enum frequency
{
  HZ_1000 = 1,
  HZ_500  = 2,
  HZ_333  = 3,
  HZ_250  = 4,
  HZ_200  = 5,
  HZ_167  = 6,
  HZ_143  = 7,
  HZ_125  = 8,
  HZ_111  = 9,
  HZ_100  = 10,
  HZ_50   = 20,
  HZ_25   = 40
  
};
const int loopTime = HZ_100;
int latestLoop = loopTime, latestTime = loopTime;
unsigned long startTime = 0;
boolean stopped = false;

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

PID pid(0, 0, 0, WANTED_ANGLE, loopTime); // 7, 2, 8

Kalman kalman;

void setup() 
{
  pinMode(53, OUTPUT);
  
  setupLCD();
  Wire.begin();
  lcd.print(".");
  setupMotorShield();
  stop();
  lcd.print(".");
  setupMPU();
  lcd.print(".");
  serialInit(BAUD_SPEED); // Init serial
  lcd.print(".");
  setupTemperature();
  lcd.print(".");
  calibrate();
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Done calibrating");
  lcd.setCursor(0,1);
  lcd.print("Starting in ");
  for(int i = 1; i > 0; i--)
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
  static float _p = 14, _i = 100, _d = 1, _w = 1;
  
  angle = readAngle();
  gyro_y = readRotation();
  
  //fall angle seems to be -0.3, we'll compensate
  //gyro_y += 0.35;
  
  k_angle = kalman.Kalkulate(angle, gyro_y);
  //u_o = pid.Calculate_original(k_angle);
  u_o = pid.Calculate_wiki(k_angle);
  
  if(k_angle > 50 || k_angle < -50)
  {
    stop();
    stopped = true;  
  }
  else
  {
    drive(u_o, _w);
    stopped = false;
  }
  
  //10 times per sec
  if(shortLoop == 100/shortLoop)
  {
    shortLoop = 0;
    if(!stopped)
    {
      lcdB.Tick();
    }
    if(readFromSerial(_p, _i, _d, _w))
    {
      pid.SetPidConstants(_p, _i, _d);  
    }
  }
  
  //once per sec
  if(longLoop == 1000/loopTime)
  {
    static int internalLongLoop = 0;

    longLoop = 0;
    temperature = getTemperature();
    
    if(stopped)
    {
      lcdB.LCDStop();  
    }        
    
    // Debug follows
    if(internalLongLoop % 2 == 0)
    {
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print(_p);
      lcd.print(" ");
      lcd.print(_i);
      lcd.print(" ");
      lcd.print(_d);      
    }
    else
    {
      //lcd.print("pwm: ");
      //lcd.print(((int)u_o > 0) ? (_w + (int)u_o) : ((-_w) + (int)u_o));
      lcd.setCursor(0, 1);
      lcd.print(pid.GetWantedAngle());
      /*lcd.print("");
      lcd.print(gyro_y);
      lcd.print(" ");
      lcd.print(angle);
      lcd.print(" ");
      lcd.print(k_angle);
      */
    }
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

void drive(float u_o, float w)
{
  static int debug_i = 0;
  
  float pwm = u_o;
  
  //set direction
  if(pwm > 0)
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
  
  pwm *= w;
  pwm = min(abs(pwm), 245);
  pwm += 10;
  
  if(debug_i++ == 100)
  {
    debug_i = 0;
    Serial.println(pwm);  
  }
  
  analogWrite(M1_PWM, (int)pwm);
  analogWrite(M2_PWM, (int)pwm);
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
  gyro_x = -((buffer_gyro[0]<<8) | buffer_gyro[1]);
  return gyro_x / (float)GYRO_SENSITIVITY;
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

boolean readFromSerial(float& p, float& i, float& d, float& w)
{
  char _tempin[11]; // char to read
  unsigned char _in[11];
  unsigned int _p = 0, _i = 0, _d = 0, _w = 0;
  int mode, drive;
  boolean b = false;
  
  if(Serial.available() > 0)
  {
    int nrRead = Serial.readBytes(_tempin, sizeof(_tempin));
    if(nrRead < sizeof(_tempin))
    {
      Serial.print("_tempin too small ");
      Serial.print(nrRead);
      Serial.print(" ");
      Serial.println((char*)_tempin);
      return false;  
    }
  
    for(int i = 0; i < sizeof(_in); i++)
    {
      _in[i] = (unsigned char)(_tempin[i]);  
    }
  
    mode  = (int)_in[0];    
    
    if(mode == 4) //DRIVE
    {
      drive = (unsigned int)((_in[9] << 8) | _in[10]);  
      Serial.println(drive);
      
      if(drive == 1) //UP
      {
        pid.SetWantedAngle(pid.GetWantedAngle() + 0.05);  
      }
      else if(drive == 2) //DOWN
      {
        pid.SetWantedAngle(pid.GetWantedAngle() - 0.05);  
      }
      else if(drive == 3) //RIGHT
      {
        
      }
      else if(drive == 4) //LEFT
      {
        
      }
    }
    else
    {
      _p    = (unsigned int)((_in[1] << 8) | _in[2]);
      _i    = (unsigned int)((_in[3] << 8) | _in[4]);
      _d    = (unsigned int)((_in[5] << 8) | _in[6]);      
      _w    = (unsigned int)((_in[7] << 8) | _in[8]);  
      
      p = (float)_p / 10;
      i = (float)_i / 10;
      d = (float)_d / 10;
      w = (float)_w / 10;    
      
      b = true;
    }    
  }
  
  return b;
}

