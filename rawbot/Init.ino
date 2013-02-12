void setupLCD()
{
  /*
   * Pin 23 == RW == disabled
   * */
  pinMode(23, OUTPUT);
  digitalWrite(23, LOW);
  
  lcd.begin(16, 2);
}

void setupMPU()
{
  /*
  * Set gyro to max sensitivity
  * B7-B5 - Self test, not used = 0
  * B4-B3 - Gyro sensitivity = 0 (max sensitivity, +-250 deg/s)
  * B2-B0 - Unused = 0
  */
  writeToI2C(MPU6050_I2C_ADDRESS, MPU6050_GYRO_CONFIG, 0x00);  

  /*
  * Set accel to max sensitivity
  * B7-B5 - Self test, not used = 0
  * B4-B3 - Accel sensitivity = 0 (max sensitivity, +-2g)
  * B2-B0 - Unused = 0
  */
  writeToI2C(MPU6050_I2C_ADDRESS, MPU6050_ACCEL_CONFIG, 0x00);  
  
  /*
  * Set internal clock to Gyro X
  * Set sleep disabled
  * B7 - DEVICE_RESET = 0
  * B6 - SLEEP        = 0
  * B5 - CYCLE        = 0
  * B4 - NOT USED     = 0
  * B3 - DISABLE_TEMP = 0
  * B2 - CLKSEL       = 0
  * B1 - CLKSEL       = 0
  * B0 - CLKSEL       = 1
  *
  * B2-B0 - 3 bit value to set the internal clock. 1 == X gyro.
  */
  writeToI2C(MPU6050_I2C_ADDRESS, MPU6050_PWR_MGMT, 0x01);   
}

void setupTemperature()
{
  float t = 0;
  for(int i = 0; i < 10; i++)
  {
    t += getTemperature();
    delay(100);
  }

  temperature = t/10;  
}

void calibrate()
{
  float gyro_x;
  float angle, startAngle;

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Calibrating");
  lcd.setCursor(0,1);
  
  /* read out the angle before calibrating, 
   * to use as an input to the calibration. 
   */
  //to avoid interference, read a few values
  for (int i = 0; i < 5; i++)
  {
    angle += readAngle();  
  }
  if(angle/5 < 20 && angle/5 > -20)
  {
    startAngle = 0; //assume upright position
  }
  else if(angle/5 >= 20)
  {
    startAngle = 90; //assume front down 
  }
  else if(angle/5 <= -20)
  {
    startAngle = -90; //assume back down  
  }
  else
  {
    startAngle = 0; //shouldn't happen  
  }
  kalman.SetStartAngle(startAngle);
  kalman.SetFrequency(loopTime);
   
  for (int i = 0; i < CALIB_SAMPLES; i++)
  {
    gyro_x = readRotation();
    //KALMAN_x_bias = KALMAN_x_bias + gyro_x;
    kalman.K_bias += gyro_x;
    delay(10);
    if(i % (CALIB_SAMPLES/5) == 0)
    {
      lcd.print(".");
    }
  }
  //KALMAN_x_bias=(-KALMAN_x_bias/CALIB_SAMPLES)*PI/180;  
  kalman.K_bias = (-kalman.K_bias / CALIB_SAMPLES) * PI/180;
    
  /* Run Kalman filter to get good values */
  for (int i = 0; i < CALIB_SAMPLES; i++)
  {
    angle = readAngle();
    gyro_x = readRotation();
    //kalmanCalculate(angle, gyro_x, float(latestLoop));
    kalman.Kalkulate(angle, gyro_x);
    if(i % (CALIB_SAMPLES/5) == 0)
    {
      lcd.print(".");
    }
  }
}

void setupMotorShield()
{
  pinMode(M1_DIRA, OUTPUT);
  pinMode(M1_DIRB, OUTPUT);
  pinMode(M2_DIRA, OUTPUT);
  pinMode(M2_DIRB, OUTPUT);
  pinMode(M1_PWM, OUTPUT);
  pinMode(M2_PWM, OUTPUT);
  
  digitalWrite(M1_DIRA, LOW);
  digitalWrite(M1_DIRB, LOW);
  digitalWrite(M2_DIRA, LOW);
  digitalWrite(M2_DIRB, LOW);
  
  analogWrite(M1_PWM, 0);
  analogWrite(M2_PWM, 0);
}

void serialInit(int baud_speed)
{
  Serial.begin(baud_speed);
}

