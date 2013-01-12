void setupLCD()
{
  /*
   * Pin 23 == RW == disabled
   * */
  pinMode(23, OUTPUT);
  digitalWrite(23, LOW);
  pinMode(pinLCDR,   OUTPUT);
  pinMode(pinLCDG, OUTPUT);
  pinMode(pinLCDB,  OUTPUT);
	//r_pwm = r_pwm = b_pwm = 0;

  analogWrite(pinLCDR, 63);
  analogWrite(pinLCDG, 0);
  analogWrite(pinLCDB, 127);

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
  float angle;

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Calibrating");
  lcd.setCursor(0,1);
 
  for (int i = 0; i < CALIB_SAMPLES; i++)
  {
    gyro_x = readRotation();
    KALMAN_x_bias = KALMAN_x_bias + gyro_x;
    delay(10);
    if(i % (CALIB_SAMPLES/5) == 0)
    {
      lcd.print(".");
    }
  }
  KALMAN_x_bias=(-KALMAN_x_bias/CALIB_SAMPLES)*PI/180;  
  
  /* Run Kalman filter to get good values */
  for (int i = 0; i < CALIB_SAMPLES; i++)
  {
    angle = readAngle();
    gyro_x = readRotation();
    kalmanCalculate(angle, gyro_x, float(latestLoop));
    if(i % (CALIB_SAMPLES/5) == 0)
    {
      lcd.print(".");
    }
  }
}
  

