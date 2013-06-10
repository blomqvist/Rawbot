
#include <Arduino.h>
#include <Wire.h>

void writeToI2C(int device, byte adress, byte val)
{
  Wire.beginTransmission(device);
  Wire.write(adress);
  Wire.write(val);
  Wire.endTransmission();
}

void readFromI2C(int device, byte adress, int num, byte buffer[])
{
  Wire.beginTransmission(device);
  Wire.write(adress);
  Wire.endTransmission();

  Wire.beginTransmission(device);
  Wire.requestFrom(device, num);

  int i= 0;
  while(Wire.available())
  {
    buffer[i]=Wire.read();
    i++;
  }
  Wire.endTransmission();
}
