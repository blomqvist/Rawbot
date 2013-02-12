int M1_DIRA = 2;
int M1_DIRB = 4;
int M2_DIRA = 7;
int M2_DIRB = 8;
int M1_PWM  = 9;
int M2_PWM  = 10;

static int leftPWM = 0, rightPWM = 0;

void setup()
{
  Serial.begin(9600);
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
  
  analogWrite(M1_PWM, leftPWM);
  analogWrite(M2_PWM, rightPWM);
}

void loop()
{
  readSerial(leftPWM, rightPWM);
  digitalWrite(M1_DIRA, HIGH);
  digitalWrite(M1_DIRB, LOW);
  analogWrite(M1_PWM, leftPWM);
  digitalWrite(M2_DIRA, HIGH);
  digitalWrite(M2_DIRB, LOW);
  analogWrite(M2_PWM, rightPWM);
  delay(1500);

/*  digitalWrite(M1_DIRA, LOW);
  digitalWrite(M1_DIRB, LOW);
  analogWrite(M1_PWM, 0);
  digitalWrite(M2_DIRA, LOW);
  digitalWrite(M2_DIRB, LOW);
  analogWrite(M2_PWM, 40);
  delay(3000);
*/

  digitalWrite(M1_DIRA, LOW);
  digitalWrite(M1_DIRB, HIGH);
  analogWrite(M1_PWM, leftPWM);
  digitalWrite(M2_DIRA, LOW);
  digitalWrite(M2_DIRB, HIGH);
  analogWrite(M2_PWM, rightPWM);  
  delay(1500);

/*
  digitalWrite(M1_DIRA, HIGH);
  digitalWrite(M1_DIRB, HIGH);
  analogWrite(M1_PWM, 40);
  digitalWrite(M2_DIRA, HIGH);
  digitalWrite(M2_DIRB, HIGH);
  analogWrite(M2_PWM, 40);
  
  delay(3000);
*/  
}

void readSerial(int& left, int& right)
{
  char _in; // char to read
  int _times = 0;
  int read_to = 'l'; // which int to read in to
  if (Serial.available() > 0)
  {
    while ((_in = Serial.read()) != '\n')
    {
      if (_in != ',')
      {
        if (read_to == 'l')      left  = left * _times + ((int)_in - 48);
        else                     right = right * _times + ((int)_in - 48);
        _times = 10;
      }
      else
      {
        if      (read_to == 'l') read_to = 'r';
        else                     read_to = 'l';
        _times = 0;
      }
    }   
  }
}
