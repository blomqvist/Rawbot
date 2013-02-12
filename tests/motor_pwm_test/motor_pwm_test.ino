#define PWM_FREQUENCY 20000 // The motor driver can handle a pwm frequency up to 20kHz
#define F_CPU 16000000L
#define PWMVALUE F_CPU/PWM_FREQUENCY/2 // Frequency is given by F_CPU/(2*N*ICR) - where N is the prescaler, we use no prescaling so frequency is given by F_CPU/(2*ICR) - ICR = F_CPU/PWM_FREQUENCY/2


int M1_DIRA = 2;
int M1_DIRB = 4;
int M2_DIRA = 7;
int M2_DIRB = 8;
int M1_PWM  = 9;
int M2_PWM  = 10;

static int leftPWM = 11, rightPWM = 14;

void setup()
{
  Serial.begin(9600);
  /* Set PWM frequency to 20kHz - see the datasheet http://www.atmel.com/Images/doc8025.pdf page 128-135 */
  // Set up PWM, Phase and Frequency Correct on pin 9 (OC1A) & pin 10 (OC1B) with ICR1 as TOP using Timer1
  TCCR1B = _BV(WGM13) | _BV(CS10); // Set PWM Phase and Frequency Correct with ICR1 as TOP and no prescaling
  ICR1H = (PWMVALUE >> 8); // ICR1 is the TOP value - this is set so the frequency is equal to 20kHz
  ICR1L = (PWMVALUE & 0xFF);
  
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
