#include <EEPROM.h>

#include <avr/io.h>
#include <avr/interrupt.h>

const int NES_CLOCK  = 2;
const int NES_LATCH  = 3;
const int NES_DATA   = 4;

const int SR_CLOCK   = 11;
const int SR_LATCH   = 9;
const int SR_DATA    = 10;

const int DIG_1 = 8;
const int DIG_2 = 7;
const int DIG_3 = 6;
const int DIG_4 = 5;

const int LAMP_1 = 12;

/*  if (value) {
  PORTD |= (1<<7);
} else {
  PORTD &= ~(1<<7);
}
*/
/*const byte DIG_1 = B00000001;
const byte DIG_2 = B10000000;
const byte DIG_3 = B01000000;
const byte DIG_4 = B00100000;*/

const byte LED_A  = B00000001;
const byte LED_B  = B00000100;
const byte LED_C  = B00010000;
const byte LED_D  = B01000000;
const byte LED_E  = B10000000;
const byte LED_F  = B00000010;
const byte LED_G  = B00001000;
const byte LED_DP = B00100000;
const byte LED_1  = LED_B | LED_C;
const byte LED_2  = LED_A | LED_B | LED_D | LED_E | LED_G;
const byte LED_3  = LED_A | LED_B | LED_C | LED_D | LED_G;
const byte LED_4  = LED_B | LED_C | LED_F | LED_G;
const byte LED_5  = LED_A | LED_C | LED_D | LED_F | LED_G;
const byte LED_6  = LED_A | LED_C | LED_D | LED_E | LED_F | LED_G;
const byte LED_7  = LED_A | LED_B | LED_C;
const byte LED_8  = LED_A | LED_B | LED_C | LED_D | LED_E | LED_F | LED_G;
const byte LED_9  = LED_A | LED_B | LED_C | LED_D | LED_F | LED_G;
const byte LED_0  = LED_A | LED_B | LED_C | LED_D | LED_E | LED_F;
const byte digit[11] = {LED_0, LED_1, LED_2, LED_3, LED_4, 
                        LED_5, LED_6, LED_7, LED_8, LED_9, LED_DP};
volatile byte digits[4] = {0,0,0,0};
unsigned int values[5];// = {120, 1000, 10, 10, 0};

#define NES_A       B00000001
#define NES_B       B00000010
#define NES_SELECT  B00000100
#define NES_START   B00001000
#define NES_UP      B00010000
#define NES_DOWN    B00100000
#define NES_LEFT    B01000000
#define NES_RIGHT   B10000000

unsigned long lastTimePressed = 0;
unsigned long lastTimeSent = 0;
boolean state = false;
volatile boolean finished = true;

enum Modes
{
  P = 0,
  I = 1,
  D = 2,
  X = 3,
  DRIVE = 4 
} modes;

volatile Modes mode = P;

void setup()
{
  pinMode(SR_LATCH,OUTPUT);
  pinMode(SR_CLOCK,OUTPUT);
  pinMode(SR_DATA,OUTPUT);
  
  pinMode(DIG_4, OUTPUT);
  pinMode(DIG_3, OUTPUT);
  pinMode(DIG_2, OUTPUT);
  pinMode(DIG_1, OUTPUT);  
  digitalWrite(DIG_4, HIGH);
  digitalWrite(DIG_3, HIGH);
  digitalWrite(DIG_2, HIGH);
  digitalWrite(DIG_1, HIGH);
  
  pinMode(NES_LATCH,OUTPUT);
  pinMode(NES_CLOCK,OUTPUT);
  pinMode(NES_DATA,INPUT);
  
  digitalWrite(NES_LATCH,HIGH);
  digitalWrite(NES_CLOCK,HIGH);
  
  pinMode(LAMP_1, OUTPUT);
  pinMode(13, OUTPUT);
  digitalWrite(LAMP_1, LOW);
  digitalWrite(13, HIGH);
  
  Serial.begin(9600);
  
  for(int i = 0; i < sizeof(digits); i++)
  {
    digits[i] = LED_0;  
  }

  load(values);  

  // initialize Timer1
  cli();          // disable global interrupts
  TCCR1A = 0;     // set entire TCCR1A register to 0
  TCCR1B = 0;     // same for TCCR1B
  
  // set compare match register to desired timer count:
  // every ~13ms
  // wanted time in s / (16MHz/pre scalar for 16 bit register (65535 values))^-1
  // 0.013 / (1/16*10^/8)
  // OCR1A = 25999;
  OCR1A = 10000;
  
  // turn on CTC mode:
  TCCR1B |= (1 << WGM12);
  
  // Set CS11 for 8 prescaler:
  TCCR1B |= (1 << CS11);

  // enable timer compare interrupt:
  TIMSK1 |= (1 << OCIE1A);
  
  // enable global interrupts:
  sei();
}

ISR(TIMER1_COMPA_vect)
{
  static int digit = 0;
  static int blink = 0;
  static Modes oldMode = mode;
  
  if(!finished)
  {
    PORTB |= (1 << (LAMP_1 - 8));  
  }
  finished = false;
  
  if(blink == 100 || mode != oldMode)
  {
    blink = 0;
    if(oldMode != mode)
    {
      for(int i = 0; i < sizeof(digits); i++)
      {
        digits[i] &= ~LED_DP;  
      }
      oldMode = mode;
    }
    
    if(mode == DRIVE)
    {
      for(int i = 0; i < sizeof(digits); i++)
      {
        digits[i] ^= LED_DP;  
      }
    }    
    else
    {
      digits[mode] ^= LED_DP;
    }
  }
  blink++;
  
  PORTB |= (1 << (DIG_1 - 8));
  PORTD |= (1 << DIG_2);
  PORTD |= (1 << DIG_3);
  PORTD |= (1 << DIG_4);
/*  digitalWrite(DIG_1, HIGH);
  digitalWrite(DIG_2, HIGH);
  digitalWrite(DIG_3, HIGH);
  digitalWrite(DIG_4, HIGH);*/
  registerWrite(0);
  
  switch(digit)
  {
    case 0:
    {
      PORTB &= ~(1 << (DIG_1 - 8));
      //digitalWrite(DIG_1, LOW);
      registerWrite(digits[digit]);
      digit++;
      break;
    }
    case 1:
    {
      PORTD &= ~(1 << DIG_2);
      //digitalWrite(DIG_2, LOW);
      registerWrite(digits[digit]);
      digit++;
      break;
    }
    case 2:
    {
      PORTD &= ~(1 << DIG_3);
      //digitalWrite(DIG_3, LOW);
      registerWrite(digits[digit]);
      digit++;
      break;
    }
    case 3:
    {
      PORTD &= ~(1 << DIG_4);
      //digitalWrite(DIG_4, LOW);
      registerWrite(digits[digit]);
      digit = 0;
      break;  
    }
  }  
  finished = true;
}


void loop()
{
  byte pressed = 0;
  static byte lastButtonPressed = 0;

  if(millis() - lastTimePressed > 100)
  {
    lastButtonPressed = pressed;
    pressed = buttons();
    lastTimePressed = millis();
  }
  else
  {
    delay(millis() - lastTimePressed);  
  } 
  
  if(pressed & NES_SELECT && pressed != lastButtonPressed)
  {    
    if(mode == P) mode = I;
    else if(mode == I) mode = D;
    else if(mode == D) mode = X;
    else if(mode == X) mode = DRIVE;
    else if(mode == DRIVE) mode = P;
  }
  if(mode != DRIVE)
  {
    if(pressed & NES_UP)
    {
      if(values[mode] < 9998)
        values[mode]++;  
    }
    else if(pressed & NES_DOWN)
    {
      if(values[mode] > 0)
        values[mode]--;  
    }
    else if(pressed & NES_RIGHT)
    {
      if(values[mode] < 9990)
        values[mode] += 10;  
    }
    else if(pressed & NES_LEFT)
    {
      if(values[mode] > 9)
        values[mode] -= 10;  
    }
    else if(pressed & NES_START)
    {
      if(millis() - lastTimeSent > 1000)
      {
        sendData();
        lastTimeSent = millis();
      }  
    }
    else if(pressed & NES_A)
    {
      //unsigned int *temp;
      load(values);
    }
    else if(pressed & NES_B)
    {
      save();
    }  
  }
  //DRIVE
  else
  {
    if(pressed & NES_UP)
    {
      values[mode] = 1;
    }
    else if(pressed & NES_DOWN)
    {
      values[mode] = 2;
    }
    else if(pressed & NES_RIGHT)
    {
      values[mode] = 3;
    }
    else if(pressed & NES_LEFT)
    {
      values[mode] = 4;
    }
    
    if(pressed & (NES_UP | NES_DOWN | NES_RIGHT | NES_LEFT))
    {
      sendData();    
    }
  }
  
  display();
}

void load(unsigned int* in)
{
  unsigned char mem[8];
  
  for(int i = 0; i < 8; i++)
  {
    mem[i] = (unsigned char)EEPROM.read(i);
  }
  
  for(int i = 0, j = 0; i < 4; i++, j += 2)
  {
    in[i] = (unsigned int)((mem[j] << 8) | mem[j+1]);
  }
  
  if(sizeof(in)/sizeof(int) == 5)
  {
    //set the direction to a known value
    in[4] = 0;
  }
}

void save()
{
  boolean identical = true;
  unsigned int loaded[4];
  load(loaded);
  for(int i = 0; i < sizeof(loaded)/sizeof(int); i++)
  {
    if(loaded[i] != values[i])
    {
      identical = false;  
    }
  }
  //only save if they are not identical (to save time and cycles)
  if(!identical)
  {    
    byte save[8];
    for(int i = 0, j = 0; i < sizeof(values)/sizeof(int); i++, j += 2)
    {
        save[j]   = values[i] >> 8;
        save[j+1] = values[i];
    }
    for(int i = 0; i < sizeof(save); i++)
    {
      EEPROM.write(i, save[i]);
    }
  }
}

void sendData()
{
  byte string[sizeof(values)+1];
  
  string[0] = mode;

  for(int i = 0; i < sizeof(values)/sizeof(int); i++)
  {
    string[1+(2*i)] = (byte)((values[i] >> 8) & 0xFF);
    string[2+(2*i)] = (byte)(values[i] & 0xFF);
  }
  
  /*for(int i = 0; i < sizeof(string); i++)
  { 
    //Serial.write(string[i], HEX);  
  }*/
  noInterrupts();
  Serial.write(string, sizeof(string));
  interrupts();
}

void display()
{
  int value = values[mode];
  
  if(mode != DRIVE)
  {
    if(value > 999)
    {
      int temp = value/1000;
      digits[0] = digit[temp];  
    }
    else digits[0] = 0;
    
    if(value > 99)
    {
      int temp = value % 1000;
      temp /= 100;
      digits[1] = digit[temp];
    }
    else digits[1] = 0;
  
    if(value > 9)
    {
      int temp = value % 1000;
      temp %=  100;
      temp /= 10;
      digits[2] = digit[temp];
    }
    else digits[2] = 0;
  
    if(value > 0)
    {
      int temp = value % 1000;
      temp %= 100;
      temp %= 10;
      digits[3] = digit[temp];
    }
    else digits[3] = digit[0];  
  }
  else
  {
    digits[0] = LED_E | LED_F | LED_G; 
    digits[1] = LED_G;
    digits[2] = LED_G;
    digits[3] = LED_B | LED_C | LED_G; 
  }
}

void registerWrite(byte value) 
{
  noInterrupts();
  PORTB &= ~(1 << (SR_LATCH-8));
  //digitalWrite(SR_LATCH, LOW); //9
  shiftOut(SR_DATA, SR_CLOCK, LSBFIRST, value);
  PORTB |= (1 << (SR_LATCH-8));
  //digitalWrite(SR_LATCH, HIGH);
  interrupts();
}

byte buttons(void)
{
  noInterrupts();
  byte ret = 0;
  byte i;
  strobe();
  for (i = 0; i < 8; i++) 
  {
    ret |= shiftin() << i;
  }
  
  /*Serial.print(ret, BIN);
  Serial.print(" ");
  Serial.println(byte(~ret), BIN);*/
  interrupts();
  return ~ret;
}

void strobe(void)
{
  PORTD |= (1 << NES_LATCH);
  //digitalWrite(NES_LATCH, HIGH);
  delayMicroseconds(12);
  PORTD &= ~(1 << NES_LATCH);
  //digitalWrite(NES_LATCH, LOW);
}

byte shiftin(void)
{
  byte ret = digitalRead(NES_DATA);
  delayMicroseconds(12);
  PORTD |= (1 << NES_CLOCK);
  //digitalWrite(NES_CLOCK, HIGH);
  delayMicroseconds(12);
  PORTD &= ~(1 << NES_CLOCK);
  //digitalWrite(NES_CLOCK, LOW);
  return ret;
}

