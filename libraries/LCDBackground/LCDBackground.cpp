#include "Arduino.h"
#include "LCDBackground.h"

const int LCDBackground::colorArray[4][3] = 
{
    {255, 255, 0},   //yellow 
    {255, 0,   0},   //red
    {0,   255, 0},   //green
    {0,   0,   255}  //blue
};

LCDBackground::LCDBackground(const int pinR, const int pinG, const int pinB)
{
    _pinR = pinR;
    _pinG = pinG;
    _pinB = pinB;
    
    pinMode(_pinR, OUTPUT);
    pinMode(_pinG, OUTPUT);
    pinMode(_pinB, OUTPUT);


    //Always start with yellow
    color = YELLOW;
    LCDWrite(colorArray[color][0],
             colorArray[color][1],
             colorArray[color][2]);
}

void LCDBackground::Tick()
{
    if(colorArray[color][0] == R &&
       colorArray[color][1] == G &&
       colorArray[color][2] == B)
    {
        color = random(1, 4);
    }
/*
    R < colorArray[color][0] ? R++ : R--;
    G < colorArray[color][1] ? G++ : G--;
    B < colorArray[color][2] ? B++ : B--;
  */  
    if(color == YELLOW)
    {
        (R < 255) ? R++ : R = 255;
        (G < 255) ? G++ : G = 255;
        (B > 0  ) ? B-- : B = 0;
    }
    else if(color == RED)
    {
        (R < 255) ? R++ : R = 255;
        (G > 0  ) ? G-- : G = 0;
        (B > 0  ) ? B-- : B = 0;
    }
    else if(color == GREEN)
    {
        (R > 0  ) ? R-- : R = 0;
        (G < 255) ? G++ : G = 255;
        (B > 0  ) ? B-- : B = 0;
    }
    else if(color == BLUE)
    {
        (R > 0  ) ? R-- : R = 0;
        (G > 0  ) ? G-- : G = 0;
        (B < 255) ? B++ : B = 255;
    }

    LCDWrite(R, G, B);
}

void LCDBackground::LCDStop()
{
    static bool red = true;
    int G = 0, B = 0;

    if(red)
    {
        G = 255;
        B = 255;
    }
    red = !red;
    LCDWrite(255, G, B);
}

void LCDBackground::LCDWrite(const int R, const int G, const int B)
{
    analogWrite(_pinR, R);
    analogWrite(_pinG, G);
    analogWrite(_pinB, B);
}
