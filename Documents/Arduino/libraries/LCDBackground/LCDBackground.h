#ifndef __LCDBACKGROUND_h
#define __LCDBACKGROUND_h

#include "Arduino.h"

class LCDBackground
{
    public:
        LCDBackground(const int pinR, const int pinG, const int pinB);
        void Tick();
        void LCDStop();
    private:
        void LCDWrite(const int R, const int G, const int B);
        uint8_t _pinR, _pinG, _pinB;
        enum colors {YELLOW, RED, GREEN, BLUE};
        uint8_t color;        
        uint8_t R, G, B;
        static const int colorArray[4][3]; 
};


#endif
