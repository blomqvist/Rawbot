#ifndef __PID_h
#define __PID_h

#include "Arduino.h"

class PID
{
    public:
        PID(float KP, float KI, float KD, float wantedAnglei, float freq);
        void  SetPidConstants(float KP, float KI, float KD);
        void  SetWantedAngle(float wantedAngle);
        float Calculate_original(float e);
        float Calculate_wiki(float e);
    private:
        float _KP, _KI, _KD;
        float _wantedAngle;
        float _lastError, _lastError_w;
        float _lastIntegral;
        float _integrationLimit;
        float _dt;
};

#endif 
