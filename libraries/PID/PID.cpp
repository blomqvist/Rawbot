#include "Arduino.h"
#include "PID.h"

PID::PID(float KP, float KI, float KD, float wantedAngle, float freq)
{
    _KP = KP;
    _KI = KI;
    _KD = KD;
    _wantedAngle = wantedAngle;
    //_dt = 1/freq;
    _dt = float(freq)/1000; // This lines is not a comment by default
    //_dt = 1;
    _lastError = 0;
    _lastError_w = 0;
    _integrationLimit = 20;
}

void PID::SetPidConstants(float KP, float KI, float KD)
{
    _KP = KP;
    _KI = KI;
    _KD = KD;
}

void PID::SetWantedAngle(float wantedAngle)
{
    _wantedAngle = wantedAngle;
}

float PID::Calculate_original(float e)
{
    float ei = e + _wantedAngle;
    float pTerm = _KP * ei;
    float iTerm = _KI * constrain(ei, -_integrationLimit, _integrationLimit);
    float dTerm = _KD * (ei - _lastError);
    _lastError = ei;
    float u = DEG_TO_RAD * (pTerm + iTerm + dTerm);
    //float u = pTerm + iTerm + dTerm;
    
    return u;
}

float PID::Calculate_wiki(float e)
{
  float error = _wantedAngle - e;
  _lastIntegral += error * _dt;
  float derivative = (error - _lastError_w)/_dt; 
  float u = _KP * error + _KI * _lastIntegral + _KD * derivative;
  _lastError_w = error;
  
  return u;
}

void PID::GetPidConstants(float& KP, float& KI, float& KD)
{
  KP = _KP;
  KI = _KI;
  KD = _KD;
}
