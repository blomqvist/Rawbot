#ifndef __KALMAN_h
#define __KALMAN_h

#include "Arduino.h"

class Kalman
{
    public:
        Kalman(void);
        Kalman(int looptime, float startAngle);
        void SetFrequency(int looptime);
        float GetFrequency();
        void SetStartAngle(float startAngle);
        float Kalkulate(float newAngle, float newRate);
        float K_bias;
    private:
        float _dt;
        static float _P[2][2];
        float _K_angle;
        
        //static const float _K_Q_angle;
        //static const float _K_Q_gyro;
        //static const float _K_Q_angle;
        float _K_Q_angle;
        float _K_Q_gyro;
        float _K_R_angle;
};

#endif
