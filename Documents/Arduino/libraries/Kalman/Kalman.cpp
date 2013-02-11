#include "Arduino.h"
#include "Kalman.h"

float Kalman::_P[2][2] = {{0 ,0}, {0, 0}};
//const float Kalman::_K_Q_angle = 0.001;
//const float Kalman::_K_Q_gyro  = 0.003;
//const float Kalman::_K_R_angle = 0.03;

Kalman::Kalman()
{
    K_bias = 0;
    _K_Q_angle = 0.001;
    _K_Q_gyro  = 0.003;
    _K_R_angle = 0.03;
}

Kalman::Kalman(int looptime, float startAngle)
{
    K_bias = 0;
    _dt = looptime/1000;
    _K_angle = startAngle;
    _K_Q_angle = 0.001;
    _K_Q_gyro  = 0.003;
    _K_R_angle = 0.03;
}

void Kalman::SetFrequency(int looptime)
{
    _dt = float(looptime)/1000;
}

float Kalman::GetFrequency()
{
    return _dt;
}

void Kalman::SetStartAngle(float startAngle)
{
    _K_angle = startAngle;
}

float Kalman::Kalkulate(float newAngle, float newRate)
{
    float K[2];
    float y, S;

    _K_angle += _dt * (newRate - K_bias);

    _P[0][0] += _dt * (_dt * _P[1][1] - _P[0][1] - _P[1][0] + _K_Q_angle);
    _P[0][1] -= _dt * _P[1][1];
    _P[1][0] -= _dt * _P[1][1];
    _P[1][1] += _K_Q_gyro * _dt;

    y = newAngle - _K_angle;
    S = _P[0][0] + _K_R_angle;
    K[0] = _P[0][0] / S;
    K[1] = _P[1][0] / S;

    _K_angle  += K[0] * y;
    K_bias    += K[1] * y;
    _P[0][0]  -= K[0] * _P[0][0];
    _P[0][1]  -= K[0] * _P[0][1];
    _P[1][0]  -= K[1] * _P[0][0];
    _P[1][1]  -= K[1] * _P[0][1];

    return _K_angle;
}

