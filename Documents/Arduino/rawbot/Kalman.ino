float kalmanCalculate(float newAngle, float newRate, float looptime)
{
  float dt = looptime/1000;
  float KALMAN_K[2];
  float y, S;

  KALMAN_x_angle += dt * (newRate - KALMAN_x_bias);
  
  KALMAN_P[0][0] += dt * (dt * KALMAN_P[1][1] - KALMAN_P[0][1] - KALMAN_P[1][0] + KALMAN_Q_angle);
  KALMAN_P[0][1] -= dt * KALMAN_P[1][1];
  KALMAN_P[1][0] -= dt * KALMAN_P[1][1];
  KALMAN_P[1][1] += KALMAN_Q_gyro * dt;

  y = newAngle - KALMAN_x_angle;
  S = KALMAN_P[0][0] + KALMAN_R_angle;
  KALMAN_K[0] = KALMAN_P[0][0] / S;
  KALMAN_K[1] = KALMAN_P[1][0] / S;

  KALMAN_x_angle += KALMAN_K[0] * y;
  KALMAN_x_bias  += KALMAN_K[1] * y;
  KALMAN_P[0][0] -= KALMAN_K[0] * KALMAN_P[0][0];
  KALMAN_P[0][1] -= KALMAN_K[0] * KALMAN_P[0][1];
  KALMAN_P[1][0] -= KALMAN_K[1] * KALMAN_P[0][0];
  KALMAN_P[1][1] -= KALMAN_K[1] * KALMAN_P[0][1];

  return KALMAN_x_angle;
}

