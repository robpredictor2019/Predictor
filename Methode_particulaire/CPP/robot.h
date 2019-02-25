#include "Kalman.h"

class Robot
{
private:
  float m_x;
  float m_y;
  float m_theta;
  float m_u;
  Kalman m_kalman;

public:
  void evolution();
  void draw();

};
