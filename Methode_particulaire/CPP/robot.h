#include "Kalman.h"
#include <vector>
#include "gnuplot-iostream.h"

typedef std::pair<double, double> Point;

class Robot
{
private:
  float m_x;
  float m_y;
  float m_theta;
  float m_u;
  Kalman m_kalman;
  int m_number;
  std::vector<Point> plot;



  Robot(); // Constructeur par defaut
  Robot(float x,float y, float theta);// Constructeur
  void Show() const; // Affichage
  //Methodes
  void evolution();
  void draw(Gnuplot gp);
  float scenario();
  //setter
  void setX(float x);
  void setY(float y);
  void setTheta(float theta);
  void setU(float u);
  //getter
  int getX() const;
  int getY() const;
  int getTheta() const;
  int getU() const;
};
