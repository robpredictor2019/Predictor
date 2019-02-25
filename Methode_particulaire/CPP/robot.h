#include "Kalman.h"
#include <opencv2/core/core.hpp>

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


public:

  Robot(); // Constructeur par defaut
  Robot(float x,float y, float theta);// Constructeur
  void Show() const; // Affichage
  //Methodes
  void evolution();
  void draw(Gnuplot gp);
  float scenario();
  void kalman_predict(Mat xup_k,Mat Pup_k, Mat Q, Mat A, Mat u, Mat* x_k1, Mat* P_k1);
  void kalman_correct(Mat x_k1, Mat P_k1, Mat  C, Mat R, Mat y, Mat* xup_k1, Mat* Pup_k1);
  void kalman_x(Mat x_k,Mat P_k,Mat u,Mat y,Mat Q, Mat R,Mat A ,Mat C, Mat* P_k1, Mat* x_k1);
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
  //variables kalman
  Mat x;
  Mat u;
  Mat C;
  Mat A;
  Mat Gbeta;
  Mat Galpha;
  Mat y;
  Mat Xout, Galpha_out;
  double hm;

  double m_hm[1] ;
  Mat hm_mat;

};
