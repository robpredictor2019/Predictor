
#include <opencv2/core/core.hpp>
#include <fstream>
#include <vector>
#include "gnuplot-iostream.h"
#include <boost/tuple/tuple.hpp>
#include <math.h>
#include <random>

#define PI 3.141592653

struct State{
  int ID;
  double t;
  double x;
  double y;
  double theta;
}typedef State;

typedef std::pair<double, double> point;


class Robot
{
private:
  double dt;
  int m_ID;
  std::vector<State> m_state;
  double Kp;

public:
  double t;
  double theta;
  double theta_bar;
  double theta_dot;
  int v;
  double theta_mission;
  //Variables Kalman
  cv::Mat x;
  cv::Mat Gx;
  cv::Mat u;
  cv::Mat C;
  cv::Mat A;
  cv::Mat B;
  cv::Mat Gbeta;
  cv::Mat Galpha;
  cv::Mat y;
  cv::Mat x_hat, Gx_hat;

  Robot(); // Constructeur par defaut
  //~Robot(); // Destructeur
  void P_theta(); //Porportionnel pour theta
  Robot(int,double);

  Robot(cv::Mat x, cv::Mat u, cv::Mat C, cv::Mat A, cv::Mat Galpha, cv::Mat y, cv::Mat Gbeta, cv::Mat Gx);// Constructeur
  void Show() const; // Affichage
  //Methodes
  void evolution();
  void draw(std::vector<point>*);
  void draw_x_y(std::vector<point>*);
  void draw_x_y_hat(std::vector<point>*);
  std::vector<point> draw_x_y();
  float scenario();
  double distance(point p);

  void save_state();
  void Export(std::ofstream & fs);

  void kalman_predict( cv::Mat& xup_k, cv::Mat& Pup_k, cv::Mat* x_k1, cv::Mat* P_k1);
  void kalman_correct( cv::Mat* xup_k1, cv::Mat* Pup_k1);
  void kalman_x( cv::Mat* P_k1, cv::Mat* x_k1);

};

std::vector<point> circle(double x,double y,double r);
std::vector<point> circle(point p,double r);
