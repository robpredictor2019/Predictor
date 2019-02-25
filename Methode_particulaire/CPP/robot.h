#include "Kalman.h"
#include <opencv2/core/core.hpp>
#include <fstream>
#include <vector>
#include "gnuplot-iostream.h"


struct State{
  int ID,
  double t,
  double x,
  double y,
  double theta
}typedef State;

typedef std::pair<double, double> Point;

class Robot
{
private:
  float m_x;
  float m_y;
  float m_theta;
  float m_u;
  Kalman m_kalman;
  int m_ID;
  std::vector<Point> plot;
  std::vector<State> m_state;
  double m_t;


public:
  Robot(); // Constructeur par defaut
  Robot(float x,float y, float theta);// Constructeur
  void Show() const; // Affichage
  //Methodes
  void evolution();
  void draw(Gnuplot gp);
  float scenario();
  void save_state();
  void export(std::fstream fs);
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
  double dt;
  double h0;
  double d0;
  Mat x;
  Mat u;
  Mat phat;
  Mat C;
  Mat dx;
  Mat A;
  Mat theta_p;
  Mat theta_beta;
  Mat theta_alpha;
  Mat y;
  Mat phat_out, theta_p_out;
  double hm;
  bool depth_new;
  bool alt_new;
  int size_x;
  int size_u;
  int size_y;

  double cap_command;
  int count_command_test;
  double count_mag;
  double command_test ;

  double m_hm[1] ;
  Mat hm_mat;

};
