//-----------------------------------------------------------------------------------------------
//---------------------------------------COMMENTAIRES--------------------------------------------
//
//-----------------------------------------------------------------------------------------------
//-----------------------------------------INCLUDES----------------------------------------------
#include "iostream"
#include <vector>
#include <iterator>
//#include "MBUtils.h"
#include <math.h>
#include "robot.h"
#include <random>
#include <cmath>

using namespace std;
using namespace cv;

//-----------------------------------------------------------------------------------------------
//-----------------------------------------FONCTIONS---------------------------------------------


Robot::Robot()
:x(Mat::zeros(3, 1, CV_64F)), u(Mat::zeros(1, 1, CV_64F)),C(Mat::zeros(2, 3, CV_64F)),
A(Mat::zeros(3, 3, CV_64F)),B(Mat::zeros(3, 1, CV_64F)),
Galpha(Mat::zeros(3, 3, CV_64F)),y(Mat::zeros(2, 1, CV_64F)),Gbeta(Mat::zeros(2, 2, CV_64F)),
Gx(Mat::zeros(3, 3, CV_64F)),Gx_out(Mat::zeros(3, 3, CV_64F)) ,x_out(Mat::zeros(3, 1, CV_64F)),
t(0),m_ID(0),theta_bar(0),v(1),theta(0)
{
  x.at<double>(0,0) = 0;
  x.at<double>(1,0) = 0;
  x.at<double>(2,0) = 1;

  u.at<double>(0,0) = 0;

  A.at<double>(0,2) = v*cos(theta_bar*PI/180);
  A.at<double>(1,2) = v*sin(theta_bar*PI/180);
  A.at<double>(2,2) = -v;

  B.at<double>(2,0) = 1;

  Galpha.at<double>(0,0) = pow(0,2);
  Galpha.at<double>(1,1) = pow(1,2);
  Galpha.at<double>(2,2) = pow(1,2);

  Gbeta.at<double>(0,0) = pow(0.45,2);
  Gbeta.at<double>(1,1) = pow(0.45,2);
  Gbeta.at<double>(2,2) = pow(0.45,2);

  Gx.at<double>(0,0) = pow(0.1,2);
  Gx.at<double>(1,1) = pow(0.1,2);
  Gx.at<double>(2,2) = pow(0.1,2);
}


Robot::Robot(int ID,double dt)
:x(Mat::zeros(3, 1, CV_64F)), u(Mat::zeros(1, 1, CV_64F)),C(Mat::zeros(2, 3, CV_64F)),
A(Mat::zeros(3, 3, CV_64F)),B(Mat::zeros(3, 1, CV_64F)),
Galpha(Mat::zeros(3, 3, CV_64F)),y(Mat::zeros(2, 1, CV_64F)),Gbeta(Mat::zeros(2, 2, CV_64F)),
Gx(Mat::zeros(3, 3, CV_64F)),Gx_out(Mat::zeros(3, 3, CV_64F)) ,x_out(Mat::zeros(3, 1, CV_64F)),
t(0),m_ID(ID),dt(dt),theta_bar(0),v(1),theta(0)
{
  x.at<double>(0,0) = 0;
  x.at<double>(1,0) = 0;
  x.at<double>(2,0) = 1;

  u.at<double>(0,0) = 1;

  A.at<double>(0,2) = v*cos(theta*PI/180);
  A.at<double>(1,2) = v*sin(theta*PI/180);
  A.at<double>(2,2) = -v;

  B.at<double>(2,0) = 1;

  Galpha.at<double>(0,0) = pow(1,2);
  Galpha.at<double>(1,1) = pow(1,2);
  Galpha.at<double>(2,2) = pow(1,2);

  Gbeta.at<double>(0,0) = pow(0.1,2);
  Gbeta.at<double>(1,1) = pow(0.1,2);

  Gx.at<double>(0,0) = pow(0.1,2);
  Gx.at<double>(1,1) = pow(0.1,2);
  Gx.at<double>(2,2) = pow(0.1,2);
}

void Robot::evolution()
{
  default_random_engine generator;
  normal_distribution<double> dx(0,Galpha.at<double>(0,0));
  normal_distribution<> dy(0,Galpha.at<double>(1,1));
  normal_distribution<> dv(0,Galpha.at<double>(2,2));
  Mat xdot = Mat::zeros(3, 1, CV_64F);
  double thetadot;


  xdot.at<double>(0) = x.at<double>(2)*cos((theta*PI/180));// + dx(generator);
  xdot.at<double>(1) = x.at<double>(2)*sin((theta*PI/180));// + dy(generator);
  xdot.at<double>(2) = u.at<double>(0) - x.at<double>(2);// + dv(generator);
  thetadot = max( min(10.0,3*(theta_bar-theta)),-10.0 );
  cout<<"thetadot ="<<thetadot<<endl;
  //cout<<Gx.at<double>(2,2)<<endl;
  x = x + dt*xdot;
  theta = theta + dt*thetadot;
  cout<<"theta ="<<theta<<endl;
  t+=dt;
}

void Robot::kalman_predict(Mat xup_k,Mat Pup_k, Mat* x_k1, Mat* P_k1)
{
  *P_k1 = (A*Pup_k*A.t()) + Galpha;
  *x_k1 = A*xup_k + B*u;
}

void Robot::kalman_correct( Mat* xup_k1, Mat* Pup_k1)
{
   Mat S = Mat::zeros(Size(2,2),CV_64F);
   Mat K = Mat::zeros(Size(3,2),CV_64F);
   Mat err = Mat::zeros(2,1,CV_64F);

   S = (C*Gx*C.t()) + Gbeta;
   K = Gx*C.t()*S.inv();
   err = y - (C*x);
   *Pup_k1 = (Mat::eye(3,3,CV_64F) - (K*C)) * Gx;
   *xup_k1 = x + K*err;
}

void Robot::kalman_x( Mat* P_k1, Mat* x_k1)
{
  Mat Pup_k = Mat::zeros(Size(3,3),CV_64F);
  Mat xup_k = Mat::zeros(Size(3,1),CV_64F);

  kalman_correct( &xup_k, &Pup_k);
  kalman_predict( xup_k, Pup_k, x_k1, P_k1);
}


void Robot::draw(vector<point> *plot)
{
    double norm = 0;
    for (int i=0;i<3;i++)
    {
        for (int j=0;j<3;j++)
        {
            norm += pow(Gx_out.at<double>(i,j),2);
        }
    }
    norm = pow(norm,0.5);
    //cout<<norm<<endl;
    plot->push_back(point(t,norm));
}

void Robot::draw_x_y(vector<point>*plot)
{
  //cout<<"x="<<x.at<double>(0,0)<<"\n";
  //cout<<"y="<<x.at<double>(1,0)<<"\n";
  plot->push_back(point(x.at<double>(0,0), x.at<double>(1,0)));
}

vector<point> Robot::draw_x_y()
{
  vector<point> plot;
  //cout<<"x="<<x.at<double>(0,0)<<"\n";
  //cout<<"y="<<x.at<double>(1,0)<<"\n";
  plot.push_back(point(x.at<double>(0,0), x.at<double>(1,0)));
  return plot;
}


void Robot::save_state()
{
    State s;
    s.ID = m_ID;
    s.t  = t;
    s.x  = x.at<double>(0);
    s.y  = x.at<double>(1);
    s.theta = x.at<double>(2);
    m_state.push_back(s);
}

void Robot::P_theta()
{
  double x0(0);
  double y0(0);
}

void Robot::Export(ofstream & fs)
{
    State s;
    for (int i=0;i<m_state.size();i++)
    {
        s = m_state[i];
        fs << s.ID<<";"<<s.t<<";"<<s.x<<";"<<s.y<<";0;0;0;"<<s.theta<<std::endl;
    }
}
