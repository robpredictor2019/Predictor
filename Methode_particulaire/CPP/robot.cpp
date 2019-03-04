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

using namespace std;
using namespace cv;

//-----------------------------------------------------------------------------------------------
//-----------------------------------------FONCTIONS---------------------------------------------

Robot::Robot()
:x(Mat::zeros(3, 1, CV_64F)), u(Mat::zeros(1, 1, CV_64F)),C(Mat::zeros(2, 3, CV_64F)),
A(Mat::zeros(3, 3, CV_64F)),B(Mat::zeros(3, 1, CV_64F)),
Galpha(Mat::zeros(3, 3, CV_64F)),y(Mat::zeros(2, 1, CV_64F)),Gbeta(Mat::zeros(2, 2, CV_64F)),
Gx(Mat::zeros(3, 3, CV_64F)),Gx_out(Mat::zeros(3, 3, CV_64F)) ,x_out(Mat::zeros(3, 1, CV_64F))
{  x.at<double>(0,0) = 1;
  x.at<double>(1,0) = 1;
  x.at<double>(2,0) = 0;

  u.at<double>(0,0) = 0;

  A.at<double>(0,2) = cos(0);
  A.at<double>(1,2) = sin(0);
  A.at<double>(2,2) = -1;

  B.at<double>(2,0) = 1;

  Galpha.at<double>(0,0) = pow(0.1,2);
  Galpha.at<double>(1,1) = pow(0.1,2);
  Galpha.at<double>(2,2) = pow(0.1,2);

  Gbeta.at<double>(0,0) = pow(0.1,2);
  Gbeta.at<double>(1,1) = pow(0.1,2);
  Gbeta.at<double>(2,2) = pow(0.1,2);

  Gx.at<double>(0,0) = pow(0.1,2);
  Gx.at<double>(1,1) = pow(0.1,2);
  Gx.at<double>(2,2) = pow(0.1,2);
}

Robot::~Robot()
{
  m_state.resize(0);
  plot.resize(0);
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


void Robot::draw(Gnuplot &gp)
{
    double norm = 0;
    for (int i=0;i<3;i++)
    {
        for (int j=0;j<3;j++)
        {
            norm += Gx_out.at<double>(i,j);
        }
    }
    plot.push_back(point(m_t,norm));
    gp << "plot '-'\n";
    gp.send1d(plot);
}

void Robot::save_state()
{
    State s;
    s.ID = m_ID;
    s.t  = m_t;
    s.x  = x.at<double>(0);
    s.y  = x.at<double>(1);
    s.theta = x.at<double>(2);
    m_state.push_back(s);
}

void Robot::Export(ofstream & fs)
{
    State s;
    for (int i=0;i<m_state.size();i++)
    {
        s = m_state[i];
        fs << s.ID<<";"<<s.t<<";"<<s.x<<";"<<s.y<<";0;0;0"<<s.theta<<std::endl;
    }
}
