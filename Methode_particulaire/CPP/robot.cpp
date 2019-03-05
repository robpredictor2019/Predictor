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
Gx(Mat::zeros(3, 3, CV_64F)),Gx_out(Mat::zeros(3, 3, CV_64F)) ,x_out(Mat::zeros(3, 1, CV_64F)),
t(0),m_ID(0)
{
  x.at<double>(0,0) = 1;
  x.at<double>(1,0) = 1;
  x.at<double>(2,0) = 0;

  u.at<double>(0,0) = 0;

  A.at<double>(0,2) = cos(0);
  A.at<double>(1,2) = sin(0);
  A.at<double>(2,2) = -1;

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


Robot::Robot(int ID)
:x(Mat::zeros(3, 1, CV_64F)), u(Mat::zeros(1, 1, CV_64F)),C(Mat::zeros(2, 3, CV_64F)),
A(Mat::zeros(3, 3, CV_64F)),B(Mat::zeros(3, 1, CV_64F)),
Galpha(Mat::zeros(3, 3, CV_64F)),y(Mat::zeros(2, 1, CV_64F)),Gbeta(Mat::zeros(2, 2, CV_64F)),
Gx(Mat::zeros(3, 3, CV_64F)),Gx_out(Mat::zeros(3, 3, CV_64F)) ,x_out(Mat::zeros(3, 1, CV_64F)),
t(0),m_ID(ID)
{
  x.at<double>(0,0) = 1;
  x.at<double>(1,0) = 1;
  x.at<double>(2,0) = 90;

  u.at<double>(0,0) = 0;

  A.at<double>(0,2) = cos(0);
  A.at<double>(1,2) = sin(0);
  A.at<double>(2,2) = -1;

  B.at<double>(2,0) = 1;

  Galpha.at<double>(0,0) = pow(ID,2);
  Galpha.at<double>(1,1) = pow(1,2);
  Galpha.at<double>(2,2) = pow(1,2);

  Gbeta.at<double>(0,0) = pow(0.1,2);
  Gbeta.at<double>(1,1) = pow(0.1,2);
  Gbeta.at<double>(2,2) = pow(0.1,2);

  Gx.at<double>(0,0) = pow(0.1,2);
  Gx.at<double>(1,1) = pow(0.1,2);
  Gx.at<double>(2,2) = pow(0.1,2);
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
  cout<<"x="<<x.at<double>(0,0)<<"\n";
  cout<<"y="<<x.at<double>(1,0)<<"\n";
  plot->push_back(point(x.at<double>(0,0), x.at<double>(1,0)));
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
  int K = 1;
  double angle, x0, y0, gpsx, gpsy;
  if (t=0){
    x0 = x.at<double>(0);
    y0 = x.at<double>(1);
  }
  if (t = 60){
    angle = 90 + atan((gpsx - x0) / (gpsy - y0));
    x0 = K *(angle - 0);
  }
  else
    y0 = K *(90 - 0);
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
