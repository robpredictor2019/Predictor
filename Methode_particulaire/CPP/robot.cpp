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
Gx_hat(Mat::zeros(3, 3, CV_64F)) ,x_hat(Mat::zeros(3, 1, CV_64F)),
t(0),m_ID(0),dt(0.1),theta_bar(0),v(1),theta(0),theta_dot(0),Kp(1),theta_mission(0)
{
  x.at<double>(0,0) = 0;
  x.at<double>(1,0) = 0;
  x.at<double>(2,0) = 1;

  x_hat.at<double>(0,0) = 0;
  x_hat.at<double>(1,0) = 0;
  x_hat.at<double>(2,0) = 1;


  u.at<double>(0,0) = 1;

  A.at<double>(0,2) = cos(theta*PI/180);
  A.at<double>(1,2) = sin(theta*PI/180);
  A.at<double>(2,2) = -1;

  B.at<double>(2,2) = 1;

  Galpha.at<double>(0,0) = pow(1,2);
  Galpha.at<double>(1,1) = pow(1,2);
  Galpha.at<double>(2,2) = pow(1,2);

  Gbeta.at<double>(0,0) = pow(0.1,2);
  Gbeta.at<double>(1,1) = pow(0.1,2);

  Gx_hat.at<double>(0,0) = pow(0.1,2);
  Gx_hat.at<double>(1,1) = pow(0.1,2);
  Gx_hat.at<double>(2,2) = pow(0.1,2);
}


Robot::Robot(int ID,double dt)
:x(Mat::zeros(3, 1, CV_64F)), u(Mat::zeros(1, 1, CV_64F)),C(Mat::zeros(2, 3, CV_64F)),
A(Mat::zeros(3, 3, CV_64F)),B(Mat::zeros(3, 1, CV_64F)),
Galpha(Mat::zeros(3, 3, CV_64F)),y(Mat::zeros(2, 1, CV_64F)),Gbeta(Mat::zeros(2, 2, CV_64F)),
Gx_hat(Mat::zeros(3, 3, CV_64F)) ,x_hat(Mat::zeros(3, 1, CV_64F)),
t(0),m_ID(ID),dt(dt),theta_bar(0),v(1),theta(0),theta_dot(0),Kp(1),theta_mission(0)
{
  x.at<double>(0,0) = 0;
  x.at<double>(1,0) = 0;
  x.at<double>(2,0) = 1;

  x_hat.at<double>(0,0) = 0;
  x_hat.at<double>(1,0) = 0;
  x_hat.at<double>(2,0) = 1;


  u.at<double>(0,0) = 1;

  A.at<double>(0,0) = 1;
  A.at<double>(0,2) = cos(theta*PI/180)*dt;
  A.at<double>(1,1) = 1;
  A.at<double>(1,2) = sin(theta*PI/180)*dt;
  A.at<double>(2,2) = 1*dt+1;

  B.at<double>(2,0) = 1;

  Galpha.at<double>(0,0) = pow(1,2);
  Galpha.at<double>(1,1) = pow(1,2);
  Galpha.at<double>(2,2) = pow(1,2);

  Gbeta.at<double>(0,0) = 0;
  Gbeta.at<double>(1,1) = 0;

  Gx_hat.at<double>(0,0) = pow(0.1,2);
  Gx_hat.at<double>(1,1) = pow(0.1,2);
  Gx_hat.at<double>(2,2) = pow(0.1,2);
}

void Robot::evolution()
{
  random_device generator;
  normal_distribution<double> dx(0,Galpha.at<double>(0,0));
  normal_distribution<> dy(0,Galpha.at<double>(1,1));
  normal_distribution<> dv(0,Galpha.at<double>(2,2));
  Mat xdot = Mat::zeros(3, 1, CV_64F);




  xdot.at<double>(0) = x.at<double>(2)*cos((theta*PI/180)) + dx(generator);
  xdot.at<double>(1) = x.at<double>(2)*sin((theta*PI/180)) + dy(generator);
  xdot.at<double>(2) = u.at<double>(0) - x.at<double>(2) + dv(generator);
  //cout<<"thetadot ="<<theta_dot<<endl;
  //cout<<dx(generator)<<endl;
  x += dt*xdot;
  theta += dt*theta_dot;
  if (theta>360)
    theta -= 360;
  //Update A
  A.at<double>(0,2) = cos(theta*PI/180)*dt;
  A.at<double>(1,2) = sin(theta*PI/180)*dt;
  A.at<double>(2,2) = -dt+1;

  //theta += dtheta(generator)

  //cout<<"v ="<<v<<endl;

  t+=dt;
}

void Robot::kalman_predict(Mat & xup_k,Mat& Pup_k, Mat* x_k1, Mat* P_k1)
{
  *P_k1 = (A*Pup_k*A.t()) + Galpha;
  *x_k1 = A*xup_k + B*u;
}

void Robot::kalman_correct( Mat* xup_k1, Mat* Pup_k1)
{
   Mat S = Mat::zeros(Size(2,2),CV_64F);
   Mat K = Mat::zeros(Size(3,2),CV_64F);
   Mat err = Mat::zeros(2,1,CV_64F);

   S = (C*Gx_hat*C.t()) + Gbeta;
   K = Gx_hat*C.t()*S.inv();
   err = y - (C*x);
   *Pup_k1 = (Mat::eye(3,3,CV_64F) - (K*C)) * Gx_hat;
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
            norm += pow(Gx_hat.at<double>(i,j),2);
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

void Robot::draw_x_y_hat(vector<point>*plot)
{
  //cout<<"x="<<x.at<double>(0,0)<<"\n";
  //cout<<"y="<<x.at<double>(1,0)<<"\n";
  plot->push_back(point(x_hat.at<double>(0), x_hat.at<double>(1)));
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
    s.x  = x_hat.at<double>(0);
    s.y  = x_hat.at<double>(1);
    s.theta = theta;
    m_state.push_back(s);
}

void Robot::P_theta()
{

  //theta_dot = K*(theta_bar - theta);
  //if (t>60){
    theta_bar = atan2(x_hat.at<double>(1),x_hat.at<double>(0))*180/PI + theta_mission;
    theta_dot = Kp*(theta_bar - theta);
    theta_dot = max( min(10.0,theta_dot),-10.0);
    //cout<<"x_hat"<<x_hat.at<double>(0)<<x_hat.at<double>(1)<<x_hat.at<double>(2)<<endl;
  //}
  cout<<"theta_bar "<<theta_bar<<endl;
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
