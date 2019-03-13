//-----------------------------------------------------------------------------------------------
//---------------------------------------COMMENTAIRES--------------------------------------------
//
//-----------------------------------------------------------------------------------------------
//-----------------------------------------INCLUDES----------------------------------------------
#include "iostream"
#include <vector>
#include <iterator>
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

  Gbeta.at<double>(0,0) = 0;
  Gbeta.at<double>(1,1) = 0;

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
  A.at<double>(2,2) = 1-dt;

  B.at<double>(2,0) = dt;

  Galpha.at<double>(0,0) = dt*pow(0.1,2);
  Galpha.at<double>(1,1) = dt*pow(0.1,2);
  Galpha.at<double>(2,2) = dt*pow(0.15,2);

  Gbeta.at<double>(0,0) = 0;
  Gbeta.at<double>(1,1) = 0;

  Gx_hat.at<double>(0,0) = pow(0,2);
  Gx_hat.at<double>(1,1) = pow(0,2);
  Gx_hat.at<double>(2,2) = pow(0,2);
}

void Robot::evolution()
{
  random_device generator;
  normal_distribution<double> dx(0,pow(Galpha.at<double>(0,0),0.5));
  normal_distribution<> dy(0,pow(Galpha.at<double>(1,1),0.5));
  normal_distribution<> dv(0,pow(Galpha.at<double>(2,2),0.5));
  Mat xdot = Mat::zeros(3, 1, CV_64F);

  Mat Bruit = Mat::zeros(3, 1, CV_64F);

  Bruit.at<double>(0) = dx(generator);
  Bruit.at<double>(1) = dy(generator);
  Bruit.at<double>(2) = dv(generator);

  xdot.at<double>(0) = x.at<double>(2)*cos((theta*PI/180));
  xdot.at<double>(1) = x.at<double>(2)*sin((theta*PI/180));
  xdot.at<double>(2) = u.at<double>(0) - x.at<double>(2);

  x += dt*xdot + Bruit;
  theta += dt*theta_dot;
  if (theta>360)
    theta -= 360;
  //Update A
  A.at<double>(0,2) = cos(theta*PI/180)*dt;
  A.at<double>(1,2) = sin(theta*PI/180)*dt;

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
   err = y - (C*x_hat);
   *Pup_k1 = (Mat::eye(3,3,CV_64F) - (K*C)) * Gx_hat;
   *xup_k1 = x_hat + K*err;
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
    for (int i=0;i<2;i++)
    {
        //for (int j=0;j<3;j++)
        //{
            norm += pow(Gx_hat.at<double>(i,i),2);
        //}
    }
    norm = pow(norm,0.5);
    plot->push_back(point(t,norm));
}

void Robot::draw_x_y(vector<point>*plot)
{
  plot->push_back(point(x.at<double>(0,0), x.at<double>(1,0)));
}

void Robot::draw_x_y_hat(vector<point>*plot)
{
  plot->push_back(point(x_hat.at<double>(0), x_hat.at<double>(1)));
}

vector<point> Robot::draw_x_y()
{
  vector<point> plot;
  plot.push_back(point(x.at<double>(0,0), x.at<double>(1,0)));
  return plot;
}

vector<point> Robot::draw_x_y_hat()
{
  vector<point> plot;
  plot.push_back(point(x_hat.at<double>(0,0), x_hat.at<double>(1,0)));
  return plot;
}

double Robot::distance(point p){
  double dist;
  dist = pow( pow(x_hat.at<double>(0)-p.first,2) + pow(x_hat.at<double>(1)-p.second,2) ,0.5);
  return dist;
}


void Robot::save_state()
{
    State s;
    s.ID = m_ID;
    s.t  = t;
    s.x  = x.at<double>(0);
    s.y  = x.at<double>(1);
    s.theta = theta;
    m_state.push_back(s);
}

void Robot::P_theta(point p)
{

  theta_bar = atan2(p.second-x_hat.at<double>(1),p.first-x_hat.at<double>(0))*180/PI;
  theta_dot = Kp*(theta_bar - theta);
  theta_dot = /*0.05**/2*atan(tan(theta_dot*PI/360)) * 180/PI;
  theta_dot = max( min(10.0,theta_dot),-10.0);
  //cout<<"theta_bar "<<theta_bar<<endl;
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


vector<point> circle(double x,double y,double r){
  vector<point> data;
  double xc;
  double yc;
  for (int i = 0;i<360;i++){
    xc = r*cos(i*PI/180) + x;
    yc = r*sin(i*PI/180) + y;
    data.push_back(point(xc,yc));
  }
  return data;
}

vector<point> circle(point p,double r){
  vector<point> data = circle(p.first,p.second,r);
  return data;
}

void  draw_ellipse(double x,double y,Mat Gx){
    Mat A /*= cv::sqrt(-2*log(1)*Gx)*/;
    sqrt(2*Gx,A);
    Mat w;
    //eigen(A,w);
    //std::cout<<w.size()<<endl;
    /*v1=array([[v[0,0]],[v[1,0]]])
    v2=array([[v[0,1]],[v[1,1]]])
    f1=A @ v1
    f2=A @ v2
    φ =  (arctan2(v1 [1,0],v1[0,0]))
    α=φ*180/3.14
    e = Ellipse(xy=c, width=2*norm(f1), height=2*norm(f2), angle=α)
    ax.add_artist(e)
    e.set_clip_box(ax.bbox)
    e.set_alpha(0.7)
    e.set_facecolor(col)*/
}

void covar_particule(vector<point> * plot_x,vector<point> * plot_y,vector<point> * plot_xy,vector<point> * plot_dist,vector<Robot> & Lr){
  for (int i=0;i<Lr[0].m_state.capacity();i++){
    double x_mean(0),y_mean(0),x_sigma(0),y_sigma(0);
    for(int j=0;j<Lr.capacity();j++){
      x_mean += Lr[j].m_state[i].x;
      y_mean += Lr[j].m_state[i].y;
    }

    x_mean = x_mean/Lr.capacity();
    y_mean = y_mean/Lr.capacity();



    for(int j=0;j<Lr.capacity();j++){
      x_sigma += pow(Lr[j].m_state[i].x - x_mean,2);
      y_sigma += pow(Lr[j].m_state[i].y - y_mean,2);
    }
    x_sigma = pow(x_sigma/Lr.capacity(),0.5);
    y_sigma = pow(y_sigma/Lr.capacity(),0.5);
    double dist_max(0),dist(0);

    for(int j=0;j<Lr.capacity();j++){
      dist = pow(Lr[j].m_state[i].x - x_mean,2) + pow(Lr[j].m_state[i].y - y_mean,2);
      if (dist>dist_max){
        dist_max = dist;
      }
    }
    plot_x ->push_back(point(Lr[0].m_state[i].t,x_sigma));
    plot_y ->push_back(point(Lr[0].m_state[i].t,y_sigma));
    plot_xy->push_back(point(Lr[0].m_state[i].t,x_sigma + y_sigma));
    plot_dist ->push_back(point(Lr[0].m_state[i].t,pow(dist_max,0.5)));
  }
}
