//-----------------------------------------------------------------------------------------------
//---------------------------------------COMMENTAIRES--------------------------------------------
//
//-----------------------------------------------------------------------------------------------
//-----------------------------------------INCLUDES----------------------------------------------
#include "iostream"
#include <vector>
#include <iterator>
#include "MBUtils.h"
#include <math.h>

using namespace std;
using namespace cv;

//-----------------------------------------------------------------------------------------------
//-----------------------------------------FONCTIONS---------------------------------------------

Robot::Robot()
:x(Mat::zeros(3, 1, CV_64F)), u(Mat::zeros(1, 1, CV_64F)),C(Mat::zeros(2, 3, CV_64F)),A(Mat::zeros(3, 3, CV_64F)),
Galpha(Mat::zeros(3, 3, CV_64F)),y(Mat::zeros(2, 1, CV_64F)),Gbeta(Mat::zeros(3, 3, CV_64F)),m_kalman(Kalman()),m_number(0),
Gx(Mat::zeros(3, 3, CV_64F)){}

void Robot::InitValues(){
  x.at<double>(0,0) = 1;
  x.at<double>(1,0) = 1;
  x.at<double>(2,0) = 0;

  u.at<double>(0,0) = 0;

  A.at<double>(0,2) = cos(0);
  A.at<double>(1,2) = sin(0);
  A.at<double>(2,2) = -1;

  Galpha.at<double>(0,0) = 0.1^2;
  Galpha.at<double>(1,1) = 0.1^2;
  Galpha.at<double>(2,2) = 0.1^2;

  Gbeta.at<double>(0,0) = 0.1^2;
  Gbeta.at<double>(1,1) = 0.1^2;
  Gbeta.at<double>(2,2) = 0.1^2;

  Gx.at<double>(0,0) = 0.1^2;
  Gx.at<double>(1,1) = 0.1^2;
  Gx.at<double>(2,2) = 0.1^2;

}

void Robot::kalman_predict(Mat xup_k,Mat Pup_k, Mat Q, Mat A, Mat u, Mat* x_k1, Mat* P_k1){
  *P_k1 = (A*Pup_k*A.t()) + Q;
  *x_k1 = A*xup_k + u;
}

void Robot::kalman_correct(Mat x_k1, Mat P_k1, Mat  C, Mat R, Mat y, Mat* xup_k1, Mat* Pup_k1){
   Mat S = Mat::zeros(Size(SIZEY,SIZEY),CV_64F);
   Mat K = Mat::zeros(Size(SIZEX,SIZEY),CV_64F);
   Mat err = Mat::zeros(SIZEY,1,CV_64F);

   S = (C*P_k1*C.t()) + R;
   K = P_k1*C.t()*S.inv();
   err = y - (C*x_k1);
   *Pup_k1 = (Mat::eye(SIZEX,SIZEX,CV_64F) - (K*C)) * P_k1;
   *xup_k1 = x_k1 + K*err;
}

void Robot::kalman_x(Mat x_k, Mat P_k, Mat u, Mat y, Mat Q, Mat R,Mat A ,Mat C, Mat* P_k1, Mat* x_k1){
  Mat Pup_k = Mat::zeros(Size(SIZEX,SIZEX),CV_64F);
  Mat xup_k = Mat::zeros(Size(SIZEX,1),CV_64F);

  kalman_correct(x_k , P_k, C, R, y, &xup_k, &Pup_k);
  kalman_predict(xup_k, Pup_k, Q, A, u, x_k1, P_k1);
}


void Robot::draw(Gnuplot gp){
    double norm = 0;
    for (int i=0;i<3;i++){
        for (int j=0;j<3;j++){
            norm += m_kalman.theta_p_out.at<double>(i,j);
        }
    }
    plot.push_back(Point(m_number+1,norm));
    gp << "plot '-'\n";
    gp.send1d(plot);
    m_number++;
}
