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

void Robot::kalman_x(Mat x_k,Mat P_k,Mat u,Mat y,Mat Q, Mat R,Mat A ,Mat C, Mat* P_k1, Mat* x_k1){
  Mat Pup_k = Mat::zeros(Size(SIZEX,SIZEX),CV_64F);
  Mat xup_k = Mat::zeros(Size(SIZEX,1),CV_64F);

  kalman_correct(x_k,P_k, C,R, y, &xup_k, &Pup_k);
  kalman_predict(xup_k,Pup_k, Q, A,u,x_k1,P_k1);
}

float Robot::scenario()
{
  kalman_x();
}
