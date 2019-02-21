/************************************************************/
/*    NAME: COSTA Maria                                              */
/*    ORGN: MIT                                             */
/*    FILE: kalman.h                                          */
/*    DATE:                                                 */
/************************************************************/

#ifndef Kalman_HEADER
#define Kalman_HEADER

#include "MOOS/libMOOS/MOOSLib.h"
#include <opencv2/core/core.hpp>


#define M_PI           3.14159265358979323846
#define SIZEX 3
#define SIZEU 3
#define SIZEY 1


using namespace cv;
using namespace std;


class Kalman : public CMOOSApp
{
 public:
   Kalman();
   ~Kalman();
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


 protected: // Standard MOOSApp functions to overload
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();
   Mat g(Mat, Mat);
/*   float sawtooth(float);
   void control(Mat, Mat, Mat , Mat*);
   void f(Mat, Mat, Mat*);
   void euler(double dt, Mat dx, Mat * x);*/

   void kalman_predict(Mat,Mat,Mat,Mat,Mat, Mat*, Mat*);
   void kalman_correct(Mat, Mat, Mat, Mat, Mat, Mat*, Mat*);
   void kalman_x(Mat,Mat,Mat,Mat,Mat, Mat,Mat ,Mat, Mat*, Mat* );


 protected:
   void RegisterVariables();

 private: // Configuration variables

 private: // State variables
};

#endif
