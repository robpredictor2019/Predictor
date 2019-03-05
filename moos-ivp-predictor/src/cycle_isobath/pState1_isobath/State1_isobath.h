/************************************************************/
/*    NAME: Louis VALERY                                              */
/*    ORGN: MIT                                             */
/*    FILE: State1_isobath.h                                          */
/*    DATE:                                                 */
/************************************************************/

#ifndef State1_isobath_HEADER
#define State1_isobath_HEADER

#include "MOOS/libMOOS/MOOSLib.h"
#include <opencv2/core/core.hpp>
#include <queue>
#include <time.h>

# define M_PI           3.14159265358979323846
# define  STATE1_PERIOD  20

using namespace cv;
using namespace std;


class State1_isobath : public CMOOSApp
{
 public:
   State1_isobath();
   ~State1_isobath();
   double state_1;
   double dt;
   double d0;
   double h0;
   double hm;
   double begin_state1;

   Mat phat;
   Mat x;
   Mat dx;
   Mat u;
   Mat y;
   double clock_fin;

   double heading;
   clock_t begin_time;

 protected: // Standard MOOSApp functions to overload
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();

   Mat g(Mat, Mat);
   float sawtooth(float);
   void control(Mat, Mat, Mat , Mat*);
   void f(Mat, Mat, Mat*);
   void euler(double dt, Mat dx, Mat * x);


 protected:
   void RegisterVariables();
   double current_heading;
 private: // Configuration variables
 int sz_history;
 int i;
 double mean_heading;

 queue<double> heading_history;
 queue<double> heading_history_tmp;



 private: // State variables
};

#endif
