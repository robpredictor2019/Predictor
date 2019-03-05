/************************************************************/
/*    NAME: Louis VALERY                                              */
/*    ORGN: MIT                                             */
/*    FILE: StateMachine.h                                          */
/*    DATE:                                                 */
/************************************************************/

#ifndef StateMachine_HEADER
#define StateMachine_HEADER

#include "MOOS/libMOOS/MOOSLib.h"
#include <time.h>
using namespace std;
//using namespace cv;

#define M_PI           3.14159265358979323846

class StateMachine : public CMOOSApp
{
 public:
   StateMachine();
   ~StateMachine();
   double clock_state1;
   double time_iso;
   double depth__iso;
   double depth__inc;
   double time_passed;
   double begin_state1;
   clock_t begin_time;

   double state_1;
   double state_2;
   double state_3;
   double state_4;

   double state2_angle_turn;
   double state2_range_turn;

   double state4_angle_turn;
   double state4_range_turn;

   double boucle;
   double depth;
   bool min_loc;

   double clock_isobath;
   double heading;
   double pression;
   double sondeur;
   double heading_last_iso;

 protected: // Standard MOOSApp functions to overload
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();

 protected:
   void RegisterVariables();

 private: // Configuration variables

 private: // State variables
};

#endif
