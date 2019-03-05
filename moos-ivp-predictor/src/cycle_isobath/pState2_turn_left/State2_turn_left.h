/************************************************************/
/*    NAME:                                          */
/*    ORGN: MIT                                             */
/*    FILE: State2_turn_left.h                                          */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#ifndef State2_turn_left_HEADER
#define State2_turn_left_HEADER
//PID coefficients
#define COEFF 6
#define COEFFDER 12
#define COEFINTERR 0.020
//
#define FIRST 361
#define MAXINT 50/COEFINTERR

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include <math.h>

class State2_turn_left : public AppCastingMOOSApp
{
 public:
   State2_turn_left();
   ~State2_turn_left();
   double state2;
   double state4;

 protected: // Standard MOOSApp functions to overload
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();

 protected: // Standard AppCastingMOOSApp function to overload
   bool buildReport();

 protected:
   void registerVariables();
   double HT_BowRightLeft, HT_SternRightLeft;
   double AH_Heading, HT_HeadingWanted, HT_HeadingStable;
   double DB_UPTIME;

 private: // Configuration variables
    double previousErr, derErr, command, sumErr, err, t0;

 private: // State variables
};

#endif
