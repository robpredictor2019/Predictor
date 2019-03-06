/************************************************************/
/*    NAME:                                               */
/*    ORGN: MIT                                             */
/*    FILE: SimulationStateMachine.h                                          */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#ifndef SimulationStateMachine_HEADER
#define SimulationStateMachine_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include <math.h>

class SimulationStateMachine : public AppCastingMOOSApp
{
 public:
   SimulationStateMachine();
   ~SimulationStateMachine();

 protected: // Standard MOOSApp functions to overload  
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();

 protected: // Standard AppCastingMOOSApp function to overload 
   bool buildReport();

 protected:
   void registerVariables();
   double nav_x, nav_y,  point_a_x, point_b_x, point_c_x, point_a_y, point_b_y, point_c_y;

 private: // Configuration variables
  double heading, depth, speed;
 private: // State variables
  double time_underwater, point_m_x, point_m_y;
  int state, N; // State value is 0 (Up), 1 (Down and orienting itself) or 2 (Following the desired heading)
  char point; // Four possible points : A, B ,C
};

#endif 
