/************************************************************/
/*    NAME:                                                 */
/*    ORGN: MIT                                             */
/*    FILE: SimulationStateMachine.h                        */
/*    DATE:                                                 */
/************************************************************/

#ifndef SimulationStateMachine_HEADER
#define SimulationStateMachine_HEADER
#define M_PI           3.14159265358979323846

#include <time.h>
#include <string>
#include "MOOS/libMOOS/MOOSLib.h"

using namespace std;

class SimulationStateMachine : public CMOOSApp
{
 public:
   SimulationStateMachine();
   ~SimulationStateMachine();

   double gps_lat;
   double gps_long;
   double gps_m_lat;
   double gps_m_long;
   double propulsion;
   double a_to_b; // on va de a a b
   double b_to_c;
   double c_to_a;
   double time_croisiere;
   double dist_croisiere;
   double eps_yaw;
   double time_passed;
   double time_passed_s1;
   double eps_prof;
   double nbr_gps;
   double vitesse;
   double A_lat;
   double A_long;
   double B_lat;
   double nav_depth;
   double desired_depth ;
   double desired_heading;
   double compteur_gps  ;
   double err_depth;
   double err_heading;
   double actual_depth;
   double B_long;
   double C_lat;
   double C_long;
   double state_1;
   double state_2;
   double state_3;
   clock_t begin_time;
   double heading;
   int start_state3 ;


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
