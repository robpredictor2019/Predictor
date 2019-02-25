/************************************************************/
/*    NAME:                                               */
/*    ORGN: MIT                                             */
/*    FILE: SimulationPredictor.h                                          */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#ifndef SimulationPredictor_HEADER
#define SimulationPredictor_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"

class SimulationPredictor : public AppCastingMOOSApp
{
 public:
   SimulationPredictor();
   ~SimulationPredictor();

 protected: // Standard MOOSApp functions to overload  
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();

 protected: // Standard AppCastingMOOSApp function to overload 
   bool buildReport();

 protected:
   void registerVariables();

 private: // Configuration variables

 private: // State variables
};

#endif 
