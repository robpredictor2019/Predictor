/************************************************************/
/*    NAME:                                               */
/*    ORGN: MIT                                             */
/*    FILE: Command.h                                          */
/*    DATE:                                                 */
/************************************************************/

#ifndef Command_HEADER
#define Command_HEADER

#include "MOOS/libMOOS/MOOSLib.h"

class Command : public CMOOSApp
{
 public:
   Command();
   ~Command();

 protected: // Standard MOOSApp functions to overload
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();

 protected:
   void RegisterVariables();

 private: // Configuration variables
  double Erreur_cap;
  double dTheta;

 private: // State variables
};

#endif
