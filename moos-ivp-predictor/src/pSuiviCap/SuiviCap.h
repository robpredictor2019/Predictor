/************************************************************/
/*    NAME:                                               */
/*    ORGN: MIT                                             */
/*    FILE: SuiviCap.h                                          */
/*    DATE:                                                 */
/************************************************************/

#ifndef SuiviCap_HEADER
#define SuiviCap_HEADER
#define COEFF 6
#define COEFFDER 12
#define COEFINTERR 0.020
#define FIRST 361
#define MAXINT 50/COEFINTERR

#include "MOOS/libMOOS/MOOSLib.h"
#include <math.h>

class SuiviCap : public CMOOSApp
{
 public:
   SuiviCap();
   ~SuiviCap();

 protected: // Standard MOOSApp functions to overload
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();

 protected:
   void RegisterVariables();
   double Theta,Theta_voulu;

 private: // Configuration variables
   double previousErr, derErr, Erreur_cap, sumErr, err, t0;

 private: // State variables
};

#endif
