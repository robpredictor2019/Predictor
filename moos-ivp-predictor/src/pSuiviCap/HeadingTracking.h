/************************************************************/
/*    NAME: Paul-Antoine GRAU                                              */
/*    ORGN: MIT                                             */
/*    FILE: HeadingTracking.h                                          */
/*    DATE:                                                 */
/************************************************************/

#ifndef HeadingTracking_HEADER
#define HeadingTracking_HEADER
#define COEFF 6
#define COEFFDER 12
#define COEFINTERR 0.020
#define FIRST 361
#define MAXINT 50/COEFINTERR

#include "MOOS/libMOOS/MOOSLib.h"
#include <math.h>

class HeadingTracking : public CMOOSApp
{
 public:
   HeadingTracking();
   ~HeadingTracking();

 protected: // Standard MOOSApp functions to overload  
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();

 protected:
   void RegisterVariables();
   double HT_BowRightLeft, HT_SternRightLeft;
   double AH_Heading, HT_HeadingWanted, HT_HeadingStable;
   double DB_UPTIME;

 private: // Configuration variables
   double previousErr, derErr, command, sumErr, err, t0;

 private: // State variables
};

#endif
