/************************************************************/
/*    NAME:  Antony                                             */
/*    ORGN: MIT                                             */
/*    FILE: State3_cross_river.h                                          */
/*    DATE:                                                 */
/************************************************************/

#ifndef State3_cross_river_HEADER
#define State3_cross_river_HEADER



#define COEFF 6
#define COEFFDER 12
#define COEFINTERR 0.020
#define FIRST 361
#define MAXINT 50/COEFINTERR

#include "MOOS/libMOOS/MOOSLib.h"

class State3_cross_river : public CMOOSApp
{
 public:
   State3_cross_river();
   ~State3_cross_river();
   double state_3;
   double state_4;
   double profondeur;
   double altitude;
   double colon_deau ;
   bool milieu_passe ;

   double depth__iso ;
   double depth__inc;
   double boucle;

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
