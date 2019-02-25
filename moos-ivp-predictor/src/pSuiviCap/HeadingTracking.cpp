/************************************************************/
/*    NAME: Paul-Antoine GRAU                                              */
/*    ORGN: MIT                                             */
/*    FILE: HeadingTracking.cpp                                        */
/*    DATE:                                                 */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "HeadingTracking.h"

using namespace std;

//---------------------------------------------------------
// Constructor

HeadingTracking::HeadingTracking()
{
  HT_HeadingWanted = 0;
  err = FIRST;
  sumErr = 0;
  AH_Heading = HT_HeadingWanted;
  HT_HeadingStable = 0;
  t0 = 0;
}

//---------------------------------------------------------
// Destructor

HeadingTracking::~HeadingTracking()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail

bool HeadingTracking::OnNewMail(MOOSMSG_LIST &NewMail)
{
  MOOSMSG_LIST::iterator p;

  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;

    string key = msg.GetKey();
    if (key == "AH_Heading") {
      AH_Heading = msg.GetDouble();
    } else if (key == "HT_HeadingWanted") {
      err    = 0;
      sumErr = 0;
      derErr = 0;
      HT_HeadingWanted = msg.GetDouble();
    } else if (key == "DB_UPTIME") {
      DB_UPTIME = msg.GetDouble();
    }

#if 0 // Keep these around just for template
    string key   = msg.GetKey();
    string comm  = msg.GetCommunity();
    double dval  = msg.GetDouble();
    string sval  = msg.GetString();
    string msrc  = msg.GetSource();
    double mtime = msg.GetTime();
    bool   mdbl  = msg.IsDouble();
    bool   mstr  = msg.IsString();
#endif
   }

   return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer

bool HeadingTracking::OnConnectToServer()
{
   // register for variables here
   // possibly look at the mission file?
   // m_MissionReader.GetConfigurationParam("Name", <string>);
   // m_Comms.Register("VARNAME", 0);

   RegisterVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool HeadingTracking::Iterate()
{
  if (err == FIRST) { // Premier passage dans Iterate
    err = (fmod(((HT_HeadingWanted - AH_Heading))+360,360))-180;
    derErr = 0;
    sumErr = err;

  }
  else {
    previousErr = err;
    err = (fmod(((HT_HeadingWanted - AH_Heading)+360),360))-180;
    derErr = (err - previousErr);
    sumErr += err;
  }

  // Seuillage de l'intégrale
  if (abs(sumErr) > MAXINT) {
    sumErr = sumErr / abs(sumErr) * MAXINT;
  }

  if(fabs(err)*COEFF>=100)
  {
    sumErr=0;
    derErr=0;
  }

  //printf("Err : %f \n",err);
  //printf("Intégrale : %f \n",sumErr);
  // Commande Proportionnelle Intégrale Dérivée
  command = COEFF * err + COEFFDER * derErr + COEFINTERR * sumErr;

  // Vérification de la possibilité de la correction classique
  if (command > 100) {
    HT_BowRightLeft  = 100;
    HT_SternRightLeft = -100;
  }
  else if (command < -100) {
    HT_BowRightLeft = -100;
    HT_SternRightLeft = 100;
  }
  else {
    HT_BowRightLeft = command;
    HT_SternRightLeft = -command;
  }

  if (abs(err) > 10) {
    t0 = DB_UPTIME;
    HT_HeadingStable = 0;
  }
  if (DB_UPTIME - t0 > 5) {
    HT_HeadingStable = 1;
    t0 = DB_UPTIME;
  }

  // Enregistrement des variables partagées
  Notify("HT_BowRightLeft",HT_BowRightLeft,"%");
  Notify("HT_SternRightLeft",HT_SternRightLeft,"%");
  Notify("HT_HeadingStable",HT_HeadingStable);

  HT_HeadingStable = 0;
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool HeadingTracking::OnStartUp()
{
  list<string> sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if(m_MissionReader.GetConfiguration(GetAppName(), sParams)) {
    list<string>::iterator p;
    for(p=sParams.begin(); p!=sParams.end(); p++) {
      string original_line = *p;
      string param = stripBlankEnds(toupper(biteString(*p, '=')));
      string value = stripBlankEnds(*p);

      if(param == "FOO") {
        //handled
      }
      else if(param == "BAR") {
        //handled
      }
    }
  }

  RegisterVariables();
  return(true);
}

//---------------------------------------------------------
// Procedure: RegisterVariables

void HeadingTracking::RegisterVariables()
{
  Register("HT_HeadingWanted",0);
  Register("AH_Heading",0);
  Register("DB_UPTIME");
  // Register("FOOBAR", 0);
}
