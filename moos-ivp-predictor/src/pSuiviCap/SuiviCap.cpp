/************************************************************/
/*    NAME:                                               */
/*    ORGN: MIT                                             */
/*    FILE: SuiviCap.cpp                                        */
/*    DATE:                                                 */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "SuiviCap.h"

using namespace std;

//---------------------------------------------------------
// Constructor

SuiviCap::SuiviCap()
{
  Theta_voulu = 0;
  err = FIRST;
  sumErr = 0;
  Theta = Theta_voulu;
  t0 = 0;
}

//---------------------------------------------------------
// Destructor

SuiviCap::~SuiviCap()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail

bool SuiviCap::OnNewMail(MOOSMSG_LIST &NewMail)
{
  MOOSMSG_LIST::iterator p;

  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;

    string key = msg.GetKey();
    if (key == "Nav_Heading") {
      Theta = msg.GetDouble();
    } else if (key == "Theta_voulu") {
      err    = 0;
      sumErr = 0;
      derErr = 0;
      Theta_voulu = msg.GetDouble();
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

bool SuiviCap::OnConnectToServer()
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

bool SuiviCap::Iterate()
{
  if (err == FIRST) { // Premier passage dans Iterate
    err = (fmod(((Theta_voulu - Theta))+360,360))-180;
    derErr = 0;
    sumErr = err;

  }
  else {
    previousErr = err;
    err = (fmod(((Theta_voulu - Theta)+360),360))-180;
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
  Erreur_cap = COEFF * err + COEFFDER * derErr + COEFINTERR * sumErr;
  Notify("Erreur_cap ",Erreur_cap);

  return(true);
}
//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool SuiviCap::OnStartUp()
{
  list<string> sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if(m_MissionReader.GetConfiguration(GetAppName(), sParams)) {
    list<string>::iterator p;
    for(p=sParams.begin(); p!=sParams.end(); p++) {
      string line  = *p;
      string param = tolower(biteStringX(line, '='));
      string value = line;

      if(param == "foo") {
        //handled
      }
      else if(param == "bar") {
        //handled
      }
    }
  }

  RegisterVariables();
  return(true);
}

//---------------------------------------------------------
// Procedure: RegisterVariables

void SuiviCap::RegisterVariables()
{
  Register("Theta",0);
  Register("Theta_voulu",0);
  // Register("FOOBAR", 0);
}
