/************************************************************/
/*    NAME:                                               */
/*    ORGN: MIT                                             */
/*    FILE: State2_turn_left.cpp                                        */
/*    DATE:                                                 */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "ACTable.h"
#include "State2_turn_left.h"

using namespace std;

//---------------------------------------------------------
// Constructor

State2_turn_left::State2_turn_left()
{
  HT_HeadingWanted = 0;
  err = FIRST;
  sumErr = 0;
  AH_Heading = HT_HeadingWanted;
  HT_HeadingStable = 0;
  t0 = 0;
  state2 = 0;
  state2 = 4;
}

//---------------------------------------------------------
// Destructor

State2_turn_left::~State2_turn_left()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail

bool State2_turn_left::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key    = msg.GetKey();

    if (key == "AH_Heading") {
      AH_Heading = msg.GetDouble() ;
    }

    else if (key == "HT_HeadingWanted") {
      err    = 0;
      sumErr = 0;
      derErr = 0;
      HT_HeadingWanted = msg.GetDouble();
    }

    else if (key == "DB_UPTIME") {
      DB_UPTIME = msg.GetDouble();
    }

    else if (key == "State_2") {
      state2 = msg.GetDouble();
    }

    else if (key == "State_4") {
      state4 = msg.GetDouble();
    }

#if 0 // Keep these around just for template
    string comm  = msg.GetCommunity();
    double dval  = msg.GetDouble();
    string sval  = msg.GetString();
    string msrc  = msg.GetSource();
    double mtime = msg.GetTime();
    bool   mdbl  = msg.IsDouble();
    bool   mstr  = msg.IsString();
#endif

     if(key == "FOO")
       cout << "great!";

     else if(key != "APPCAST_REQ") // handled by AppCastingMOOSApp
       reportRunWarning("Unhandled Mail: " + key);
   }

   return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer

bool State2_turn_left::OnConnectToServer()
{
   registerVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
// happens AppTick times per second

bool State2_turn_left::Iterate()
{
  AppCastingMOOSApp::Iterate();
  if (state2 || state4 ){
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
    command = COEFF * err + COEFFDER * derErr + COEFINTERR * sumErr;
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
  } //fin if state2
  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool State2_turn_left::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();

  STRING_LIST sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if(!m_MissionReader.GetConfiguration(GetAppName(), sParams))
    reportConfigWarning("No config block found for " + GetAppName());

  STRING_LIST::iterator p;
  for(p=sParams.begin(); p!=sParams.end(); p++) {
    string orig  = *p;
    string line  = *p;
    string param = tolower(biteStringX(line, '='));
    string value = line;

    bool handled = false;
    if(param == "foo") {
      handled = true;
    }
    else if(param == "bar") {
      handled = true;
    }

    if(!handled)
      reportUnhandledConfigWarning(orig);

  }

  registerVariables();
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables

void State2_turn_left::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  // Register("FOOBAR", 0);
  Register("AH_Heading", 0);
  Register("HT_HeadingWanted", 0);
  Register("DB_UPTIME", 0);
  Register("State_2", 0);
  Register("State_4", 0);
}


//------------------------------------------------------------
// Procedure: buildReport()

bool State2_turn_left::buildReport()
{
  m_msgs << "============================================ \n";
  m_msgs << "File:                                        \n";
  m_msgs << "============================================ \n";

  ACTable actab(4);
  actab << "Alpha | Bravo | Charlie | Delta";
  actab.addHeaderLines();
  actab << "one" << "two" << "three" << "four";
  m_msgs << actab.getFormattedString();

  return(true);
}
