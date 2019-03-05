/************************************************************/
/*    NAME:                                               */
/*    ORGN: MIT                                             */
/*    FILE: State3_cross_river.cpp                                        */
/*    DATE:                                                 */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "State3_cross_river.h"

using namespace std;

//---------------------------------------------------------
// Constructor

State3_cross_river::State3_cross_river()
{
  colon_deau = 0;
  state_4 = 0;
  milieu_passe = false;


  HT_HeadingWanted = 0;
  err = FIRST;
  sumErr = 0;
  HT_HeadingStable = 0;
  t0 = 0;

  depth__iso = 0;
  depth__inc = 0;
  boucle = 0;
}

//---------------------------------------------------------
// Destructor

State3_cross_river::~State3_cross_river()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail

bool State3_cross_river::OnNewMail(MOOSMSG_LIST &NewMail)
{
  MOOSMSG_LIST::iterator p;

  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;

  string key = msg.GetKey();

  if (key == "State_3"){
      state_3 = msg.GetDouble() ;
	}

  if (key == "AH_Heading") {
    AH_Heading = msg.GetDouble() ;
  //pression
    }


  if (key == "AF_Depth"){
    profondeur = msg.GetDouble();   //in m
      //maj depth
    }
  else if (key == "DB_UPTIME") {
      DB_UPTIME = msg.GetDouble();
    }

  //sondeur
  if (key == "FA_Altitude"){
    altitude = msg.GetDouble()/100;   //in m
  }

  else if (key == "Boucle"){
    boucle = msg.GetDouble();
  }
  else if (key == "Depth__iso"){
    depth__iso = msg.GetDouble();
  }

  else if (key == "Depth__inc"){
    depth__inc = msg.GetDouble();
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

bool State3_cross_river::OnConnectToServer()
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

bool State3_cross_river::Iterate()
{
  if (state_3 ==1){  // si on est dans l'etat 3

    // on avance selon le cap actuel

    if (err == FIRST) { // Premier passage dans Iterate
      HT_HeadingWanted = AH_Heading;
      err = 0;
      derErr = 0;
      sumErr = err;

    }
    else {
      previousErr = err;
      err = (fmod(((HT_HeadingWanted - AH_Heading)+360),360))-180;
      derErr = (err - previousErr);
      sumErr += err;
    }
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

    // on verifie les conditions de depassmeent
    if (colon_deau >  depth__iso + depth__inc * boucle + 5
    && milieu_passe == false)
    {
      milieu_passe = true;
    }

    if (milieu_passe == true && colon_deau < depth__iso +5  ){
      // arret si bon moment
      Notify("SBT_Propulsion",1); //notify n'accepte pas zéro - rédundance

    }

    else
      Notify("SBT_Propulsion",50);

    // Enregistrement des variables partagées
    Notify("HT_BowRightLeft",HT_BowRightLeft,"%");
    Notify("HT_SternRightLeft",HT_SternRightLeft,"%");
    Notify("HT_HeadingStable",HT_HeadingStable);



  }


  return(true);
}



//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool State3_cross_river::OnStartUp()
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

void State3_cross_river::RegisterVariables()
{
  Register("State_3", 0);
  Register("AF_Depth", 0);
  Register("FA_Altitude", 0);
  Register("AH_Heading",0);
  Register("DB_UPTIME",0);
  Register("Boucle",0);
  Register("Depth__iso",0);
  Register("Depth__inc",0);
  // Register("FOOBAR", 0);
}
