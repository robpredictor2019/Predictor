/************************************************************/
/*    NAME: Louis VALERY                                              */
/*    ORGN: MIT                                             */
/*    FILE: StateMachine.cpp                                        */
/*    DATE:                                                 */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "StateMachine.h"
#include <time.h>

using namespace std;
//using namespace cv;
//---------------------------------------------------------
// Constructor

StateMachine::StateMachine()
{
	//parameters
	time_iso = 0.1; //time for first isobath in seconds

	depth__iso = 10;   // depth of the first isobath in m (will be increased for the following)
  depth__inc = 5;    // every isobath is 5m deeper than the previous

	//states state
	state_1 = 1;  // Isobath following
	state_2 = 0;  // Turning left
	state_3 = 0;  // Crossing river
	state_4 = 0;  // Finding right direction for next isobath

  //state1 variables
	begin_time = clock();
  begin_state1 = 0;
	//state2 variables
  state2_angle_turn = 90/360 * 2 * M_PI ;
	state2_range_turn = 10/360 * 2 * M_PI ;

	//state4 variables
  state4_angle_turn = 180/360 * 2 * M_PI ;
	state4_range_turn = 10/360 * 2 * M_PI ;

	//counter
	boucle = 0;

	//Aquired data
	clock_isobath = 0;
  heading  =0;
  pression = 0;
  sondeur = 0;
  heading_last_iso = 0;

	depth = 0;
	min_loc = false;

  clock_state1 = 0;
}

//---------------------------------------------------------
// Destructor

StateMachine::~StateMachine()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail

bool StateMachine::OnNewMail(MOOSMSG_LIST &NewMail)
{
  MOOSMSG_LIST::iterator p;

  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;

    string key = msg.GetKey();

    //clock_isobath
	if (key == "Clock_isobath"){
      clock_isobath = msg.GetDouble() ;
	}

    //heading
	if (key == "AH_Heading"){
	  heading = msg.GetDouble()/360 * 2 * M_PI ; //heading in radians
	}

	//pression
	if (key == "AF_Depth"){
	  pression = msg.GetDouble();   //in m
	  depth = sondeur + pression;  //maj depth
  }

  //sondeur
  if (key == "FA_Altitude"){
	  sondeur = msg.GetDouble()/100;   //in m
	  depth = sondeur + pression;  //maj depth
	}

	//heading_last_iso
	if (key == "Current_heading_isobath"){
      heading_last_iso = msg.GetDouble() ;
	}

	if (key == "Clock_State1"){
      clock_state1 = msg.GetDouble() ;
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

bool StateMachine::OnConnectToServer()
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

bool StateMachine::Iterate()
{
	time_passed = double( clock() - begin_time)/CLOCKS_PER_SEC;
	if (time_passed > time_iso){
		clock_state1 = 1;}
	else{
		clock_state1 = 0;}

		//pour la simu
  Notify("Boucle", boucle);
  Notify("Depth__iso", depth__iso);
  Notify("Depth__inc", depth__inc);


	// an isobath was followed and time out...
  if (state_1 == 1 && clock_state1 == 1){
    state_1 == 0;
  	state_2 == 1;
		//Notify("HT_HeadingWanted",heading_last_iso + state2_angle_turn);

  }

  // we are turning (targeting the other side of the river) and the angle is aprox ok...
  else if (state_2 == 1
      && heading <= heading_last_iso + state2_angle_turn + state2_range_turn
      && heading >= heading_last_iso +state2_angle_turn - state2_range_turn){
    state_2 == 0;
  	state_3 == 1;
  }

  // if a heading is followed (to reach the other side of the river)
  //and the right depth is nearnly reached
  else if (state_3 == 1
  	  && depth >= depth__iso + depth__inc * boucle + 5   //5 meters before reching the right depth
  	  && min_loc == true){                               // if the river as been crossed
    min_loc = false;
    state_3 == 0;
  	state_4 == 1;
		Notify("HT_HeadingWanted",heading_last_iso + state4_angle_turn);
  }

  // if we are in the right direction to follow the next isobath
  else if (state_4 == 1
      && heading <= heading_last_iso + state4_angle_turn + state4_range_turn
      && heading >= heading_last_iso + state4_angle_turn - state4_range_turn){
    state_4 == 0;
  	state_1 == 1;
		begin_time = clock();
  }

  Notify("State_1",state_1);
  Notify("State_2",state_2);
  Notify("State_3",state_3);
  Notify("State_4",state_4);

  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool StateMachine::OnStartUp()
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

void StateMachine::RegisterVariables()
{
  // Register("FOOBAR", 0);
  Register("clock_isobath", 0);
  Register("AH_Heading", 0);
  Register("AF_Depth", 0);
  Register("FA_Altitude", 0);
  Register("current_heading_isobath", 0);
}
