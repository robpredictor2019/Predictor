/************************************************************/
/*    NAME:                                                 */
/*    ORGN: MIT                                             */
/*    FILE: SimulationStateMachine.cpp                      */
/*    DATE:                                                 */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "ACTable.h"
#include "SimulationStateMachine.h"

/*
Used for creating a state machine for the robot
*/

using namespace std;

//---------------------------------------------------------
// Constructor

SimulationStateMachine::SimulationStateMachine()
{
  heading = 90;
  depth = 0;
  speed = 0;
  point_m_x = 0;
  point_m_y = 0;
  time_underwater = 0;
  point_a_x = POINT_A_X; 
  point_b_x = POINT_B_X;
  point_c_x = POINT_C_X;
  point_a_y = POINT_A_Y;
  point_b_y = POINT_B_Y;
  point_c_y = POINT_C_Y;
  state = 0;
  point = 'A';
  nav_x = 0;
  nav_y = 0;
  N = 0;
  
}

//---------------------------------------------------------
// Destructor

SimulationStateMachine::~SimulationStateMachine()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail

bool SimulationStateMachine::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key    = msg.GetKey();
    if(key == "NAV_X"){
        nav_x = msg.GetDouble();
    } else if(key == "NAV_Y") {
      nav_y = msg.GetDouble();
    }
    else if(key != "APPCAST_REQ") // handled by AppCastingMOOSApp
      reportRunWarning("Unhandled Mail: " + key);

#if 0 // Keep these around just for template
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

bool SimulationStateMachine::OnConnectToServer()
{
   registerVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool SimulationStateMachine::Iterate()
{
  AppCastingMOOSApp::Iterate();
  // Do your thing here!
  AppCastingMOOSApp::PostReport();

  // Do something depending the case :
  switch(state){
    case 0: // Taking GPS Measurements
      N+=1;
      speed = 0; // Stop the AUV
      if (N == 10)
            state = 1; // Go underwater
            depth = 2;
    case 1: // Going underwater and computing the desired heading and the time underwater
      point_m_x = nav_x;
      point_m_y = nav_y;
      speed = 2; // Restart the AUV
      N = 0;
      switch(point){ // Checking the current position
      
        case 'A': // Next point is B
            heading = atan((point_b_x - point_m_x)/(point_b_y - point_m_y));
            time_underwater = (sqrt(pow(point_b_x - point_m_x,2) + pow(point_b_y - point_m_y,2) ))/speed;
        case 'B': // Next point is C
            heading =  atan((point_c_x - point_m_x)/(point_c_y - point_m_y));
            time_underwater = (sqrt(pow(point_c_x - point_m_x,2) + pow(point_c_y - point_m_y,2) ))/speed;
        case 'C': // Next point is A
            heading =  atan((point_a_x - point_m_x)/(point_a_y - point_m_y));
            time_underwater = (sqrt(pow(point_a_x - point_m_x,2) + pow(point_a_y - point_m_y,2) ))/speed;
        case 'D': // Next point is A
            heading =  atan((point_a_x - point_m_x)/(point_a_y - point_m_y));
            time_underwater = (sqrt(pow(point_a_x - point_m_x,2) + pow(point_a_y - point_m_y,2) ))/speed;
        }
        if (depth == 2)
              state = 2; // Follow the heading
    case 2:
      time_underwater = time_underwater - 1/10;
      switch(point){ // Checking the current position
          case 'A': // Next point is B
              time_underwater = (sqrt(pow(point_b_x - nav_x,2) + pow(point_b_y - nav_y,2) ))/speed;
          case 'B': // Next point is C
              time_underwater = (sqrt(pow(point_c_x - nav_x,2) + pow(point_c_y - nav_y,2) ))/speed;
          case 'C': // Next point is A
              time_underwater = (sqrt(pow(point_a_x - nav_x,2) + pow(point_a_y - nav_y,2) ))/speed;
          case 'D': // Next point is A
              time_underwater = (sqrt(pow(point_a_x - nav_x,2) + pow(point_a_y - nav_y,2) ))/speed;
        }
      if (time_underwater < 1){
        state = 0;
        depth = 0;
          switch(point){ // Checking the current position
          case 'A': // Next point is B
            point = 'B';
          case 'B': // Next point is C
            point = 'C';
          case 'C': // Next point is A
            point = 'A';
          }
      }
  }
  
  // Sawtooth
  //heading = fmod(heading + M_PI, 2*M_PI) - M_PI;

  Notify("DESIRED_RUDDER", heading);
  Notify("DESIRED_ELEVATOR", -depth);
  Notify("DESIRED_THRUST", 25*speed);
  Notify("STATE_MACHINE_STATE", state);
  Notify("NEXT_POINT_TARGETED", point);
  Notify("TIME_UNDERWATER", time_underwater);
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool SimulationStateMachine::OnStartUp()
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
    string param = toupper(biteStringX(line, '='));
    string value = line;

    bool handled = false;
    if(param == "FOO") {
      handled = true;
    }
    else if(param == "BAR") {
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

void SimulationStateMachine::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("NAV_X", 0);
  Register("NAV_Y", 0);
  // Register("FOOBAR", 0);
}


//------------------------------------------------------------
// Procedure: buildReport()

bool SimulationStateMachine::buildReport() 
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




