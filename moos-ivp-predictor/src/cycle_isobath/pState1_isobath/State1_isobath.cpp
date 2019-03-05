/************************************************************/
/*    NAME: Louis VALERY                                              */
/*    ORGN: MIT                                             */
/*    FILE: State1_isobath.cpp                                        */
/*    DATE:                                                 */
/************************************************************/

//This file is approximativly the same that pControllerIsobath
//There are two main differences
//-First it count and publish time when it is used
//-Second it estimate the heading of the last minute



#include <iterator>
#include "MBUtils.h"
#include "State1_isobath.h"
#include <math.h>
#include <queue>
//#include <chrono>
#include <iostream>
//#include <time.h>

using namespace std;
using namespace cv;

//---------------------------------------------------------
// Constructor

State1_isobath::State1_isobath()
{
  dt = 0.05;
  d0 = 0.0;
  h0 = 0.0;
  hm = 0.0;
  phat = Mat::zeros(3, 1, CV_64F);

  state_1 = 0; // state of the isobath following


  x = Mat::zeros(4, 1, CV_64F);
  x.at<double>(0,0) = 0;
  x.at<double>(1,0) = 0;
  x.at<double>(2,0) = 0;
  x.at<double>(3,0) = 0;

  dx = Mat::zeros(4, 1, CV_64F);

  u = Mat::zeros(2, 1, CV_64F);
  y = Mat::zeros(3, 1, CV_64F);

  current_heading = 0;  //direct from the sensor
  sz_history = 100;
  mean_heading = 0;     //mean of the last 100 samples
  //begin_time = clock();
  begin_state1 = 0;
  //time_passed  = 0;
  //clock_fin = 0;
}

//---------------------------------------------------------
// Destructor

State1_isobath::~State1_isobath()
{
}


float State1_isobath::sawtooth(float i){
    //cout << "sawtooth = "<< fmod(i + M_PI , 2.0 * M_PI)-M_PI;
    return  fmod(i + M_PI , 2.0 * M_PI)-M_PI;
}


void State1_isobath::control(Mat y, Mat x, Mat phat, Mat* u){
    u->at<double>(0,0) =  (d0 - y.at<double>(2,0));
    u->at<double>(1,0) = tanh(-h0+phat.at<double>(2,0)) + sawtooth(atan2(phat.at<double>(1,0),phat.at<double>(0,0))+ (M_PI/2));
}

void State1_isobath::f(Mat x, Mat u, Mat* dx){
    dx->at<double>(0,0) =  cos(x.at<double>(3,0));
    dx->at<double>(1,0) = sin(x.at<double>(3,0));
    dx->at<double>(2,0) = u.at<double>(0,0);
    dx->at<double>(3,0) = u.at<double>(1,0);
}

void State1_isobath::euler(double dt, Mat dx, Mat * x){
    *x = *x + dx * dt;
}


//---------------------------------------------------------
// Procedure: OnNewMail

bool State1_isobath::OnNewMail(MOOSMSG_LIST &NewMail)
{
  MOOSMSG_LIST::iterator p;

  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
      CMOOSMsg &msg = *p;

    string key = msg.GetKey();

    //Capteur pression
    if (key == "AF_Depth"){
      y.at<double>(2,0) = msg.GetDouble() ;
      x.at<double>(2,0) = -msg.GetDouble();
    }

    if (key == "NAV_X"){
      x.at<double>(0,0) = msg.GetDouble();
    }
    if (key == "NAV_Y"){
      x.at<double>(1,0) = msg.GetDouble();
    }
    if (key == "NAV_HEADING"){
      x.at<double>(3,0) = msg.GetDouble();
    }
    if (key == "FA_Altitude"){
      y.at<double>(0,0) = msg.GetDouble() ;
    }
    if (key == "DEPTH_WANTED"){
      d0 = msg.GetDouble() ;
    }
    if (key == "ORDER"){
      h0 = msg.GetDouble() ;
    }

    if (key == "State_1"){
      state_1 = msg.GetDouble() ;
    }

    //heading history
    if (key == "HD_Heading"){
      current_heading = msg.GetDouble() ;
      heading_history.push(msg.GetDouble());
      if (heading_history.size() == (sz_history + 1)){
        heading_history.pop();
       }
    }

    //Kalman map estimation
    if (key == "PHAT_Kalman_0"){
      phat.at<double>(0,0) = msg.GetDouble() ;
    }
    if (key == "PHAT_Kalman_1"){
      phat.at<double>(1,0) = msg.GetDouble() ;
    }
    if (key == "PHAT_Kalman_2"){
      phat.at<double>(2,0) = msg.GetDouble() ;
      //printf("p_hat received in Controller : %f, %f, %f \n", phat.at<double>(0,0), phat.at<double>(1,0), phat.at<double>(2,0));
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

bool State1_isobath::OnConnectToServer()
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

bool State1_isobath::Iterate()
{
  if (state_1 == 1){
    //if (begin_state1 == 1){
      //begin_state1 = 0;
      //begin_time = clock();
//    }
    /****************************************/
    //File the queue of the last heading and compute the mean
    /******************************************/
    if (heading_history.size() != sz_history){
      mean_heading = current_heading;
    }

    // mean of the last 100 values
    else{
      mean_heading = 0; //init
      heading_history_tmp = heading_history; //copy of the queue

      for (i=0; i < sz_history; i++){  //sum of the elements of the queue
        mean_heading += heading_history_tmp.front();
        heading_history_tmp.pop();
      }
      mean_heading = mean_heading / sz_history;
    }


    /*******************************************/
    //compute the commande
    /*******************************************/
    hm = (y.at<double>(0, 0) + y.at<double>(2, 0));
    f(x,  u, &dx);
    euler(dt, dx, &x);
    control(y,x, phat, &u) ;

    //verify time passed since the beginning of state1
  //  time_passed = double( clock() - begin_time)/CLOCKS_PER_SEC;
//    if (time_passed > STATE1_PERIOD)
  //    clock_fin = 1;
  //  else
  //    clock_fin = 0;


    Notify("NAV_X",x.at<double>(0,0));
    Notify("NAV_Y",x.at<double>(1,0));
    Notify("NAV_HEADING",x.at<double>(3,0));
    Notify("Profondeur_tot",hm);
    Notify("Commande_U0",u.at<double>(0,0));
    Notify("Commande_U1",u.at<double>(1,0));
  //  Notify("Clock_State1",clock_fin);
    Notify("Current_heading_isobath", mean_heading);

  }

  return(true);
}
//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool State1_isobath::OnStartUp()
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

void State1_isobath::RegisterVariables()
{
  // Register("FOOBAR", 0);
  Register("AF_Depth", 0);
  Register("FA_Altitude", 0);
  Register("DEPTH_WANTED", 0);
  Register("PHAT_Kalman_0", 0);
  Register("PHAT_Kalman_1", 0);
  Register("PHAT_Kalman_2", 0);
  Register("State_1",0);
  Register("ORDER", 0);
}
