/************************************************************/
/*    NAME: COSTA Maria                                              */
/*    ORGN: MIT                                             */
/*    FILE: kalman.cpp                                        */
/*    DATE:                                                 */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "Kalman.h"
#include <math.h>

using namespace std;
using namespace cv;

//---------------------------------------------------------
// Constructor

Kalman::Kalman()
{
  dt = 0.05;
  d0 = 0.0;

  u = Mat::zeros(2, 1, CV_64F);
  phat = Mat::zeros(3, 1, CV_64F);

  C = Mat::zeros(1, 3, CV_64F);
  C.at<double>(0,0) = 0;
  C.at<double>(0,1) = 0;
  C.at<double>(0,2) = 1;

  A = Mat::zeros(3, 3, CV_64F);
  A.at<double>(0,0) = 1 ;
  A.at<double>(0,1) = dt * u.at<double>(1,0);
  A.at<double>(0,2) = 0;

  A.at<double>(1,0) = -dt * u.at<double>(1,0) ;
  A.at<double>(1,1) = 1;
  A.at<double>(1,2) = 0;

  A.at<double>(2,0) = dt ;
  A.at<double>(2,1) = 0;
  A.at<double>(2,2) = 1;


  //Pour le calcul du Kalman
  theta_p = Mat::eye(3, 3, CV_64FC1);
  theta_p = theta_p.mul(100);
  theta_beta = Mat::eye(1, 1, CV_64FC1);
  theta_beta = theta_beta.mul(0.1);
  theta_alpha = Mat::eye(3, 3, CV_64FC1);
  theta_alpha = theta_alpha.mul(0.1);

  y = Mat::zeros(3, 1, CV_64F);

  cap_command = u.at<double>(1,0)*100/(M_PI + 1.0);

  count_command_test = 0;
  count_mag = 4;

  command_test = 0;

  hm_mat = Mat::zeros(1,1,CV_64F);

}

//---------------------------------------------------------
// Destructor

Kalman::~Kalman()
{
}

//---------------------------------------------------------

void Kalman::kalman_predict(Mat xup_k,Mat Pup_k, Mat Q, Mat A, Mat u, Mat* x_k1, Mat* P_k1){
  *P_k1 = (A*Pup_k*A.t()) + Q;
  *x_k1 = A*xup_k + u;
}

void Kalman::kalman_correct(Mat x_k1, Mat P_k1, Mat  C, Mat R, Mat y, Mat* xup_k1, Mat* Pup_k1){
   Mat S = Mat::zeros(Size(SIZEY,SIZEY),CV_64F);
   Mat K = Mat::zeros(Size(SIZEX,SIZEY),CV_64F);
   Mat err = Mat::zeros(SIZEY,1,CV_64F);

   S = (C*P_k1*C.t()) + R;
   K = P_k1*C.t()*S.inv();
   err = y - (C*x_k1);
   *Pup_k1 = (Mat::eye(SIZEX,SIZEX,CV_64F) - (K*C)) * P_k1;
   *xup_k1 = x_k1 + K*err;
}

void Kalman::kalman_x(Mat x_k,Mat P_k,Mat u,Mat y,Mat Q, Mat R,Mat A ,Mat C, Mat* P_k1, Mat* x_k1){
  Mat Pup_k = Mat::zeros(Size(SIZEX,SIZEX),CV_64F);
  Mat xup_k = Mat::zeros(Size(SIZEX,1),CV_64F);

  kalman_correct(x_k,P_k, C,R, y, &xup_k, &Pup_k);
  kalman_predict(xup_k,Pup_k, Q, A,u,x_k1,P_k1);
}


// Procedure: OnNewMail

bool Kalman::OnNewMail(MOOSMSG_LIST &NewMail)
{

  MOOSMSG_LIST::iterator p;

  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
 	  CMOOSMsg &msg = *p;

	  string key = msg.GetKey();
	  /*
	  if (key == "AF_Depth"){
	    y.at<double>(2,0) = msg.GetDouble() ;
	  }
	  if (key == "FA_Altitude"){
	    y.at<double>(0,0) = msg.GetDouble() ;
	  }

	  if (key == "ORDER"){
	    h0 = msg.GetDouble() ;
	  }


	  if (key == "DEPTH_WANTED"){z
	    d0 = msg.GetDouble() ;
	  }*/

	  if (key == "Commande_U0"){
	    u.at<double>(0,0) = msg.GetDouble() ;
	  }

	  if (key == "Commande_U1"){
	    u.at<double>(1,0) = msg.GetDouble() ;
	  }

	  if (key == "Profondeur_tot"){
	    hm_mat.at<double>(0,0) =  msg.GetDouble() ;
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

bool Kalman::OnConnectToServer()
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

bool Kalman::Iterate()
{
  A.at<double>(0,0) = 1 ;
  A.at<double>(0,1) = dt * u.at<double>(1,0);
  A.at<double>(0,2) = 0;

  A.at<double>(1,0) = -dt * u.at<double>(1,0) ;
  A.at<double>(1,1) = 1;
  A.at<double>(1,2) = 0;

  A.at<double>(2,0) = dt ;
  A.at<double>(2,1) = 0;
  A.at<double>(2,2) = 1;

  kalman_x(phat,theta_p,Mat::zeros(3,1,CV_64F),hm_mat,theta_alpha, theta_beta,A ,C, &theta_p_out, &phat_out);
  phat = phat_out;
  theta_p = theta_p_out;

  cout << "=========================================\n\n";
  cout << "phat : \n" << phat << "\n\n";
  cout << "theta_p : \n" << theta_p << "\n\n";
  cout << "hm_mat : \n" << hm_mat << "\n\n";
  cout << "theta_alpha : \n" << theta_alpha << "\n\n";
  cout << "theta_beta : \n" << theta_beta << "\n\n";
  cout << "A : \n" << A << "\n\n";
  cout << "C : \n" << C << "\n\n";

  printf("phat_from_kalman = (%f, %f, %f) \n",phat.at<double>(0,0) ,phat.at<double>(1, 0) ,phat.at<double>(2,0));
  Notify("PHAT_Kalman_0",phat.at<double>(0,0));
  Notify("PHAT_Kalman_1",phat.at<double>(1,0));
  Notify("PHAT_Kalman_2",phat.at<double>(2,0));


  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool Kalman::OnStartUp()
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

void Kalman::RegisterVariables()
{
  // Register("FOOBAR", 0);
 /* Register("FA_Altitude", 0);
  Register("AF_Depth", 0);
  Register("ORDER", 0);
  Register("DEPTH_WANTED", 0);*/
  Register("Commande_U0", 0);
  Register("Commande_U1", 0);
  Register("Profondeur_tot", 0);


}
