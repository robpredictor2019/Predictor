#include "robot.h"
#include "iostream"
#include <unistd.h>
using namespace std;

#define NOMBRE_ROBOT 1
#define TEMPS_ITERATION 150
#define DT 0.1


int main(int argc, char **argv){
  Gnuplot gp;
  Gnuplot Gamma;

  vector<Robot> List_robot;
  List_robot.reserve(100);
  //cout<<List_robot.capacity()<<endl;

  vector<point> plot;
  vector<point> p;
  vector<point> p_hat;
  //plot.reserve( (NOMBRE_ROBOT+1) * (TEMPS_ITERATION/DT) );

  std::ofstream fs;
  fs.open ("../Robot_States.txt", std::fstream::in | std::fstream::out | std::fstream::trunc);
  fs << "N="<<NOMBRE_ROBOT<<";T="<<TEMPS_ITERATION<<";dt="<<DT<<endl;

  for (int i=0; i<NOMBRE_ROBOT;i++){
    List_robot[i]=Robot(i,DT);
  }

  gp << "set xrange [-150:150]\n";
  gp << "set yrange [-150:150]\n";
  gp << "set ylabel \"y\"\n";
  gp << "set xlabel \"x\"\n";
  gp << "set linetype 1 linecolor rgb 'red'\n";
  gp << "set linetype 2 linecolor rgb 'blue'\n";
  gp << "set title 'Robot Position'\n";

  //gp << "plot";//for post calcul show

  for (int i=0;i<NOMBRE_ROBOT;i++){
    Robot robot = List_robot[i];
    for (int j=0; j<TEMPS_ITERATION/DT; j++){
      //cout<<"Temps :"<<robot.t<<endl;
      if (abs(robot.t-75.0)<0.15){
        cout<<"je change de direction !!!!!!!!!"<<endl;
        robot.C.at<double>(0,0)=1;
        robot.C.at<double>(1,1)=1;
        robot.Gbeta.at<double>(0,0) = pow(3,2);
        robot.Gbeta.at<double>(1,1) = pow(3,2);
        robot.theta_mission = 180.0;
      }
      robot.P_theta(); //Proportionnel pour
      robot.kalman_x(&robot.Gx_hat, &robot.x_hat);
      robot.evolution();
      robot.draw(&plot);
      robot.draw_x_y_hat(&p_hat); // for real time plot
      robot.draw_x_y(&p);
      //p = robot.draw_x_y(); //for post calcul show


      gp<<"plot '-' with  linespoint ls 1 points 0,"; //for real time plot
      gp<<" '-' with linespoint ls 2 points 0\n"; //for real time plot
      gp.send1d(p_hat); //for real time plot
      gp.send1d(p);

      //usleep(100000);//sleep for real time plot
      //cout<<j<<endl;

      //gp << gp.file1d(p)<<" notitle with linespoint ls 1,";//for post calcul show
      robot.save_state();
      if (abs(robot.t-75.0)<0.15){
        robot.C.at<double>(0,0)=0;
        robot.C.at<double>(1,1)=0;
        robot.Gbeta.at<double>(0,0) = 0;
        robot.Gbeta.at<double>(1,1) = 0;
      }
      //robot.t+=DT;
    }
    //gp << gp.file1d(p)<<" notitle with linespoint ls 1\n";
    List_robot[i] = robot;
  }

  cout<<"Simulation done\n";
  Gamma << "set title '|Gx|'\n";
  Gamma<<"plot '-' \n";
  Gamma.send1d(plot);

  for(int i=0; i<NOMBRE_ROBOT;i++){
    Robot robot = List_robot[i];
    robot.Export(fs);
  }
  cout<<"Export done\n";
  fs.close();

  return 0;
}
