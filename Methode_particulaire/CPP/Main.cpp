#include "robot.h"
#include "iostream"
#include <unistd.h>
using namespace std;

#define NOMBRE_ROBOT 1
#define TEMPS_ITERATION 100
#define DT 1


int main(int argc, char **argv){
  Gnuplot gp;
  Gnuplot Gamma;

  vector<Robot> List_robot;
  List_robot.reserve(100);
  //cout<<List_robot.capacity()<<endl;

  vector<point> plot;
  vector<point> p;
  //plot.reserve( (NOMBRE_ROBOT+1) * (TEMPS_ITERATION/DT) );

  std::ofstream fs;
  fs.open ("../Robot_States.txt", std::fstream::in | std::fstream::out | std::fstream::trunc);
  fs << "N="<<NOMBRE_ROBOT<<";T="<<TEMPS_ITERATION<<";dt="<<DT<<endl;

  for (int i=0; i<NOMBRE_ROBOT;i++){
    List_robot[i]=Robot(i,DT);
  }

  gp << "set xrange [-10:10]\n";
  gp << "set yrange [-5:5]\n";
  gp << "set ylabel \"y\"\n";
  gp << "set xlabel \"x\"\n";
  gp << "set title 'Robot Position'\n";

  //gp << "plot";//for post calcul show

  for (int i=0;i<NOMBRE_ROBOT;i++){
    Robot robot = List_robot[i];
    for (int j=0; j<TEMPS_ITERATION/DT; j++){
      
      if (robot.t==60 || robot.t==120){
        robot.C.at<double>(0,0)=1;
        robot.C.at<double>(1,1)=1;
        robot.Gbeta.at<double>(0,0) = pow(3,2);
        robot.Gbeta.at<double>(1,1) = pow(3,2);
        robot.theta_bar = robot.theta;
      }
      //robot.P_theta(); //Proportionnel pour
      robot.kalman_x( &robot.Gx_out, &robot.x_out);
      robot.x = robot.x_out;
      robot.Gx = robot.Gx_out;
      robot.evolution();
      robot.draw(&plot);
      robot.draw_x_y(&p); // for real time plot
      //p = robot.draw_x_y(); //for post calcul show


      gp<<"plot '-'\n"; //for real time plot
      gp.send1d(p); //for real time plot
      //usleep(100000);//sleep for real time plot
      //cout<<j<<endl;

      //gp << gp.file1d(p)<<" notitle with linespoint ls 1,";//for post calcul show
      robot.save_state();
      if (robot.t==60 || robot.t==120){
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
