#include "robot.h"
#include "iostream"
using namespace std;

#define NOMBRE_ROBOT 1
#define TEMPS_ITERATION 100
#define DT 0.1
#define Agps 3.0

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
  gp << "set linetype 1 linecolor rgb 'blue'\n";
  gp << "plot";

  for (int i=0;i<NOMBRE_ROBOT;i++){
    Robot robot = List_robot[i];
    for (int j=0; j<TEMPS_ITERATION/DT; j++){

      if (robot.t==60 || robot.t==120){
        robot.x_out = robot.x;
        robot.C.at<double>(0,0)=1;
        robot.C.at<double>(1,1)=1;
        robot.Gx.at<double>(0,0) = Agps;
        robot.Gx.at<double>(0,1) = Agps;
        robot.Gbeta.at<double>(0,0) = pow(3,2);
        robot.Gbeta.at<double>(1,1) = pow(3,2);
      }
      robot.P_theta(); //Proportionnel pour
      robot.kalman_x( &robot.Gx_out, &robot.x_out);
      robot.x = robot.x_out;
      robot.Gx = robot.Gx_out;
      robot.evolution();
      robot.draw(&plot);
      //robot.draw_x_y(&plot);
      p = robot.draw_x_y();

      //gp<<"plot '-'\n";
      //gp.send1d(plot);
      gp << gp.file1d(p)<<" notitle with linespoint ls 1,";
      robot.save_state();
      if (robot.t==60 || robot.t==120){
        robot.C.at<double>(0,0)=0;
        robot.C.at<double>(1,1)=0;
        robot.Gbeta.at<double>(0,0) = pow(0.1,2);
        robot.Gbeta.at<double>(1,1) = pow(0.1,2);
      }
      //robot.t+=DT;
    }
    gp << gp.file1d(p)<<" notitle with linespoint ls 1\n";
    List_robot[i] = robot;
  }
  //cout<<"size "<<plot.size()<<endl;
  /*for (int i=0;i<plot.size();i++){
    cout<<i<<"("<<plot[i].first<<","<<plot[i].second<<")\n";
  }*/



  //gp << "set ylabel \"y\"\n";
  //gp << "set xlabel \"x\"\n";
  //gp<<"show xlabel\n";
  //Gamma<<"plot"<<Gamma.file1d(plot)<<" notitle\n'";
  Gamma<<"plot '-' \n";
  Gamma.send1d(plot);

  for(int i=0; i<NOMBRE_ROBOT;i++){
    Robot robot = List_robot[i];
    robot.Export(fs);
  }
  fs.close();

  return 0;
}
