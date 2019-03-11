#include "robot.h"
#include "iostream"
#include <unistd.h>
using namespace std;

#define NOMBRE_ROBOT 100
#define TEMPS_ITERATION 245
#define DT 0.1


int main(int argc, char **argv){
  Gnuplot gp;
  Gnuplot Gamma;

  vector<Robot> List_robot;
  List_robot.reserve(100);
  //cout<<List_robot.capacity()<<endl;
  double radius = 5;
  bool inzone(0);

  vector<point> amer;

  amer.push_back(point(0,0));
  amer.push_back(point(50,0));
  amer.push_back(point(25,25));

  int amer_number(1);

  vector<point> plot_x;
  vector<point> plot_y;
  vector<point> plot_xy;
  vector<point> p;
  vector<point> p_hat;
  vector<point> Cercle_0;
  vector<point> Cercle_1;
  vector<point> Cercle_2;

  Cercle_0 = circle(amer.at(0),radius);
  Cercle_1 = circle(amer.at(1),radius);
  Cercle_2 = circle(amer.at(2),radius);

  std::ofstream fs;
  fs.open ("../Robot_States.txt", std::fstream::in | std::fstream::out | std::fstream::trunc);
  fs << "N="<<NOMBRE_ROBOT<<";T="<<TEMPS_ITERATION<<";dt="<<DT<<endl;

  for (int i=0; i<NOMBRE_ROBOT;i++){
    List_robot[i]=Robot(i,DT);
  }

  gp << "set xrange [-10:60]\n";
  gp << "set yrange [-10:60]\n";
  gp << "set ylabel \"y\"\n";
  gp << "set xlabel \"x\"\n";
  gp << "set linetype 1 linecolor rgb 'red'\n";
  gp << "set linetype 2 linecolor rgb 'blue'\n";
  gp << "set linetype 3 linecolor rgb 'black'\n";
  gp << "set title 'Robot Position'\n";


  for (int i=0;i<NOMBRE_ROBOT;i++){
    Robot robot = List_robot[i];
    amer_number = 1;
    for (int j=0; j<TEMPS_ITERATION/DT-1; j++){
      if (robot.distance(amer[amer_number])<radius){
        //cout<<"je change de direction !!!!!!!!!"<<endl;
        robot.C.at<double>(0,0)=1;
        robot.C.at<double>(1,1)=1;
        robot.Gbeta.at<double>(0,0) = pow(0.48,2);
        robot.Gbeta.at<double>(1,1) = pow(0.48,2);
        robot.y.at<double>(0) = robot.x.at<double>(0);
        robot.y.at<double>(1) = robot.x.at<double>(1);
        if (not(inzone)){
          inzone = 1;
          amer_number++;
          if (amer_number>=amer.size()){
            amer_number = 0; 
          }
        }
      }
      robot.P_theta(amer[amer_number]); //Proportionnel pour
      robot.kalman_x(&robot.Gx_hat, &robot.x_hat);
      robot.evolution();

      robot.draw_x_y_hat(&p_hat);
      robot.draw_x_y(&p);

      /*if (j%20==0){ //Real Time Plot
        gp<<"plot '-' title 'Kalman'  with linespoint ls 1 points 0,"; //for real time plot
        gp<<" '-' title 'Real' with linespoint ls 2 points 0,"; //for real time plot
        gp<<" '-' notitle with linespoint ls 3 points 0,"; //for real time plot
        gp<<" '-' notitle with linespoint ls 3 points 0,"; //for real time plot
        gp<<" '-' notitle with linespoint ls 3 points 0\n"; //for real time plot
        gp.send1d(p_hat); //for real time plot
        gp.send1d(p);
        gp.send(Cercle_0);
        gp.send(Cercle_1);
        gp.send(Cercle_2);
      }*/
      robot.save_state();

      if (inzone && robot.distance(amer[amer_number])>radius){
        inzone = 0;
        robot.C.at<double>(0,0)=0;
        robot.C.at<double>(1,1)=0;
        robot.Gbeta.at<double>(0,0) = 0;
        robot.Gbeta.at<double>(1,1) = 0;
      }
    }
    List_robot[i] = robot;
  }

  covar_particule(&plot_x,&plot_y,&plot_xy,List_robot);

  //Post show
  gp<<"plot '-' title 'Real'  with linespoint ls 2 points 0,";
  gp<<" '-' title 'Kalman' with linespoint ls 1 points 0,"; 
  gp<<" '-' notitle with linespoint ls 3 points 0,"; 
  gp<<" '-' notitle with linespoint ls 3 points 0,"; 
  gp<<" '-' notitle with linespoint ls 3 points 0\n"; 
  gp.send1d(p);
  gp.send1d(p_hat);
  gp.send(Cercle_0);
  gp.send(Cercle_1);
  gp.send(Cercle_2);

  cout<<"Simulation done\n";
  Gamma << "set title 'Sigma'\n";
  Gamma<<"plot '-' title 'Sigma_x' with linespoint ls 1 points 0,";
  Gamma<<" '-' title 'Sigma_y' with linespoint ls 2 points 0,";
  Gamma<<" '-' title 'Sigma_x^2+Sigma_y^2' with linespoint ls 3 points 0\n";
  Gamma.send1d(plot_x);
  Gamma.send1d(plot_y);
  Gamma.send1d(plot_xy);

  for(int i=0; i<NOMBRE_ROBOT;i++){
    Robot robot = List_robot[i];
    robot.Export(fs);
  }
  cout<<"Export done\n";
  fs.close();

  sleep(1000);
  return 0;
}
