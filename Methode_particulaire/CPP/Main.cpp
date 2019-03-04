#include "robot.h"
#include "iostream"
using namespace std;
#define NOMBRE_ROBOT 40
#define TEMPS_ITERATION 10
#define DT 0.1

int main(int argc, char **argv){
  Gnuplot gp;
  vector<Robot>  List_robot;
  List_robot.reserve(150);
  cout<<List_robot.capacity()<<endl;
  vector<point> plot;
  //plot.reserve( (NOMBRE_ROBOT+1) * (TEMPS_ITERATION/DT) );

  std::ofstream fs;
  fs.open ("../Robot_States.txt", std::fstream::in | std::fstream::out | std::fstream::app);
  fs << "N="<<NOMBRE_ROBOT<<";T="<<TEMPS_ITERATION<<";dt="<<DT<<endl;

  for (int i=0; i<NOMBRE_ROBOT;i++){
    List_robot[i]=Robot(i);
  }

  for (int i=0;i<NOMBRE_ROBOT;i++){
    Robot robot = List_robot[i];
    for (int j=0; j<TEMPS_ITERATION/DT; j++){
      robot.kalman_x( &robot.Gx_out, &robot.x_out);
      robot.x = robot.x_out;
      robot.Gx = robot.Gx_out;
      robot.draw(&plot);
      //robot.save_state();
      robot.t+=DT;
    }
    //List_robot[i] = robot;
  }
  cout<<"size "<<plot.size()<<endl;
  /*for (int i=0;i<plot.size();i++){
    cout<<i<<"("<<plot[i].first<<","<<plot[i].second<<")\n";
  }*/

  gp<<"plot '-'\n";
  gp.send1d(plot);
  /*for(int i=0; i<NOMBRE_ROBOT;i++){
    Robot robot = List_robot[i];
    robot.Export(fs);
  }*/
  fs.close();

  return 0;
}
