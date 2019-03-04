#include "robot.h"
#include "iostream"
using namespace std;
#define NOMBRE_ROBOT=100
#define TEMPS_ITERATION=100
#define DT = 0.1

int main(int argc, char **argv){
  Robot robot;
  Gnuplot gp;

  //robot.InitValues();
  
  robot.kalman_x(x, Gx, u, y, Galpha, Gbeta, A, C, &Gx_out, &x_out)
  x = x_out;
  Gx = Gx_out;

  vector<Robot> List_robot;

  std::fstream fs;
  fs.open ("Robot_States.txt", std::fstream::in | std::fstream::out | std::fstream::app);
  fs<<"N="<<NOMBRE_ROBOT<<";T="<<TEMPS_ITERATION<<";dt="<<DT<<endl;

  for (int i=0; i<NOMBRE_ROBOT;i++){
    List_robot.push_back(Robot())
  }



  for (int j=0,j<=TEMPS_ITERATION * DT,j++){
    for (int i=0; i<NOMBRE_ROBOT; i++){

      robot = List_robot.at<Robot>(i)
      robot.evolution();
      robot.draw(gp);
      robot.save();
    }
  }
  for(int i=0; i<NOMBRE_ROBOT;i++){
    robot = List_robot.at<Robot>(i)
    robot.export(fs)

  }
  fs.close()

  return 0;

}
