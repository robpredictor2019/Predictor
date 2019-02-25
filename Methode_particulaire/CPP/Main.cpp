#include "robot.h"
#include "iostream"
using namespace std;
#define NOMBREROBOT=100
#define NOMBREITERATION=1000

int main(int argc, char **argv){
  Robot robot;
  Gnuplot gp;

  robot.InitValues();
  
  robot.kalman_x(x, Gx, u, y, Galpha, Gbeta, A, C, &Gx_out, &x_out)
  x = x_out;
  Gx = Gx_out;


  vector<Robot> List_robot;

  std::fstream fs;
  fs.open ("Robot_States.txt", std::fstream::in | std::fstream::out | std::fstream::app);

  for (int i=0; i<100;i++){
    List_robot.push_back(Robot())
  }

  for (int j=0,j<=nombreIteration,j++){
    for (int i=0; i<nombreRobot; i++){
      robot = List_robot.at<Robot>(i)
      robot.evolution();
      robot.draw(gp);
      robot.save();
    }
  }
  for(int i=0; i<nombreRobot;i++){
    robot = List_robot.at<Robot>(i)
    robot.export(fs)

  }
  fs.close()

  return 0;

}
