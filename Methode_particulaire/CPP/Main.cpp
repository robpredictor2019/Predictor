#include "robot.h"
#include "iostream"
using namespace std;

int main(int argc, char **argv){
  Robot robot;
  Gnuplot gp;
  robot.InitValues();
  
  robot.kalman_x(x, Gx, u, y, Galpha, Gbeta, A, C, &Gx_out, &x_out)
  x = x_out;
  Gx = Gx_out;

  vector<Robot> List_robot;
  for (int i=0; i<100;i++){
    List_robot.push_back(Robot())
  }


  for (int j=1,j<=1000,j++1){
    for (int i=1; i<100; i++1){
      robot = List_robot.at<Robot>(i)

      robot.scenario();
      robot.draw(gp);
    }
  }
}
