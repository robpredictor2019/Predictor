#include "robot.h"
#include "iostream"
using namespace std;
#define NOMBRE_ROBOT 100
#define TEMPS_ITERATION 100
#define DT 0.1

int main(int argc, char **argv){
  Gnuplot gp;




  vector<Robot> List_robot;

  std::ofstream fs;
  fs.open ("Robot_States.txt", std::fstream::in | std::fstream::out | std::fstream::app);
  fs << "N="<<NOMBRE_ROBOT<<";T="<<TEMPS_ITERATION<<";dt="<<DT<<endl;

  for (int i=0; i<NOMBRE_ROBOT;i++){
    List_robot.push_back(Robot());
  }



  for (int j=0;j<=TEMPS_ITERATION * DT;j++){
    for (int i=0; i<NOMBRE_ROBOT; i++){

      Robot robot = List_robot.at(i);
      robot.kalman_x( &robot.Gx_out, &robot.x_out);
      robot.x = robot.x_out;
      robot.Gx = robot.Gx_out;
      robot.draw(gp);
      robot.save_state();
    }
  }
  for(int i=0; i<NOMBRE_ROBOT;i++){
    Robot robot = List_robot[i];
    robot.Export(fs);

  }
  fs.close();

  return 0;

}
