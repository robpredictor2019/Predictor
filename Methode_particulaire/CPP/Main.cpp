#include "robot.h"
#include "iostream"
using namespace std;

int main(int argc, char **argv){
  Gnuplot gp;

  for (j=1,j<=1000,j+=1){
    for (i=1; i<=100; i+=1){
      Robot robot;
      Robot.scenario();
      Robot.draw(gp);

    }
  }


}
