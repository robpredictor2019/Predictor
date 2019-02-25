#include "robot.h"
#include "iostream"
using namespace std;


theta_p = Mat::eye(3, 3, CV_64FC1);
theta_p = theta_p.mul(100);

theta_beta = Mat::eye(1, 1, CV_64FC1);
theta_beta = theta_beta.mul(0.1);

theta_alpha = Mat::eye(3, 3, CV_64FC1);
theta_alpha = theta_alpha.mul(0.1);

y = Mat::zeros(2, 1, CV_64F);

Robot.A = Mat::zeros(3, 3, CV_64F)


int main(int argc, char **argv){
  Gnuplot gp;
  Robot robot;
  robot.A =
  robot.scenario();
}
