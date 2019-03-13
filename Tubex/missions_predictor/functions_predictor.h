#ifndef FUNCTIONS_PREDICTOR_H_INCLUDED
#define FUNCTIONS_PREDICTOR_H_INCLUDED


#include <string>
#include "Tube.h"
#include "VibesFigure_Tube.h"
#include "math.h"

void displayCausalMap(const tubex::Tube& x, const tubex::Tube& y, int fig_x, int fig_y, const std::string& fig_name, const std::string& color);
/*display tube x by tube y*/

void afficheWP(const std::vector<ibex::IntervalVector>& iles, const std::string& fig_name);
/*show iles boxes*/

void afficheWP(const std::vector<ibex::IntervalVector>& iles, const std::vector<ibex::IntervalVector>& zones_iles, const std::string& fig_name);
/*show iles boxes and corona boxes*/

void affichePOI(const tubex::Tube& x, const tubex::Tube& y, const std::vector<double>& t_iles, const ibex::IntervalVector& x0y0, const std::string& fig_name);
/*show interest boxes*/

const ibex::Interval calcul_angle(const ibex::IntervalVector& boite1, const ibex::IntervalVector& boite2);
/*computes the angle (in rad) to go from boite1 to boite2*/

double temps_visu_ile(int index_m, const tubex::Tube& x, const tubex::Tube& y, const ibex::IntervalVector& ile);
/*get the first time the robot is in the island's viewable zone*/

double temps_ile(int index_m, const tubex::Tube& x, const tubex::Tube& y, const ibex::IntervalVector& ile);
/*get the first time the robot is centered around the island*/

void propagation(tubex::Tube& xf, tubex::Tube& yf, const ibex::Interval& domain, const double& timestep, const ibex::IntervalVector& boite, const ibex::IntervalVector& boite_cible, const int& alpha1, const int& alpha2, const int& speed, const std::string& fig_name);
/*compute the union of tubes created from sub-boxes coming from boite*/

void egalite(tubex::Tube& x, tubex::Tube& x2, const double& t_f, const double& timestep);
/*replace tube x's data with tubes x2's data before time t_f*/

#endif //FUNCTIONS_PREDICTOR_H_INCLUDED
