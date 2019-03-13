#include "Tube.h"
#include "VibesFigure_Tube.h"
#include "math.h"

using namespace std;
using namespace ibex;
using namespace tubex;

//baser sur example 04


void displayCausalMap(const Tube& x, const Tube& y, int fig_x, int fig_y, const string& fig_name)
{
	const int slices_nb_to_display = 500;

	vibes::newFigure(fig_name);
	vibes::setFigureProperties(
		      vibesParams("figure", fig_name, "x", fig_x, "y", fig_y, "width", 900, "height", 600));
//	vibes::axisLimits(-10, 150, -10, 30);

	// Robot's tubes projection
	int startpoint;
	for(int i=0 ; i<x.size() ; i+=max((int)(x.size()/slices_nb_to_display), 1))
	 	startpoint = i;

	for(int i=0 ; i<startpoint; i+=max((int)(x.size()/slices_nb_to_display), 1))
	{
		Interval intv_x = x[i];
		Interval intv_y = y[i];
		if(!intv_x.is_unbounded() && !intv_y.is_unbounded())
			vibes::drawBox(intv_x.lb(), intv_x.ub(), intv_y.lb(), intv_y.ub(),
		               "lightGray[white]", vibesParams("figure", fig_name));
	}
}

const Interval h(const IntervalVector& meas, const Tube& x, const Interval& x0, const Interval& y0)
{
	/*computes yaw*/
	
	Interval angle_retours;
	
	//hypothese verite terrain:
	//angle_retours = Interval((178-1.15)*M_PI/180, (178+1.15)*M_PI/180); //angle 180
	
	//non VT
	Interval dist_x = x[meas[0].lb()] - x0;
	Interval dist_y = meas[1] - y0;
	angle_retours = M_PI + atan2(dist_y, dist_x);
	
	//cout <<" angle_ret deg : "<< 180/M_PI*angle_retours << endl;
	
	return angle_retours;
	
}

int main(int argc, char *argv[])
{
	//parametres robot
	Interval domain(0,120); //domain temps
	double timestep = 0.001;
	float speed = 1.;
	int alpha1 = 0, alpha2 = 0, alpha3 = 0;
	
	//initialisation des tubes commande
	Tube u(domain, timestep, Function("t", "-cos((t+33)/5)+[-0.02,0.02]"));
	const char *a0 = "[-0.02, 0.02]";
	Tube theta(domain, timestep, Function("t", a0));
	Interval domain3(double(60),domain.ub()); //domain temps
	Interval theta_end;
	theta.set(theta_end, domain3); 
	
	//CI
	Interval x0 = Interval(0.).inflate(1.);
	Interval y0 = x0;
	Interval theta0 = Interval((-6./5.)*M_PI).inflate(0.02);
	
	//equations	
	Tube xdot = speed * cos(theta) + alpha1;
	Tube ydot = speed * sin(theta) + alpha2;
	
	Tube x = xdot.primitive(x0);
	Tube y = ydot.primitive(y0);
	
	//slice à temps "GPS"	
   IntervalVector xtyt(2);
	xtyt[0] = x[60.000]; // [t]
   xtyt[1] = y[60.000]; // [y]
   
   //prise "GPS"	
	IntervalVector measurement(2);
	measurement[0] = Interval(60.000,60.001); // [t]
	double lb = xtyt[1].lb() + 0 * xtyt[1].diam()/3;
	double ub = xtyt[1].lb() + 1 * xtyt[1].diam()/3;
	measurement[1] = Interval(lb, ub); 	// [y] précision de l'incertitude

	//changement d'angle theta et propagation sur x et y
	bool contraction;
	int iter_cont = 0;
	do
	{
		Tube y_old = y;
		
		xdot &= speed * cos(theta) + alpha1; //se modifie
		ydot &= speed * sin(theta) + alpha2;
		
		// Applying a local contraction on y tube	
		y.ctcObs(ydot, measurement[0], measurement[1], false);
		theta.set(h(measurement, x, x0, y0), domain3);
		y.ctcFwd(ydot);
		x.ctcFwd(xdot);
		
		contraction = y != y_old; //contraction done
		y_old = y;
		iter_cont += 1;
		
	} while(contraction && iter_cont <= 1);
   
	//affichage
	string fig_name = "Map (top view): [x](·)x[y](·)";
	vibes::newFigure(fig_name); 
	//VibesFigure_Tube::show(&x, "Tube [x](·)", 100, 100);
	//VibesFigure_Tube::show(&y, "Tube [y](·)", 150, 120);
	//VibesFigure_Tube::show(&theta, "Tube [theta](·)", 150, 150);
	displayCausalMap(x, y, 500, 500, fig_name);
	

	
	
	//cout
	int index_m = y.size()-1;
	int index_t = 0;
	
	
	//show initial, resurface, final interval
   IntervalVector x0y0(2);
   x0y0[0] = x0; // [t]
   x0y0[1] = y0; // [y]
   IntervalVector xfinalyfinal(2);
   xfinalyfinal[0] = x[index_m]; // [t]
   xfinalyfinal[1] = y[index_m]; // [y]
   IntervalVector xgpsygps(2);
   xgpsygps[0] = x[measurement[0].lb()]; // [t]
   xgpsygps[1] = y[measurement[0].lb()]; // [y]
	
   vibes::drawBox(x0y0, "blue", vibesParams("figure", fig_name));
   vibes::drawBox(xtyt, "green", vibesParams("figure", fig_name));
   vibes::drawBox(xgpsygps, "darkGreen[darkGreen]", vibesParams("figure", fig_name));
   vibes::drawBox(xfinalyfinal, "red", vibesParams("figure", fig_name));
 
   
	//fin 
	VibesFigure_Tube::endDrawing();
}
